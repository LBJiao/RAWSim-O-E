﻿using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.IO;
using RAWSimO.Core.Items;
using RAWSimO.Core.Metrics;
using RAWSimO.Toolbox;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using RAWSimO.SolverWrappers;

namespace RAWSimO.Core.Control.Defaults.OrderBatching
{
    /// <summary>
    /// Implements a manager that uses information of the backlog to exploit similarities in orders when assigning them.
    /// </summary>
    public class PodMatchingOrderManager : OrderManager
    {
        /// <summary>
        /// Creates a new instance of this manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public PodMatchingOrderManager(Instance instance) : base(instance) { _config = instance.ControllerConfig.OrderBatchingConfig as PodMatchingOrderBatchingConfiguration; }

        /// <summary>
        /// The config of this controller.
        /// </summary>
        private PodMatchingOrderBatchingConfiguration _config;
        /// <summary>
        /// order进入Od的截止时间
        /// </summary>
        private double DueTimeOrderofMP = TimeSpan.FromMinutes(30).TotalSeconds;
        /// <summary>
        /// Checks whether another order is assignable to the given station.
        /// </summary>
        /// <param name="station">The station to check.</param>
        /// <returns><code>true</code> if there is another open slot, <code>false</code> otherwise.</returns>
        private bool IsAssignable(OutputStation station)
        { return station.Active && station.CapacityReserved + station.CapacityInUse < station.Capacity; }
        /// <summary>
        /// Checks whether another order is assignable to the given station.
        /// </summary>
        /// <param name="station">The station to check.</param>
        /// <returns><code>true</code> if there is another open slot and another one reserved for fast-lane, <code>false</code> otherwise.</returns>
        private bool IsAssignableKeepFastLaneSlot(OutputStation station)
        { return station.Active && station.CapacityReserved + station.CapacityInUse < station.Capacity; }

        private BestCandidateSelector _bestCandidateSelectNormal;
        private BestCandidateSelector _bestCandidateSelectFastLane;
        private OutputStation _currentStation = null;
        private Order _currentOrder = null;
        private VolatileIDDictionary<OutputStation, Pod> _nearestInboundPod;

        /// <summary>
        /// Initializes this controller.
        /// </summary>
        private void Initialize()
        {
            // Set some values for statistics
            _statPodMatchingScoreIndex = _config.LateBeforeMatch ? 1 : 0;
            // --> Setup normal scorers
            List<Func<double>> normalScorers = new List<Func<double>>();
            // Select late orders first
            if (_config.LateBeforeMatch)
            {
                normalScorers.Add(() =>
                {
                    return _currentOrder.DueTime > Instance.Controller.CurrentTime ? 1 : 0;
                });
            }
            // Select best by match with inbound pods
            normalScorers.Add(() =>
            {
                return _currentOrder.Positions.Sum(line => Math.Min(_currentStation.InboundPods.Sum(pod => pod.CountAvailable(line.Key)), line.Value));
            });
            // If we run into ties use the oldest order
            normalScorers.Add(() =>
            {
                switch (_config.TieBreaker)
                {
                    case Shared.OrderSelectionTieBreaker.Random: return Instance.Randomizer.NextDouble();
                    case Shared.OrderSelectionTieBreaker.EarliestDueTime: return -_currentOrder.DueTime;
                    case Shared.OrderSelectionTieBreaker.FCFS: return -_currentOrder.TimeStamp;
                    default: throw new ArgumentException("Unknown tie breaker: " + _config.FastLaneTieBreaker);
                }
            });
            // --> Setup fast lane scorers
            List<Func<double>> fastLaneScorers = new List<Func<double>>();
            // If we run into ties use the oldest order
            fastLaneScorers.Add(() =>
            {
                switch (_config.FastLaneTieBreaker)
                {
                    case Shared.FastLaneTieBreaker.Random: return Instance.Randomizer.NextDouble();
                    case Shared.FastLaneTieBreaker.EarliestDueTime: return -_currentOrder.DueTime;
                    case Shared.FastLaneTieBreaker.FCFS: return -_currentOrder.TimeStamp;
                    default: throw new ArgumentException("Unknown tie breaker: " + _config.FastLaneTieBreaker);
                }
            });
            // Init selectors
            _bestCandidateSelectNormal = new BestCandidateSelector(true, normalScorers.ToArray());
            _bestCandidateSelectFastLane = new BestCandidateSelector(true, fastLaneScorers.ToArray());
            if (_config.FastLane)
                _nearestInboundPod = new VolatileIDDictionary<OutputStation, Pod>(Instance.OutputStations.Select(s => new VolatileKeyValuePair<OutputStation, Pod>(s, null)).ToList());
        }
        /// <summary>
        /// Prepares some meta information.
        /// </summary>
        private void PrepareAssessment()
        {
            if (_config.FastLane)
            {
                foreach (var station in Instance.OutputStations.Where(s => IsAssignable(s)))
                {
                    _nearestInboundPod[station] = station.InboundPods.ArgMin(p =>
                    {
                        if (p.Bot != null && p.Bot.CurrentWaypoint != null)
                            // Use the path distance (this should always be possible)
                            return Distances.CalculateShortestPathPodSafe(p.Bot.CurrentWaypoint, station.Waypoint, Instance);
                        else
                            // Use manhattan distance as a fallback
                            return Distances.CalculateManhattan(p, station, Instance.WrongTierPenaltyDistance);
                    });
                }
            }
        }
        /// <summary>
        /// 产生Od
        /// </summary>
        /// <param name="pendingOrders"></param>
        /// <returns></returns>
        public HashSet<Order> GenerateOd(HashSet<Order> pendingOrders)
        {
            HashSet<Order> Od = new HashSet<Order>();
            foreach (Order order in pendingOrders)
                order.Timestay = order.DueTime - (Instance.SettingConfig.StartTime.AddSeconds(Convert.ToInt32(Instance.Controller.CurrentTime)) - order.TimePlaced).TotalSeconds;
            foreach (var order in pendingOrders.Where(v => v.Positions.Sum(line => Math.Min(Instance.ResourceManager.UnusedPods.Sum(pod => pod.CountAvailable(line.Key)), line.Value))
            == v.Positions.Sum(s => s.Value)))//保证Od中的所有order必须能被执行
            {
                if (order.Timestay < DueTimeOrderofMP)
                    Od.Add(order);
            }
            return Od;
        }
        /// <summary>
        /// This is called to decide about potentially pending orders.
        /// This method is being timed for statistical purposes and is also ONLY called when <code>SituationInvestigated</code> is <code>false</code>.
        /// Hence, set the field accordingly to react on events not tracked by this outer skeleton.
        /// </summary>
        protected override void DecideAboutPendingOrders()
        {
            // If not initialized, do it now
            if (_bestCandidateSelectNormal == null)
                Initialize();
            // Define filter functions
            Func<OutputStation, bool> validStationNormalAssignment = _config.FastLane ? (Func<OutputStation, bool>)IsAssignableKeepFastLaneSlot : IsAssignable;
            Func<OutputStation, bool> validStationFastLaneAssignment = IsAssignable;
            //Od为快到期order的集合
            HashSet<Order> Od = GenerateOd(_pendingOrders);
            // Assign fast lane orders while possible
            bool furtherOptions = true;
            while (furtherOptions && Od.Count > 0)
            {
                // Prepare helpers
                OutputStation chosenStation = null;
                Order chosenOrder = null;
                _bestCandidateSelectFastLane.Recycle();
                // Look for next station to assign orders to
                foreach (var station in Instance.OutputStations
                    // Station has to be valid
                    .Where(s => validStationFastLaneAssignment(s)))
                {
                    // Set station
                    _currentStation = station;
                    // Search for best order for the station in all fulfillable orders
                    foreach (var order in Od.Where(o =>
                        // Order needs to be immediately fulfillable
                        o.Positions.All(p => station.InboundPods.Any(v => v.CountAvailable(p.Key) >= p.Value) || Instance.ResourceManager.UnusedPods.Any(v => v.CountAvailable(p.Key) >= p.Value))))
                    {
                        // Set order
                        _currentOrder = order;
                        // --> Assess combination
                        if (_bestCandidateSelectFastLane.Reassess())
                        {
                            chosenStation = _currentStation;
                            chosenOrder = _currentOrder;
                        }
                    }
                }
                // Assign best order if available
                if (chosenOrder != null)
                {
                    // Assign the order
                    AllocateOrder(chosenOrder, chosenStation);
                    Od.Remove(chosenOrder);
                    // Log fast lane assignment
                    Instance.StatCustomControllerInfo.CustomLogOB1++;
                }
                else
                {
                    // No more options to assign orders to stations
                    furtherOptions = false;
                }
            }
            // Assign orders while possible
            furtherOptions = true;
            while (furtherOptions)
            {
                // Prepare helpers
                OutputStation chosenStation = null;
                Order chosenOrder = null;
                _bestCandidateSelectNormal.Recycle();
                // Look for next station to assign orders to
                foreach (var station in Instance.OutputStations
                    // Station has to be valid
                    .Where(s => validStationNormalAssignment(s)))
                {
                    // Set station
                    _currentStation = station;
                    // Search for best order for the station in all fulfillable orders        选择的订单必须满足库存的数量约束
                    foreach (var order in _pendingOrders.Where(o => o.Positions.All(p => Instance.StockInfo.GetActualStock(p.Key) >= p.Value)))
                    {
                        // Set order
                        _currentOrder = order;
                        // --> Assess combination
                        if (_bestCandidateSelectNormal.Reassess())
                        {
                            chosenStation = _currentStation;
                            chosenOrder = _currentOrder;
                        }
                    }
                }
                // Assign best order if available
                if (chosenOrder != null)
                {
                    // Assign the order
                    AllocateOrder(chosenOrder, chosenStation);
                    // Log score statistics
                    if (_statScorerValues == null)
                        _statScorerValues = _bestCandidateSelectNormal.BestScores.ToArray();
                    else
                        for (int i = 0; i < _bestCandidateSelectNormal.BestScores.Length; i++)
                            _statScorerValues[i] += _bestCandidateSelectNormal.BestScores[i];
                    _statAssignments++;
                    Instance.StatCustomControllerInfo.CustomLogOB2 = _statScorerValues[_statPodMatchingScoreIndex] / _statAssignments;
                }
                else
                {
                    // No more options to assign orders to stations
                    furtherOptions = false;
                }
            }
        }

        #region IOptimize Members

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public override void SignalCurrentTime(double currentTime) { /* Ignore since this simple manager is always ready. */ }

        #endregion

        #region Custom stat tracking

        /// <summary>
        /// Contains the aggregated scorer values.
        /// </summary>
        private double[] _statScorerValues = null;
        /// <summary>
        /// Contains the number of assignments done.
        /// </summary>
        private double _statAssignments = 0;
        /// <summary>
        /// The index of the pod matching scorer.
        /// </summary>
        private int _statPodMatchingScoreIndex = -1;
        /// <summary>
        /// The callback indicates a reset of the statistics.
        /// </summary>
        public override void StatReset()
        {
            _statScorerValues = null;
            _statAssignments = 0;
        }
        /// <summary>
        /// The callback that indicates that the simulation is finished and statistics have to submitted to the instance.
        /// </summary>
        public override void StatFinish()
        {
            Instance.StatCustomControllerInfo.CustomLogOBString =
                _statScorerValues == null ? "" :
                string.Join(IOConstants.DELIMITER_CUSTOM_CONTROLLER_FOOTPRINT.ToString(), _statScorerValues.Select(e => e / _statAssignments).Select(e => e.ToString(IOConstants.FORMATTER)));
        }

        #endregion
    }
}
