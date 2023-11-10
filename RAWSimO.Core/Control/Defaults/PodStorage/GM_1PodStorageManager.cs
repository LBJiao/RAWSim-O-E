using RAWSimO.Core.Configurations;
using RAWSimO.Core.Elements;
using RAWSimO.Core.Geometrics;
using RAWSimO.Core.Interfaces;
using RAWSimO.Core.Metrics;
using RAWSimO.Core.Waypoints;
using RAWSimO.SolverWrappers;
using static RAWSimO.Core.Management.ResourceManager;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.Core.Control.Defaults.PodStorage
{
    /// <summary>
    /// Defines by which distance measure the free storage location is selected.
    /// </summary>
    public enum GM_1PodStorageLocationDisposeRule
    {
        /// <summary>
        /// Uses the euclidean distance measure to find the next free storage location.
        /// </summary>
        Euclid,
        /// <summary>
        /// Uses the manhattan distance measure to find the next free storage location.
        /// </summary>
        Manhattan,
        /// <summary>
        /// Uses the shortest path (calculated by A*) to find the next free storage location.
        /// </summary>
        ShortestPath,
        /// <summary>
        /// Uses the most time-efficient path (calculated by A* with turn costs) to find the next free storage location.
        /// </summary>
        ShortestTime,
    }
    /// <summary>
    /// Implements a pod storage manager that aims to use the next free storage location.
    /// </summary>
    class GM_1PodStorageManager : PodStorageManager
    {
        /// <summary>
        /// Creates a new instance of the manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public GM_1PodStorageManager(Instance instance) : base(instance) { _config = instance.ControllerConfig.PodStorageConfig as GM_1PodStorageConfiguration; }

        /// <summary>
        /// The config for this manager.
        /// </summary>
        private GM_1PodStorageConfiguration _config;
        /// <summary>
        /// 用指派模型来求解货架重定位问题
        /// </summary>
        /// <param name="type"></param>
        /// <param name="pod"></param>
        /// <param name="station"></param>
        /// <returns></returns>
        public Waypoint SolveByMp(SolverType type, Pod pod, OutputStation station)
        {
            Dictionary<Pod, OutputStation> Task = new Dictionary<Pod, OutputStation>();
            foreach (var station1 in Instance.OutputStations.Where(v => Instance.ResourceManager._Ziops1[v].Select(s => s.Key).Where(u => Instance.ResourceManager.UnusedPods.Contains(u) && !Instance.ResourceManager.BottoPod.ContainsValue(u)).Count()>0))
            {
                foreach (var pod1 in Instance.ResourceManager._Ziops1[station1].Select(s => s.Key).Where(u => Instance.ResourceManager.UnusedPods.Contains(u) && !Instance.ResourceManager.BottoPod.ContainsValue(u)))
                {
                    if(!Task.ContainsKey(pod1))
                        Task.Add(pod1, station1);
                }
            }
            LinearModel wrapper = new LinearModel(type, (string s) => { Console.Write(s); });
            List<Symbol> deVarNamezij = new List<Symbol>();
            foreach (var storageLocation in Instance.ResourceManager.UnusedPodStorageLocations)
            {
                foreach (var task in Task)
                    deVarNamezij.Add(new Symbol { waypoint = storageLocation, pod = task.Key, outputstation = task.Value, name = "zij" + "_" + storageLocation.ID.ToString() + "_" + task.Key.ID.ToString() });
            }
            List<Symbol> deVarNameyj = new List<Symbol>();
            foreach (var task in Task)
                deVarNameyj.Add(new Symbol { pod = task.Key, outputstation = task.Value, name = "yj" + "_" + task.Key.ID.ToString() });
            List<Symbol> deVarNamexi = new List<Symbol>();
            foreach (var storageLocation in Instance.ResourceManager.UnusedPodStorageLocations)
                deVarNamexi.Add(new Symbol { waypoint = storageLocation, name = "xi" + "_" + storageLocation.ID.ToString()});
            VariableCollection<string> variablesBinary = new VariableCollection<string>(wrapper, VariableType.Binary, 0, 1, (string s) => { return s; });
            //目标函数为最小化bot到pod的距离之和
            if((Task.Count > 0))
            {
                wrapper.SetObjective(LinearExpression.Sum(deVarNamexi.Select(v => variablesBinary[v.name] * DistanceSetofstationtopod[station.Waypoint.ID][v.waypoint.ID])) + 
                    LinearExpression.Sum(deVarNamezij.Select(v => variablesBinary[v.name] * Distances.CalculateManhattan1(v.waypoint, v.pod))) +
                    LinearExpression.Sum(deVarNameyj.Select(v => variablesBinary[v.name] * DistanceSetofpodtostation[v.pod.ID][v.outputstation.ID])), OptimizationSense.Minimize);
            }
            else
            {
                wrapper.SetObjective(LinearExpression.Sum(deVarNamexi.Select(v => variablesBinary[v.name] * DistanceSetofstationtopod[station.Waypoint.ID][v.waypoint.ID])), OptimizationSense.Minimize);
            }
            wrapper.AddConstr(LinearExpression.Sum(deVarNamexi.Select(v => variablesBinary[v.name])) == 1, "shi1");
            if(Task.Count>0)
                wrapper.AddConstr(LinearExpression.Sum(deVarNameyj.Select(v => variablesBinary[v.name])) == 1, "shi2");
            //else
            //    wrapper.AddConstr(LinearExpression.Sum(deVarNameyj.Select(v => variablesBinary[v.name])) <= 1, "shi2");
            foreach (var storageLocation in Instance.ResourceManager.UnusedPodStorageLocations)
            {
                foreach (var task in Task)
                {
                    wrapper.AddConstr(variablesBinary["xi" + "_" + storageLocation.ID.ToString()] + variablesBinary["yj" + "_" + task.Key.ID.ToString()]
                                <= 1 + variablesBinary["zij" + "_" + storageLocation.ID.ToString() + "_" + task.Key.ID.ToString()], "shi3");
                    wrapper.AddConstr(variablesBinary["zij" + "_" + storageLocation.ID.ToString() + "_" + task.Key.ID.ToString()] <= variablesBinary["xi" + "_" + storageLocation.ID.ToString()], "shi4");
                    wrapper.AddConstr(variablesBinary["zij" + "_" + storageLocation.ID.ToString() + "_" + task.Key.ID.ToString()] <= variablesBinary["yj" + "_" + task.Key.ID.ToString()], "shi5");
                }                   
            }
            wrapper.Update();
            wrapper.Optimize();
            Waypoint waypoint = null;
            if (wrapper.HasSolution())
            {
                foreach (var itemName in deVarNamexi)
                {
                    if (Math.Round(variablesBinary[itemName.name].GetValue()) != 0)
                    {
                        waypoint= itemName.waypoint;
                        break;
                    }
                }
                foreach (var itemName in deVarNameyj)
                {
                    if (Math.Round(variablesBinary[itemName.name].GetValue()) != 0)
                    {
                        Instance.ResourceManager.BottoPod.Add(pod.Bot, itemName.pod);
                        //Instance.ResourceManager.ClaimPod(itemName.pod, itemName.robot, BotTaskType.Extract);
                        itemName.outputstation.RegisterInboundPod(itemName.pod);
                        break;
                    }
                }
            }
            return waypoint;
        }
        /// <summary>
        /// Returns a suitable storage location for the given pod
        /// </summary>
        /// <param name="pod"></param>
        /// <returns></returns>
        protected override Waypoint GetStorageLocationForPod1(Pod pod)
        {
            OutputStation station = null;
            foreach (var sta in Instance.OutputStations.Where(v => v.X == pod.X && v.Y == pod.Y))
                station = sta;
            Waypoint bestStorageLocation = null;
            Waypoint podLocation = // Get current waypoint of pod, if we want to estimate a path
                _config.PodDisposeRule == GM_1PodStorageLocationDisposeRule.ShortestPath || _config.PodDisposeRule == GM_1PodStorageLocationDisposeRule.ShortestTime ?
                    Instance.WaypointGraph.GetClosestWaypoint(pod.Tier, pod.X, pod.Y) :
                    null;
            bestStorageLocation =SolveByMp(SolverType.Gurobi, pod, station);
            // Check success
            if (bestStorageLocation == null)
                throw new InvalidOperationException("There was no suitable storage location for the pod: " + pod.ToString());
            // Return it
            return bestStorageLocation;
        }
        /// <summary>
        /// Returns a suitable storage location for the given pod.
        /// </summary>
        /// <param name="pod">The pod to fetch a storage location for.</param>
        /// <returns>The storage location to use.</returns>
        protected override Waypoint GetStorageLocationForPod(Pod pod)
        {
            double minDistance = double.PositiveInfinity; Waypoint bestStorageLocation = null;
            Waypoint podLocation = // Get current waypoint of pod, if we want to estimate a path
                _config.PodDisposeRule == GM_1PodStorageLocationDisposeRule.ShortestPath || _config.PodDisposeRule == GM_1PodStorageLocationDisposeRule.ShortestTime ?
                    Instance.WaypointGraph.GetClosestWaypoint(pod.Tier, pod.X, pod.Y) :
                    null;
            foreach (var storageLocation in Instance.ResourceManager.UnusedPodStorageLocations)
            {
                // Calculate the distance
                double distance;
                switch (_config.PodDisposeRule)
                {
                    case GM_1PodStorageLocationDisposeRule.Euclid: distance = Distances.CalculateEuclid(pod, storageLocation, Instance.WrongTierPenaltyDistance); break;
                    case GM_1PodStorageLocationDisposeRule.Manhattan: distance = Distances.CalculateManhattan(pod, storageLocation, Instance.WrongTierPenaltyDistance); break;
                    case GM_1PodStorageLocationDisposeRule.ShortestPath: distance = Distances.CalculateShortestPathPodSafe(podLocation, storageLocation, Instance); break;
                    case GM_1PodStorageLocationDisposeRule.ShortestTime: distance = Distances.CalculateShortestTimePathPodSafe(podLocation, storageLocation, Instance); break;
                    default: throw new ArgumentException("Unknown pod dispose rule: " + _config.PodDisposeRule);
                }
                // Update minimum
                if (distance < minDistance)
                {
                    minDistance = distance;
                    bestStorageLocation = storageLocation;
                }
            }
            // Check success
            if (bestStorageLocation == null)
                throw new InvalidOperationException("There was no suitable storage location for the pod: " + pod.ToString());
            // Return it
            return bestStorageLocation;
        }

        #region IOptimize Members

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public override void SignalCurrentTime(double currentTime) { /* Ignore since this simple manager is always ready. */ }

        #endregion
    }
}
