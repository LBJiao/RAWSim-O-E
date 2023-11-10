using RAWSimO.Core.Elements;
using RAWSimO.Core.Interfaces;
using RAWSimO.Core.Waypoints;
using System;
using System.Collections.Generic;
using RAWSimO.Core.Metrics;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace RAWSimO.Core.Control
{
    /// <summary>
    /// The base class of all mechanisms deciding the position when storing a pod.
    /// </summary>
    public abstract class PodStorageManager : IUpdateable, IOptimize, IStatTracker
    {
        /// <summary>
        /// Creates a new instance of this manager.
        /// </summary>
        /// <param name="instance">The instance this manager belongs to.</param>
        public PodStorageManager(Instance instance) { Instance = instance; }

        /// <summary>
        /// The instance this manager is assigned to.
        /// </summary>
        protected Instance Instance { get; set; }

        /// <summary>
        /// Decides the waypoint to use when storing a pod. This call is measured by the timing done.
        /// </summary>
        /// <param name="pod">The pod to store.</param>
        /// <returns>The waypoint to use.</returns>
        protected abstract Waypoint GetStorageLocationForPod(Pod pod);
        /// <summary>
        /// Decides the waypoint to use when storing a pod. This call is measured by the timing done.
        /// </summary>
        /// <param name="pod"></param>
        /// <returns></returns>
        protected abstract Waypoint GetStorageLocationForPod1(Pod pod);
        /// <summary>
        /// 存储pod到station的距离
        /// </summary>
        public static Dictionary<int, Dictionary<int, double>> DistanceSetofpodtostation = new Dictionary<int, Dictionary<int, double>>();
        /// <summary>
        /// 存储station到pod的距离
        /// </summary>
        public static Dictionary<int, Dictionary<int, double>> DistanceSetofstationtopod = new Dictionary<int, Dictionary<int, double>>();
        /// <summary>
        /// 保存distance
        /// </summary>
        /// <returns></returns>
        public void SaveDistance()
        {
            foreach (var station in Instance.OutputStations)
            {
                Dictionary<int, double> distancelist = new Dictionary<int, double>();
                foreach (var point in Instance.Waypoints)
                {
                    distancelist.Add(point.ID, Distances.CalculateShortestPathPodSafe1(station.Waypoint, point, Instance));
                }
                DistanceSetofstationtopod.Add(station.Waypoint.ID, distancelist);
            }
            foreach (var point in Instance.Waypoints)
            {
                Dictionary<int, double> distancelist = new Dictionary<int, double>();
                foreach (var station in Instance.OutputStations)
                {
                    distancelist.Add(station.ID, Distances.CalculateShortestPathPodSafe1(point, station.Waypoint, Instance));
                }
                DistanceSetofpodtostation.Add(point.ID, distancelist);
            }
            FileStream fs1 = new FileStream("distance1.txt", FileMode.Create, FileAccess.Write);
            StreamWriter sw1 = new StreamWriter(fs1); // 创建写bai入流du
            foreach (var stationpoint in DistanceSetofstationtopod)
            {
                foreach (var distance in stationpoint.Value)
                    sw1.WriteLine(stationpoint.Key.ToString() + "\t" + distance.Key.ToString() + "\t" + distance.Value.ToString() + "\t");
            }
            sw1.Flush();
            sw1.Close(); //关闭文件
            FileStream fs2 = new FileStream("distance2.txt", FileMode.Create, FileAccess.Write);
            StreamWriter sw2 = new StreamWriter(fs2); // 创建写bai入流du
            foreach (var podtostation in DistanceSetofpodtostation)
            {
                foreach (var distance in podtostation.Value)
                    sw2.WriteLine(podtostation.Key.ToString() + "\t" + distance.Key.ToString() + "\t" + distance.Value.ToString() + "\t");
            }
            sw2.Flush();
            sw2.Close(); //关闭文件
        }
        /// <summary>
        /// 读取distance
        /// </summary>
        /// <returns></returns>
        public void ReadDistance()
        {
            string[] lineValues;
            string actLine;
            char[] separator = new char[] { '\t' };
            string inputPath = @"C:\EE1\RAWSim-O-E\RAWSimO.Visualization\bin\x64\Debug\distance1.txt";
            StreamReader sr = new StreamReader(inputPath);
            int sum = Instance.OutputStations.Count * Instance.Waypoints.Count;
            for (int i = 0; i < sum; i++)
            {
                actLine = sr.ReadLine();
                lineValues = actLine.Split(separator);
                int i1 = Int32.Parse(lineValues[0]);//卸货点序号
                if (DistanceSetofstationtopod.ContainsKey(i1))
                    DistanceSetofstationtopod[i1].Add(Int32.Parse(lineValues[1]), Int32.Parse(lineValues[2]));
                else
                {
                    Dictionary<int, double> distancelist = new Dictionary<int, double>();
                    distancelist.Add(Int32.Parse(lineValues[1]), Int32.Parse(lineValues[2]));
                    DistanceSetofstationtopod.Add(i1, distancelist);
                }
            }
            string inputPath1 = @"C:\EE1\RAWSim-O-E\RAWSimO.Visualization\bin\x64\Debug\distance2.txt";
            StreamReader sr1 = new StreamReader(inputPath1);
            int sum1 = Instance.OutputStations.Count * Instance.Waypoints.Count;
            for (int i = 0; i < sum1; i++)
            {
                actLine = sr1.ReadLine();
                lineValues = actLine.Split(separator);
                int i1 = Int32.Parse(lineValues[0]);//卸货点序号
                if (DistanceSetofpodtostation.ContainsKey(i1))
                    DistanceSetofpodtostation[i1].Add(Int32.Parse(lineValues[1]), Int32.Parse(lineValues[2]));
                else
                {
                    Dictionary<int, double> distancelist = new Dictionary<int, double>();
                    distancelist.Add(Int32.Parse(lineValues[1]), Int32.Parse(lineValues[2]));
                    DistanceSetofpodtostation.Add(i1, distancelist);
                }
            }
        }
        #region IPodStorageManager Members

        /// <summary>
        /// Determines the storage location for the given pod.
        /// </summary>
        /// <param name="pod"></param>
        /// <returns></returns>
        public Waypoint GetStorageLocation1(Pod pod)
        {
            // Measure time for decision
            DateTime before = DateTime.Now;
            //SaveDistance();//提前计算distance
            if (Instance.ifreaddistance)
            {
                ReadDistance();
                Instance.ifreaddistance = false;
            }
            // Fetch storage location
            Waypoint wp = GetStorageLocationForPod1(pod);
            // Calculate decision time
            Instance.Observer.TimePodStorage((DateTime.Now - before).TotalSeconds);
            // Return it
            return wp;
        }
        /// <summary>
        /// Determines the storage location for the given pod.
        /// </summary>
        /// <param name="pod">The pod.</param>
        /// <returns>The storage location to use.</returns>
        public Waypoint GetStorageLocation(Pod pod)
        {
            // Measure time for decision
            DateTime before = DateTime.Now;
            // Fetch storage location
            Waypoint wp = GetStorageLocationForPod(pod);
            // Calculate decision time
            Instance.Observer.TimePodStorage((DateTime.Now - before).TotalSeconds);
            // Return it
            return wp;
        }

        #endregion

        #region IUpdateable Members

        /// <summary>
        /// The next event when this element has to be updated.
        /// </summary>
        /// <param name="currentTime">The current time of the simulation.</param>
        /// <returns>The next time this element has to be updated.</returns>
        public virtual double GetNextEventTime(double currentTime) { return double.PositiveInfinity; }
        /// <summary>
        /// Updates the element to the specified time.
        /// </summary>
        /// <param name="lastTime">The time before the update.</param>
        /// <param name="currentTime">The time to update to.</param>
        public virtual void Update(double lastTime, double currentTime) { /* Nothing to do here. */ }

        #endregion

        #region IOptimize Members

        /// <summary>
        /// Signals the current time to the mechanism. The mechanism can decide to block the simulation thread in order consume remaining real-time.
        /// </summary>
        /// <param name="currentTime">The current simulation time.</param>
        public abstract void SignalCurrentTime(double currentTime);

        #endregion

        #region IStatTracker Members

        /// <summary>
        /// The callback that indicates that the simulation is finished and statistics have to submitted to the instance.
        /// </summary>
        public virtual void StatFinish() { /* Default case: do not flush any statistics */ }

        /// <summary>
        /// The callback indicates a reset of the statistics.
        /// </summary>
        public virtual void StatReset() { /* Default case: nothing to reset */ }

        #endregion
    }
}
