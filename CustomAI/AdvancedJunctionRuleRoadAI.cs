using AdvancedJunctionRule.Util;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TrafficManager;
using TrafficManager.Geometry.Impl;
using TrafficManager.Manager.Impl;
using TrafficManager.State;
using TrafficManager.TrafficLight;
using TrafficManager.TrafficLight.Data;

namespace AdvancedJunctionRule.CustomAI
{
    public class AdvancedJunctionRuleRoadAI : RoadBaseAI
    {
        public static void GetTrafficLightState(ushort nodeId, ushort fromSegmentId, byte fromLaneIndex, ushort toSegmentId, ref NetSegment segmentData, uint frame, out RoadBaseAI.TrafficLightState vehicleLightState, out RoadBaseAI.TrafficLightState pedestrianLightState)
        {
            if (!Options.timedLightsEnabled || !TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)nodeId].IsSimulationRunning())
            {
                RoadBaseAI.GetTrafficLightState(nodeId, ref segmentData, frame, out vehicleLightState, out pedestrianLightState);
                SegmentGeometry segmentGeometry = SegmentGeometry.Get(fromSegmentId, false);
                bool startNode = segmentGeometry.StartNodeId() == nodeId;
                if (MainDataStore.canUTurn[fromSegmentId])
                {
                    if (fromSegmentId == toSegmentId)
                    {
                        if (vehicleLightState != RoadBaseAI.TrafficLightState.Green)
                        {
                            Random rand = new Random();
                            //Add this to let U turn car low proiority
                            if (rand.Next(2) == 0)
                            {
                                vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                            }
                        }
                    }
                }
                return;
            }



            GetCustomTrafficLightState(nodeId, fromSegmentId, fromLaneIndex, toSegmentId, out vehicleLightState, out pedestrianLightState, ref TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)nodeId]);
            if (MainDataStore.canUTurn[fromSegmentId])
            {
                if (fromSegmentId == toSegmentId)
                {
                    if (vehicleLightState != RoadBaseAI.TrafficLightState.Green)
                    {
                        Random rand = new Random();
                        //Add this to let U turn car low proiority
                        if (rand.Next(2) == 0)
                        {
                            vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                        }
                    }
                }
            }
        }


        public static void GetTrafficLightState(ushort nodeId, ushort fromSegmentId, byte fromLaneIndex, ushort toSegmentId, ref NetSegment segmentData, uint frame, out RoadBaseAI.TrafficLightState vehicleLightState, out RoadBaseAI.TrafficLightState pedestrianLightState, out bool vehicles, out bool pedestrians)
        {
            if (!Options.timedLightsEnabled || !TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)nodeId].IsSimulationRunning())
            {
                RoadBaseAI.GetTrafficLightState(nodeId, ref segmentData, frame, out vehicleLightState, out pedestrianLightState, out vehicles, out pedestrians);
                SegmentGeometry segmentGeometry = SegmentGeometry.Get(fromSegmentId, false);
                bool startNode = segmentGeometry.StartNodeId() == nodeId;

                if (MainDataStore.canUTurn[fromSegmentId])
                {
                    if (fromSegmentId == toSegmentId)
                    {
                        if (vehicleLightState != RoadBaseAI.TrafficLightState.Green)
                        {
                            Random rand = new Random();
                            //Add this to let U turn car low proiority
                            if (rand.Next(2) == 0)
                            {
                                vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                            }
                        }
                    }
                }
                return;
            }



            GetCustomTrafficLightState(nodeId, fromSegmentId, fromLaneIndex, toSegmentId, out vehicleLightState, out pedestrianLightState, ref TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)nodeId]);
            vehicles = false;
            pedestrians = false;
            if (MainDataStore.canUTurn[fromSegmentId])
            {
                if (fromSegmentId == toSegmentId)
                {
                    if (vehicleLightState != RoadBaseAI.TrafficLightState.Green)
                    {
                        Random rand = new Random();
                        //Add this to let U turn car low proiority
                        if (rand.Next(2) == 0)
                        {
                            vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                        }
                    }
                }
            }
        }

        private static void GetCustomTrafficLightState(ushort nodeId, ushort fromSegmentId, byte fromLaneIndex, ushort toSegmentId, out RoadBaseAI.TrafficLightState vehicleLightState, out RoadBaseAI.TrafficLightState pedestrianLightState, ref TrafficLightSimulation nodeSim)
        {
            SegmentGeometry segmentGeometry = SegmentGeometry.Get(fromSegmentId, false);
            if (segmentGeometry == null)
            {
                //Log.Error(string.Format("GetTrafficLightState: No geometry information @ node {0}, segment {1}.", nodeId, fromSegmentId));
                vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                pedestrianLightState = RoadBaseAI.TrafficLightState.Green;
                return;
            }
            bool startNode = segmentGeometry.StartNodeId() == nodeId;
            ICustomSegmentLights segmentLights = CustomSegmentLightsManager.Instance.GetSegmentLights(fromSegmentId, startNode, false, RoadBaseAI.TrafficLightState.Red);
            if (segmentLights != null)
            {
                pedestrianLightState = (segmentLights.PedestrianLightState.HasValue ? segmentLights.PedestrianLightState.Value : RoadBaseAI.TrafficLightState.Green);
            }
            else
            {
                pedestrianLightState = RoadBaseAI.TrafficLightState.Green;
            }
            ICustomSegmentLight customSegmentLight = (segmentLights == null) ? null : segmentLights.GetCustomLight(fromLaneIndex);
            if (segmentLights == null || customSegmentLight == null)
            {
                vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                return;
            }
            if (toSegmentId == fromSegmentId)
            {
                vehicleLightState = (Constants.ServiceFactory.SimulationService.LeftHandDrive ? customSegmentLight.LightRight : customSegmentLight.LightLeft);
                return;
            }
            if (segmentGeometry.IsLeftSegment(toSegmentId, startNode))
            {
                vehicleLightState = customSegmentLight.LightLeft;
                return;
            }
            if (segmentGeometry.IsRightSegment(toSegmentId, startNode))
            {
                vehicleLightState = customSegmentLight.LightRight;
                return;
            }
            vehicleLightState = customSegmentLight.LightMain;
        }
    }
}
