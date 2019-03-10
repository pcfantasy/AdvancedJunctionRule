using ColossalFramework;
using ColossalFramework.Globalization;
using ICities;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using TrafficManager.Custom.AI;
using TrafficManager.Custom.Data;
using TrafficManager.Manager.Impl;
using TrafficManager.State;
using TrafficManager.State.ConfigData;
using TrafficManager.Traffic.Data;
using TrafficManager.UI.SubTools;
using UnityEngine;

namespace AdvancedJunctionRule
{
    public class Threading : ThreadingExtensionBase
    {
        public static bool isFirstTiming = true;
        public static bool isDetoured = false;
        //private LeftTurnWaiting mod;
        public static RedirectCallsState state1;
        public static RedirectCallsState state2;
        public static RedirectCallsState state3;
        public static RedirectCallsState state4;
        public static RedirectCallsState state5;

        public override void OnBeforeSimulationFrame()
        {
            //DebugLog.LogToFileOnly(GlobalConfig.Instance.PathFinding.UturnLaneDistance.ToString());

            if (isFirstTiming && AdvancedJunctionRule.IsEnabled && Loader.isLoaded)
            {
                //protected bool MayChangeSegment(ushort frontVehicleId, ref VehicleState vehicleState, ref Vehicle vehicleData, float sqrVelocity, ref PathUnit.Position prevPos, ref NetSegment prevSegment, ushort prevTargetNodeId, uint prevLaneID, ref PathUnit.Position position, ushort targetNodeId, ref NetNode targetNode, uint laneID, ref PathUnit.Position nextPosition, ushort nextTargetNodeId, out float maxSpeed)
                if (Loader.is1637663252)
                {
                    var srcMethod1 = typeof(VehicleBehaviorManager).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
                typeof(VehicleState).MakeByRefType(),
                typeof(Vehicle).MakeByRefType(),
                typeof(float),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(NetSegment).MakeByRefType(),
                typeof(ushort),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(NetNode).MakeByRefType(),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(float).MakeByRefType()}, null);
                    var destMethod1 = typeof(NewCarAI).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
                typeof(VehicleState).MakeByRefType(),
                typeof(Vehicle).MakeByRefType(),
                typeof(float),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(NetSegment).MakeByRefType(),
                typeof(ushort),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(NetNode).MakeByRefType(),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(float).MakeByRefType() }, null);
                    state1 = RedirectionHelper.RedirectCalls(srcMethod1, destMethod1);
                } else
                {
                    var srcMethod1 = typeof(VehicleBehaviorManager).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
                typeof(VehicleState).MakeByRefType(),
                typeof(Vehicle).MakeByRefType(),
                typeof(float),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(NetSegment).MakeByRefType(),
                typeof(ushort),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(NetNode).MakeByRefType(),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(float).MakeByRefType()}, null);
                    var destMethod1 = typeof(NewCarAI).GetMethod("OldTMPEMayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
                typeof(VehicleState).MakeByRefType(),
                typeof(Vehicle).MakeByRefType(),
                typeof(float),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(NetSegment).MakeByRefType(),
                typeof(ushort),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(NetNode).MakeByRefType(),
                typeof(uint),
                typeof(PathUnit.Position).MakeByRefType(),
                typeof(ushort),
                typeof(float).MakeByRefType() }, null);
                    state1 = RedirectionHelper.RedirectCalls(srcMethod1, destMethod1);
                }


                //public void CustomSimulationStep(ushort vehicleID, ref Vehicle vehicleData, ref Vehicle.Frame frameData, ushort leaderID, ref Vehicle leaderData, int lodPhysics)
                var srcMethod2 = typeof(CarAI).GetMethod("SimulationStep", BindingFlags.Instance | BindingFlags.Public, null, new Type[] {
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(Vehicle.Frame).MakeByRefType(),
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(int)}, null);

                var destMethod2 = typeof(NewCarAI).GetMethod("CustomSimulationStep", BindingFlags.Instance | BindingFlags.Public, null, new Type[]{
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(Vehicle.Frame).MakeByRefType(),
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(int)}, null);

                state2 = RedirectionHelper.RedirectCalls(srcMethod2, destMethod2);

                var srcMethod3 = typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(bool).MakeByRefType(), typeof(bool).MakeByRefType() }, null);
                var destMethod3 = typeof(AdvancedJunctionRuleRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(bool).MakeByRefType(), typeof(bool).MakeByRefType() }, null);
                state3 = RedirectionHelper.RedirectCalls(srcMethod3, destMethod3);
                var srcMethod4 = typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType() }, null);
                var destMethod4 = typeof(AdvancedJunctionRuleRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType() }, null);
                state4 = RedirectionHelper.RedirectCalls(srcMethod4, destMethod4);


                //private bool CheckSegmentsTurningAngle(ushort sourceSegmentId, ref NetSegment sourceSegment, bool sourceStartNode, ushort targetSegmentId, ref NetSegment targetSegment, bool targetStartNode)
                var srcMethod5 = typeof(LaneConnectorTool).GetMethod("CheckSegmentsTurningAngle", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] { typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool) }, null);
                var destMethod5 = typeof(NewLaneConnectorTool).GetMethod("CheckSegmentsTurningAngle", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] { typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool) }, null);
                state5 = RedirectionHelper.RedirectCalls(srcMethod5, destMethod5);
                isDetoured = true;
                isFirstTiming = false;
                //MainDataStore.DataInit();
            }
            base.OnBeforeSimulationFrame();


            if (AdvancedJunctionRule.IsEnabled)
            {
                int num26 = (int)(Singleton<SimulationManager>.instance.m_currentFrameIndex & 0xFF);
                int num27 = num26 * 144;
                int num28 = (num26 + 1) * 144 - 1;

                if (num26 == 255)
                {
                    RoadUI.refeshOnce = true;
                }

                if (SingletonLite<LocaleManager>.instance.language.Contains("zh") && (MainDataStore.lastLanguage == 1))
                {
                }
                else if (!SingletonLite<LocaleManager>.instance.language.Contains("zh") && (MainDataStore.lastLanguage != 1))
                {
                }
                else
                {
                    MainDataStore.lastLanguage = (byte)(SingletonLite<LocaleManager>.instance.language.Contains("zh") ? 1 : 0);
                    Language.LanguageSwitch(MainDataStore.lastLanguage);
                    RoadUI.refeshOnce = true;
                }

                for (int num30 = num27; num30 <= num28; num30++)
                {
                    if (Singleton<NetManager>.instance.m_segments.m_buffer[num30].m_flags.IsFlagSet(NetSegment.Flags.Created) && Singleton<NetManager>.instance.m_segments.m_buffer[num30].Info.m_vehicleTypes.IsFlagSet(VehicleInfo.VehicleType.Car))
                    {
                        var instance = Singleton<NetManager>.instance;
                        var startNode = instance.m_segments.m_buffer[num30].m_startNode;
                        var endNode = instance.m_segments.m_buffer[num30].m_endNode;
                        if (instance.m_nodes.m_buffer[startNode].m_flags.IsFlagSet(NetNode.Flags.Junction) || instance.m_nodes.m_buffer[endNode].m_flags.IsFlagSet(NetNode.Flags.Junction) || (Math.Abs(Singleton<NetManager>.instance.m_segments.m_buffer[num30].m_middlePosition.x) > 8000 || Math.Abs(Singleton<NetManager>.instance.m_segments.m_buffer[num30].m_middlePosition.z) > 8000))
                        {
                            if (!Singleton<NetManager>.instance.m_segments.m_buffer[num30].m_flags.IsFlagSet(NetSegment.Flags.NameVisible2))
                            {
                                Singleton<NetManager>.instance.m_segments.m_buffer[num30].m_flags |= NetSegment.Flags.NameVisible2;
                                Singleton<NetManager>.instance.UpdateSegmentRenderer((ushort)num30, false);
                            }
                        }
                    }

                }
            }
        }



    }
}
