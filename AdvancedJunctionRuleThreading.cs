using AdvancedJunctionRule.CustomAI;
using AdvancedJunctionRule.Util;
using ColossalFramework;
using ColossalFramework.Globalization;
using ColossalFramework.UI;
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
    public class AdvancedJunctionRuleThreading : ThreadingExtensionBase
    {
        public static bool isFirstTime = true;
        public static bool isDetoured = false;

        public override void OnBeforeSimulationFrame()
        {
            base.OnBeforeSimulationFrame();

            if (AdvancedJunctionRule.IsEnabled)
            {
                CheckDetour();
                int num26 = (int)(Singleton<SimulationManager>.instance.m_currentFrameIndex & 0xFF);
                int num27 = num26 * 144;
                int num28 = (num26 + 1) * 144 - 1;

                if (num26 == 255)
                {
                    RoadUI.refeshOnce = true;
                }
                //Show road name for UI
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

        public void DetourAfterLoad()
        {
            //This is for Detour Other Mod method
            DebugLog.LogToFileOnly("Init DetourAfterLoad");
            bool detourFailed = false;

            if (Loader.is1637663252)
            {
                DebugLog.LogToFileOnly("Detour 1637663252.VehicleBehaviorManager::MayChangeSegment calls");
                try
                {
                    Loader.Detours.Add(new Loader.Detour(typeof(VehicleBehaviorManager).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
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
                typeof(float).MakeByRefType()}, null),
            typeof(NewCarAI).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
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
                typeof(float).MakeByRefType() }, null)));
                }
                catch (Exception)
                {
                    DebugLog.LogToFileOnly("Could not detour 1637663252.VehicleBehaviorManager::MayChangeSegment");
                    detourFailed = true;
                }
            }
            else
            {
                DebugLog.LogToFileOnly("Detour old.VehicleBehaviorManager::MayChangeSegment calls");
                try
                {
                    Loader.Detours.Add(new Loader.Detour(typeof(VehicleBehaviorManager).GetMethod("MayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
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
                typeof(float).MakeByRefType()}, null),
            typeof(NewCarAI).GetMethod("OldTMPEMayChangeSegment", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] {                 typeof(ushort),
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
                typeof(float).MakeByRefType() }, null)));
                }
                catch (Exception)
                {
                    DebugLog.LogToFileOnly("Could not detour old.VehicleBehaviorManager::MayChangeSegment");
                    detourFailed = true;
                }
            }

            DebugLog.LogToFileOnly("Detour CustomRoadAI::GetTrafficLightState calls");
            try
            {
                Loader.Detours.Add(new Loader.Detour(typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(bool).MakeByRefType(), typeof(bool).MakeByRefType() }, null),
                                       typeof(AdvancedJunctionRuleRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(bool).MakeByRefType(), typeof(bool).MakeByRefType() }, null)));
            }
            catch (Exception)
            {
                DebugLog.LogToFileOnly("Could not detour CustomRoadAI::GetTrafficLightState");
                detourFailed = true;
            }

            DebugLog.LogToFileOnly("Detour LaneConnectorTool::CheckSegmentsTurningAngle calls");
            try
            {
                Loader.Detours.Add(new Loader.Detour(typeof(LaneConnectorTool).GetMethod("CheckSegmentsTurningAngle", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] { typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool) }, null),
                                       typeof(NewLaneConnectorTool).GetMethod("CheckSegmentsTurningAngle", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] { typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool) }, null)));
            }
            catch (Exception)
            {
                DebugLog.LogToFileOnly("Could not detour LaneConnectorTool::CheckSegmentsTurningAngle");
                detourFailed = true;
            }

            DebugLog.LogToFileOnly("Detour CustomRoadAI::GetTrafficLightState calls");
            try
            {
                Loader.Detours.Add(new Loader.Detour(typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType() }, null),
                                       typeof(AdvancedJunctionRuleRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType() }, null)));
            }
            catch (Exception)
            {
                DebugLog.LogToFileOnly("Could not detour CustomRoadAI::GetTrafficLightState");
                detourFailed = true;
            }

            if (detourFailed)
            {
                DebugLog.LogToFileOnly("DetourAfterLoad failed");
            }
            else
            {
                DebugLog.LogToFileOnly("DetourAfterLoad successful");
            }
        }

        public void CheckDetour()
        {
            if (isFirstTime && Loader.DetourInited)
            {
                isFirstTime = false;
                DetourAfterLoad();
                if (Loader.DetourInited)
                {
                    DebugLog.LogToFileOnly("ThreadingExtension.OnBeforeSimulationFrame: First frame detected. Checking detours.");
                    List<string> list = new List<string>();
                    foreach (Loader.Detour current in Loader.Detours)
                    {
                        if (!RedirectionHelper.IsRedirected(current.OriginalMethod, current.CustomMethod))
                        {
                            list.Add(string.Format("{0}.{1} with {2} parameters ({3})", new object[]
                            {
                    current.OriginalMethod.DeclaringType.Name,
                    current.OriginalMethod.Name,
                    current.OriginalMethod.GetParameters().Length,
                    current.OriginalMethod.DeclaringType.AssemblyQualifiedName
                            }));
                        }
                    }
                    DebugLog.LogToFileOnly(string.Format("ThreadingExtension.OnBeforeSimulationFrame: First frame detected. Detours checked. Result: {0} missing detours", list.Count));
                    if (list.Count > 0)
                    {
                        string error = "AdvancedJunctionRuleThreading detected an incompatibility with another mod! You can continue playing but it's NOT recommended. AdvancedJunctionRuleThreading will not work as expected. Send AdvancedJunctionRuleThreading.txt to Author.";
                        DebugLog.LogToFileOnly(error);
                        string text = "The following methods were overriden by another mod:";
                        foreach (string current2 in list)
                        {
                            text += string.Format("\n\t{0}", current2);
                        }
                        DebugLog.LogToFileOnly(text);
                        UIView.library.ShowModal<ExceptionPanel>("ExceptionPanel").SetMessage("Incompatibility Issue", text, true);
                    }
                }
            }
        }
    }
}
