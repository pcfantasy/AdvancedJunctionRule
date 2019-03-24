using ColossalFramework.UI;
using ICities;
using UnityEngine;
using System.IO;
using ColossalFramework;
using System.Reflection;
using System;
using System.Linq;
using ColossalFramework.Math;
using TrafficManager.Custom.AI;
using TrafficManager.Manager.Impl;
using TrafficManager.Traffic.Data;
using TrafficManager.UI.SubTools;
using ColossalFramework.PlatformServices;
using System.Collections.Generic;
using AdvancedJunctionRule.Util;

namespace AdvancedJunctionRule
{
    public class Loader : LoadingExtensionBase
    {
        public static LoadMode CurrentLoadMode;

        public static UIPanel roadInfo;

        public static GameObject roadWindowGameObject;

        public static RoadUI guiPanel;

        public static bool isGuiRunning = false;
        public static bool isLoaded = false;
        public static bool is583429740 = false;
        public static bool is1637663252 = false;
        public static bool isRealCityRunning = false;
        public static bool isRealGasStationRunning = false;
        public static bool isTrafficCongestionReportRunning = false;
        public static bool HarmonyDetourInited = false;

        public static RedirectCallsState state1;
        public static RedirectCallsState state2;
        public static RedirectCallsState state3;
        public static RedirectCallsState state4;

        public override void OnCreated(ILoading loading)
        {
            base.OnCreated(loading);

        }

        public override void OnLevelLoaded(LoadMode mode)
        {
            base.OnLevelLoaded(mode);
            Loader.CurrentLoadMode = mode;
            if (AdvancedJunctionRule.IsEnabled)
            {
                if (mode == LoadMode.LoadGame || mode == LoadMode.NewGame)
                {
                    DebugLog.LogToFileOnly("OnLevelLoaded");
                    SetupRoadGui();
                    Detour();
                    CheckTMPE();
                    if (mode == LoadMode.NewGame)
                    {
                        DebugLog.LogToFileOnly("New Game");
                    }
                    isLoaded = true;
                }
            }
        }

        public void CheckTMPE()
        {
            if (IsSteamWorkshopItemSubscribed(583429740) && IsSteamWorkshopItemSubscribed(1637663252))
            {
                UIView.library.ShowModal<ExceptionPanel>("ExceptionPanel").SetMessage("Incompatibility Issue", "Can not sub two TM:PE, steamID:583429740 and 1637663252", true);
            } else if (IsSteamWorkshopItemSubscribed(583429740))
            {
                is583429740 = true;
            } else if (IsSteamWorkshopItemSubscribed(1637663252))
            {
                is1637663252 = true;
            }

            if (!this.Check3rdPartyModLoaded("TrafficManager", true))
            {
                UIView.library.ShowModal<ExceptionPanel>("ExceptionPanel").SetMessage("Incompatibility Issue", "Require TM:PE steamID:583429740 or 1637663252", true);
            }
        }

        public override void OnLevelUnloading()
        {
            base.OnLevelUnloading();
            is583429740 = false;
            is1637663252 = false;
            if (AdvancedJunctionRule.IsEnabled & isGuiRunning)
            {
                //remove RoadUI
                RemoveGui();
            }

            if (AdvancedJunctionRuleThreading.isDetoured)
            {
                RevertDetour();
            }
        }

        public override void OnReleased()
        {
            base.OnReleased();
        }

        public void Detour()
        {
            isRealCityRunning = Check3rdPartyModLoaded("RealCity", true);
            isRealGasStationRunning = Check3rdPartyModLoaded("RealGasStation", true);
            isTrafficCongestionReportRunning = Check3rdPartyModLoaded("TrafficCongestionReport", true);
        }

        public void RevertDetour()
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
            RedirectionHelper.RevertRedirect(srcMethod1, AdvancedJunctionRuleThreading.state1);
            var srcMethod2 = typeof(CarAI).GetMethod("SimulationStep", BindingFlags.Instance | BindingFlags.Public, null, new Type[] {
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(Vehicle.Frame).MakeByRefType(),
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(int)}, null);
            RedirectionHelper.RevertRedirect(srcMethod2, AdvancedJunctionRuleThreading.state2);
            var srcMethod3 = typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(bool).MakeByRefType(), typeof(bool).MakeByRefType() }, null);
            RedirectionHelper.RevertRedirect(srcMethod3, AdvancedJunctionRuleThreading.state3);
            var srcMethod4 = typeof(CustomRoadAI).GetMethod("GetTrafficLightState", BindingFlags.Public | BindingFlags.Instance | BindingFlags.Static, null, new Type[] { typeof(ushort), typeof(ushort), typeof(byte), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(uint), typeof(RoadBaseAI.TrafficLightState).MakeByRefType(), typeof(RoadBaseAI.TrafficLightState).MakeByRefType() }, null);
            RedirectionHelper.RevertRedirect(srcMethod4, AdvancedJunctionRuleThreading.state4);
            var srcMethod5 = typeof(LaneConnectorTool).GetMethod("CheckSegmentsTurningAngle", BindingFlags.NonPublic | BindingFlags.Instance, null, new Type[] { typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool), typeof(ushort), typeof(NetSegment).MakeByRefType(), typeof(bool) }, null);
            RedirectionHelper.RevertRedirect(srcMethod5, AdvancedJunctionRuleThreading.state5);

            AdvancedJunctionRuleThreading.isDetoured = false;
            AdvancedJunctionRuleThreading.isFirstTiming = true;
            isLoaded = false;
        }

        public static void SetupRoadGui()
        {
            roadWindowGameObject = new GameObject("roadWindowObject");
            guiPanel = (RoadUI)roadWindowGameObject.AddComponent(typeof(RoadUI));
            roadInfo = UIView.Find<UIPanel>("(Library) RoadWorldInfoPanel");
            if (roadInfo == null)
            {
                DebugLog.LogToFileOnly("UIPanel not found (update broke the mod!): (Library) RoadWorldInfoPanel\nAvailable panels are:\n");
            }
            guiPanel.transform.parent = roadInfo.transform;
            guiPanel.size = new Vector3(roadInfo.size.x, roadInfo.size.y);
            guiPanel.baseBuildingWindow = roadInfo.gameObject.transform.GetComponentInChildren<RoadWorldInfoPanel>();
            guiPanel.position = new Vector3(roadInfo.size.x, roadInfo.size.y);
            roadInfo.eventVisibilityChanged += roadInfo_eventVisibilityChanged;
            Loader.isGuiRunning = true;
        }



        public static void RemoveGui()
        {
            if (guiPanel != null)
            {
                if (guiPanel.parent != null)
                {
                    guiPanel.parent.eventVisibilityChanged -= roadInfo_eventVisibilityChanged;
                }
            }
            if (roadWindowGameObject != null)
            {
                UnityEngine.Object.Destroy(roadWindowGameObject);
            }
        }

        public static void roadInfo_eventVisibilityChanged(UIComponent component, bool value)
        {
            guiPanel.isEnabled = value;
            if (value)
            {
                Loader.guiPanel.transform.parent = Loader.roadInfo.transform;
                Loader.guiPanel.size = new Vector3(Loader.roadInfo.size.x, Loader.roadInfo.size.y);
                Loader.guiPanel.baseBuildingWindow = Loader.roadInfo.gameObject.transform.GetComponentInChildren<RoadWorldInfoPanel>();
                Loader.guiPanel.position = new Vector3(Loader.roadInfo.size.x, Loader.roadInfo.size.y);
                guiPanel.Show();
            }
            else
            {
                guiPanel.Hide();
            }
        }

        public static bool IsSteamWorkshopItemSubscribed(ulong itemId)
        {
            return ContentManagerPanel.subscribedItemsTable.Contains(new PublishedFileId(itemId));
        }

        private bool Check3rdPartyModLoaded(string namespaceStr, bool printAll = false)
        {
            bool thirdPartyModLoaded = false;

            var loadingWrapperLoadingExtensionsField = typeof(LoadingWrapper).GetField("m_LoadingExtensions", BindingFlags.NonPublic | BindingFlags.Instance);
            List<ILoadingExtension> loadingExtensions = (List<ILoadingExtension>)loadingWrapperLoadingExtensionsField.GetValue(Singleton<LoadingManager>.instance.m_LoadingWrapper);

            if (loadingExtensions != null)
            {
                foreach (ILoadingExtension extension in loadingExtensions)
                {
                    if (printAll)
                        DebugLog.LogToFileOnly($"Detected extension: {extension.GetType().Name} in namespace {extension.GetType().Namespace}");
                    if (extension.GetType().Namespace == null)
                        continue;

                    var nsStr = extension.GetType().Namespace.ToString();
                    if (namespaceStr.Equals(nsStr))
                    {
                        DebugLog.LogToFileOnly($"The mod '{namespaceStr}' has been detected.");
                        thirdPartyModLoaded = true;
                        break;
                    }
                }
            }
            else
            {
                DebugLog.LogToFileOnly("Could not get loading extensions");
            }

            return thirdPartyModLoaded;
        }

    }
}

