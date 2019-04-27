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
using AdvancedJunctionRule.CustomAI;

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
        public static bool isRealGasStationRunning = false;
        public static bool isTrafficCongestionReportRunning = false;

        public class Detour
        {
            public MethodInfo OriginalMethod;
            public MethodInfo CustomMethod;
            public RedirectCallsState Redirect;

            public Detour(MethodInfo originalMethod, MethodInfo customMethod)
            {
                this.OriginalMethod = originalMethod;
                this.CustomMethod = customMethod;
                this.Redirect = RedirectionHelper.RedirectCalls(originalMethod, customMethod);
            }
        }

        public static List<Detour> Detours { get; set; }
        public static bool DetourInited = false;

        public override void OnCreated(ILoading loading)
        {
            base.OnCreated(loading);
            Detours = new List<Detour>();
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
                    InitDetour();
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
                AdvancedJunctionRuleThreading.isFirstTime = true;
            }
        }

        public override void OnReleased()
        {
            base.OnReleased();
        }

        public void InitDetour()
        {
            isRealGasStationRunning = Check3rdPartyModLoaded("RealGasStation", true);
            isTrafficCongestionReportRunning = Check3rdPartyModLoaded("TrafficCongestionReport", true);

            if (!DetourInited)
            {
                DebugLog.LogToFileOnly("Init detours");
                bool detourFailed = false;

                //1
                DebugLog.LogToFileOnly("Detour CargoTruckAI::SetTarget calls");
                try
                {
                    Detours.Add(new Detour(typeof(CarAI).GetMethod("SimulationStep", BindingFlags.Instance | BindingFlags.Public, null, new Type[] {
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(Vehicle.Frame).MakeByRefType(),
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(int)}, null),
                typeof(NewCarAI).GetMethod("CustomSimulationStep", BindingFlags.Instance | BindingFlags.Public, null, new Type[]{
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(Vehicle.Frame).MakeByRefType(),
                typeof(ushort),
                typeof(Vehicle).MakeByRefType(),
                typeof(int)}, null)));
                }
                catch (Exception)
                {
                    DebugLog.LogToFileOnly("Could not detour CargoTruckAI::SetTarget");
                    detourFailed = true;
                }

                if (detourFailed)
                {
                    DebugLog.LogToFileOnly("Detours failed");
                }
                else
                {
                    DebugLog.LogToFileOnly("Detours successful");
                }
                DetourInited = true;
            }
        }

        public void RevertDetour()
        {
            if (DetourInited)
            {
                DebugLog.LogToFileOnly("Revert detours");
                Detours.Reverse();
                foreach (Detour d in Detours)
                {
                    RedirectionHelper.RevertRedirect(d.OriginalMethod, d.Redirect);
                }
                DetourInited = false;
                Detours.Clear();
                DebugLog.LogToFileOnly("Reverting detours finished.");
            }
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
            //guiPanel.position = new Vector3(roadInfo.size.x, roadInfo.size.y);
            guiPanel.position = new Vector3(0, 12);
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
                Loader.guiPanel.position = new Vector3(0, 12);
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

