using ColossalFramework;
using ColossalFramework.UI;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using TrafficManager.Manager.Impl;
using TrafficManager.State;
using UnityEngine;

namespace AdvancedJunctionRule
{
    public class RoadUI : UIPanel
    {
        public static readonly string cacheName = "RoadUI";

        private static readonly float SPACING = 15f;

        public RoadWorldInfoPanel baseBuildingWindow;

        public static bool refeshOnce = false;

        public static ushort lastSegment = 0;

        public static UICheckBox CanRightTurn;
        public static UICheckBox CanLeftWaiting;
        public static UICheckBox CanUTurn;

        private UILabel CanRightTurnText;
        private UILabel CanLeftWaitingText;
        private UILabel CanUTurnText;
        private UILabel Tips1;
        private UILabel Tips2;
        private UILabel Tips3;

        public override void Update()
        {
            this.RefreshDisplayData();
            base.Update();
        }

        public override void Awake()
        {
            base.Awake();
            this.DoOnStartup();
        }

        public override void Start()
        {
            base.Start();
            this.canFocus = true;
            this.isInteractive = true;
            base.isVisible = true;
            this.BringToFront();
            base.opacity = 1f;
            base.cachedName = cacheName;
            this.RefreshDisplayData();
            base.Hide();
        }

        private void DoOnStartup()
        {
            this.ShowOnGui();
            base.Hide();
        }


        private void ShowOnGui()
        {
            CanRightTurn = base.AddUIComponent<UICheckBox>();
            CanRightTurn.relativePosition = new Vector3(SPACING, 50f);
            this.CanRightTurnText = base.AddUIComponent<UILabel>();
            this.CanRightTurnText.relativePosition = new Vector3(CanRightTurn.relativePosition.x + CanRightTurn.width + 20f, CanRightTurn.relativePosition.y + 5f);
            CanRightTurn.height = 16f;
            CanRightTurn.width = 16f;
            CanRightTurn.label = this.CanRightTurnText;
            CanRightTurn.text = Language.Strings[0];
            UISprite uISprite0 = CanRightTurn.AddUIComponent<UISprite>();
            uISprite0.height = 20f;
            uISprite0.width = 20f;
            uISprite0.relativePosition = new Vector3(0f, 0f);
            uISprite0.spriteName = "check-unchecked";
            uISprite0.isVisible = true;
            UISprite uISprite1 = CanRightTurn.AddUIComponent<UISprite>();
            uISprite1.height = 20f;
            uISprite1.width = 20f;
            uISprite1.relativePosition = new Vector3(0f, 0f);
            uISprite1.spriteName = "check-checked";
            CanRightTurn.checkedBoxObject = uISprite1;
            CanRightTurn.isChecked = (MainDataStore.canRightTurn[lastSegment]) ? true : false;
            CanRightTurn.isEnabled = true;
            CanRightTurn.isVisible = true;
            CanRightTurn.canFocus = true;
            CanRightTurn.isInteractive = true;
            CanRightTurn.eventCheckChanged += delegate (UIComponent component, bool eventParam)
            {
                CanRightTurn_OnCheckChanged(component, eventParam);
            };

            CanLeftWaiting = base.AddUIComponent<UICheckBox>();
            CanLeftWaiting.relativePosition = new Vector3(15f, CanRightTurn.relativePosition.y + 23f);
            this.CanLeftWaitingText = base.AddUIComponent<UILabel>();
            this.CanLeftWaitingText.relativePosition = new Vector3(CanLeftWaiting.relativePosition.x + CanLeftWaiting.width + 20f, CanLeftWaiting.relativePosition.y + 5f);
            CanLeftWaiting.height = 16f;
            CanLeftWaiting.width = 16f;
            CanLeftWaiting.label = this.CanLeftWaitingText;
            CanLeftWaiting.text = Language.Strings[1];
            UISprite uISprite2 = CanLeftWaiting.AddUIComponent<UISprite>();
            uISprite2.height = 20f;
            uISprite2.width = 20f;
            uISprite2.relativePosition = new Vector3(0f, 0f);
            uISprite2.spriteName = "check-unchecked";
            uISprite2.isVisible = true;
            UISprite uISprite3 = CanLeftWaiting.AddUIComponent<UISprite>();
            uISprite3.height = 20f;
            uISprite3.width = 20f;
            uISprite3.relativePosition = new Vector3(0f, 0f);
            uISprite3.spriteName = "check-checked";
            CanLeftWaiting.checkedBoxObject = uISprite3;
            CanLeftWaiting.isChecked = (MainDataStore.canLeftWaiting[lastSegment]) ? true : false;
            CanLeftWaiting.isEnabled = true;
            CanLeftWaiting.isVisible = true;
            CanLeftWaiting.canFocus = true;
            CanLeftWaiting.isInteractive = true;
            CanLeftWaiting.eventCheckChanged += delegate (UIComponent component, bool eventParam)
            {
                CanLeftWaiting_OnCheckChanged(component, eventParam);
            };

            CanUTurn = base.AddUIComponent<UICheckBox>();
            CanUTurn.relativePosition = new Vector3(15f, CanLeftWaiting.relativePosition.y + 23f);
            this.CanUTurnText = base.AddUIComponent<UILabel>();
            this.CanUTurnText.relativePosition = new Vector3(CanUTurn.relativePosition.x + CanUTurn.width + 20f, CanUTurn.relativePosition.y + 5f);
            CanUTurn.height = 16f;
            CanUTurn.width = 16f;
            CanUTurn.label = this.CanUTurnText;
            CanUTurn.text = Language.Strings[9];
            UISprite uISprite4 = CanUTurn.AddUIComponent<UISprite>();
            uISprite4.height = 20f;
            uISprite4.width = 20f;
            uISprite4.relativePosition = new Vector3(0f, 0f);
            uISprite4.spriteName = "check-unchecked";
            uISprite4.isVisible = true;
            UISprite uISprite5 = CanUTurn.AddUIComponent<UISprite>();
            uISprite5.height = 20f;
            uISprite5.width = 20f;
            uISprite5.relativePosition = new Vector3(0f, 0f);
            uISprite5.spriteName = "check-checked";
            CanUTurn.checkedBoxObject = uISprite5;
            CanUTurn.isChecked = (MainDataStore.canUTurn[lastSegment]) ? true : false;
            CanUTurn.isEnabled = true;
            CanUTurn.isVisible = true;
            CanUTurn.canFocus = true;
            CanUTurn.isInteractive = true;
            CanUTurn.eventCheckChanged += delegate (UIComponent component, bool eventParam)
            {
                CSUR_OnCheckChanged(component, eventParam);
            };

            this.Tips1 = base.AddUIComponent<UILabel>();
            this.Tips1.text = Language.Strings[2];
            this.Tips1.relativePosition = new Vector3(15f, CanUTurn.relativePosition.y + 23f);
            this.Tips1.autoSize = true;

            this.Tips2 = base.AddUIComponent<UILabel>();
            this.Tips2.text = Language.Strings[3];
            this.Tips2.relativePosition = new Vector3(15f, Tips1.relativePosition.y + 23f);
            this.Tips2.autoSize = true;

            this.Tips3 = base.AddUIComponent<UILabel>();
            this.Tips3.text = Language.Strings[10];
            this.Tips3.relativePosition = new Vector3(15f, Tips2.relativePosition.y + 23f);
            this.Tips3.autoSize = true;

        }


        public static void CanRightTurn_OnCheckChanged(UIComponent UIComp, bool bValue)
        {
            if (WorldInfoPanel.GetCurrentInstanceID().Type == InstanceType.NetSegment)
            {
                lastSegment = WorldInfoPanel.GetCurrentInstanceID().NetSegment;

                if (bValue)
                {
                    MainDataStore.canRightTurn[lastSegment] = true;
                    CanRightTurn.isChecked = true;
                }
                else
                {
                    MainDataStore.canRightTurn[lastSegment] = false;
                    CanRightTurn.isChecked = false;
                }
                refeshOnce = true;
            }
        }


        public static void CSUR_OnCheckChanged(UIComponent UIComp, bool bValue)
        {
            if (WorldInfoPanel.GetCurrentInstanceID().Type == InstanceType.NetSegment)
            {
                lastSegment = WorldInfoPanel.GetCurrentInstanceID().NetSegment;
                if (bValue)
                {
                    MainDataStore.canUTurn[lastSegment] = true;
                    CanUTurn.isChecked = true;
                }
                else
                {
                    MainDataStore.canUTurn[lastSegment] = false;
                    CanUTurn.isChecked = false;
                }
                refeshOnce = true;
            }
        }


        public static void CanLeftWaiting_OnCheckChanged(UIComponent UIComp, bool bValue)
        {
            if (WorldInfoPanel.GetCurrentInstanceID().Type == InstanceType.NetSegment)
            {
                lastSegment = WorldInfoPanel.GetCurrentInstanceID().NetSegment;
                if (bValue)
                {
                    MainDataStore.canLeftWaiting[lastSegment] = true;
                    CanLeftWaiting.isChecked = true;
                }
                else
                {
                    MainDataStore.canLeftWaiting[lastSegment] = false;
                    CanLeftWaiting.isChecked = false;
                }
                refeshOnce = true;
            }
        }


        private void RefreshDisplayData()
        {
            if (refeshOnce || (lastSegment != WorldInfoPanel.GetCurrentInstanceID().NetSegment && WorldInfoPanel.GetCurrentInstanceID().Type == InstanceType.NetSegment))
            {
                if (base.isVisible)
                {
                    if (WorldInfoPanel.GetCurrentInstanceID().Type == InstanceType.NetSegment)
                    {
                        lastSegment = WorldInfoPanel.GetCurrentInstanceID().NetSegment;
                    }
                    var instance = Singleton<NetManager>.instance;
                    var startNode = instance.m_segments.m_buffer[lastSegment].m_startNode;
                    var endNode = instance.m_segments.m_buffer[lastSegment].m_endNode;
                    if (Options.timedLightsEnabled && (TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)startNode].IsSimulationRunning() || TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)endNode].IsSimulationRunning()))
                    {
                        Tips2.text = Language.Strings[3];
                    }
                    else
                    {
                        Tips2.text = Language.Strings[3] + Language.Strings[6];
                        MainDataStore.canLeftWaiting[lastSegment] = false;
                    }

                    if (!Options.timedLightsEnabled || (!TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)startNode].IsSimulationRunning() && !TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)endNode].IsSimulationRunning()))
                    {
                        if (instance.m_nodes.m_buffer[startNode].m_flags.IsFlagSet(NetNode.Flags.TrafficLights) || instance.m_nodes.m_buffer[endNode].m_flags.IsFlagSet(NetNode.Flags.TrafficLights))
                        {
                            Tips1.text = Language.Strings[2];
                        }
                        else
                        {
                            Tips1.text = Language.Strings[2] + Language.Strings[5];
                            MainDataStore.canRightTurn[lastSegment] = false;
                        }
                    }
                    else
                    {
                        Tips1.text = Language.Strings[2] + Language.Strings[4];
                        MainDataStore.canRightTurn[lastSegment] = false;
                    }


                    if (instance.m_nodes.m_buffer[startNode].m_flags.IsFlagSet(NetNode.Flags.TrafficLights) || instance.m_nodes.m_buffer[endNode].m_flags.IsFlagSet(NetNode.Flags.TrafficLights))
                    {
                        Tips3.text = Language.Strings[10];
                    }
                    else
                    {
                        Tips3.text = Language.Strings[10] + Language.Strings[5];
                        MainDataStore.canUTurn[lastSegment] = false;
                    }

                    //Tips1.text += MainDataStore.canRightTurn[lastSegment].ToString();
                    //Tips2.text += MainDataStore.canLeftWaiting[lastSegment].ToString();

                    CanLeftWaiting.isChecked = (MainDataStore.canLeftWaiting[lastSegment]) ? true : false;
                    CanRightTurn.isChecked = (MainDataStore.canRightTurn[lastSegment]) ? true : false;
                    CanUTurn.isChecked = (MainDataStore.canUTurn[lastSegment]) ? true : false;
                    refeshOnce = false;
                    this.BringToFront();
                }
                else
                {
                    this.Hide();
                }
            }
        }
    }
}
