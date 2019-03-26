using ColossalFramework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using TrafficManager;
using TrafficManager.Custom.AI;
using TrafficManager.Geometry.Impl;
using TrafficManager.Manager;
using TrafficManager.Manager.Impl;
using TrafficManager.State;
using TrafficManager.Traffic;
using UnityEngine;
using TrafficManager.Traffic.Data;
using ColossalFramework.Math;
using TrafficManager.TrafficLight;
using AdvancedJunctionRule.Util;
using static TrafficManager.Traffic.Data.PrioritySegment;

namespace AdvancedJunctionRule.CustomAI
{
    public class NewCarAI : VehicleAI
    {
        private bool ProcessLeftWaiting(ushort vehicleID, Vehicle vehicleData, ushort nodeId, ushort fromSegmentId, byte fromLaneIndex, ushort toSegmentId, uint nextLaneId)
        {
            if (!Options.timedLightsEnabled || !TrafficLightSimulationManager.Instance.TrafficLightSimulations[(int)nodeId].IsSimulationRunning())
            {
            }
            else
            {
                SegmentGeometry segmentGeometry = SegmentGeometry.Get(fromSegmentId, false);
                if (segmentGeometry == null)
                {
                }
                else
                {
                    bool startNode = segmentGeometry.StartNodeId() == nodeId;
                    ICustomSegmentLights segmentLights = CustomSegmentLightsManager.Instance.GetSegmentLights(fromSegmentId, startNode, false, RoadBaseAI.TrafficLightState.Red);
                    ICustomSegmentLight customSegmentLight = (segmentLights == null) ? null : segmentLights.GetCustomLight(fromLaneIndex);
                    if (MainDataStore.canLeftWaiting[fromSegmentId])
                    {
                        if (segmentGeometry.IsLeftSegment(toSegmentId, startNode))
                        {
                            if (customSegmentLight.LightLeft == RoadBaseAI.TrafficLightState.Red && customSegmentLight.LightMain == RoadBaseAI.TrafficLightState.Green)
                            {
                                //DebugLog.LogToFileOnly("try Left Waiting");
                                PathManager pathMan = Singleton<PathManager>.instance;
                                byte finePathPosIndex = vehicleData.m_pathPositionIndex;
                                PathUnit.Position currentPosition;
                                uint pathId = vehicleData.m_path;
                                pathMan.m_pathUnits.m_buffer[pathId].GetPosition(finePathPosIndex >> 1, out currentPosition);
                                uint curLaneId = PathManager.GetLaneID(currentPosition);

                                Vector4 targetPos = vehicleData.m_targetPos0;
                                byte nextSegOffset = 0;
                                PathUnit.CalculatePathPositionOffset(nextLaneId, targetPos, out nextSegOffset);

                                if (!MainDataStore.isLeftWaiting[vehicleID])
                                {
                                    MainDataStore.isLeftWaiting[vehicleID] = true;
                                    //MainDataStore.additionWaitingTime[vehicleID] = (byte)(distance / 5f);
                                    MainDataStore.nodeId[vehicleID] = nodeId;
                                    MainDataStore.fromSegmentId[vehicleID] = fromSegmentId;
                                    MainDataStore.fromLaneIndex[vehicleID] = fromLaneIndex;
                                    MainDataStore.toSegmentId[vehicleID] = toSegmentId;
                                    MainDataStore.laneID[vehicleID] = curLaneId;
                                    MainDataStore.offset[vehicleID] = currentPosition.m_offset;
                                    MainDataStore.nextLaneId[vehicleID] = nextLaneId;
                                    MainDataStore.nextSegOffset[vehicleID] = nextSegOffset;
                                    MainDataStore.crossStopLine[vehicleID] = false;
                                }
                                return true;
                                //Vector3 tmpPosition1 = instance.m_lanes.m_buffer[laneId].CalculatePosition((offset) * 0.003921569f);
                            }
                            else
                            {
                                //MainDataStore.isLeftWaiting[vehicleID] = false;
                                //MainDataStore.nodeId[vehicleID] = 0;
                                //MainDataStore.fromSegmentId[vehicleID] = 0;
                                //MainDataStore.fromLaneIndex[vehicleID] = 0;
                                //MainDataStore.toSegmentId[vehicleID] = 0;
                                //MainDataStore.laneID[vehicleID] = 0;
                                //MainDataStore.offset[vehicleID] = 0;
                                //MainDataStore.finePathPosIndex[vehicleID] = 0;
                                //MainDataStore.nextLaneId[vehicleID] = 0;
                                //MainDataStore.nextSegOffset[vehicleID] = 0;
                            }
                        }
                        else
                        {
                            //MainDataStore.isLeftWaiting[vehicleID] = false;
                            //MainDataStore.nodeId[vehicleID] = 0;
                            //MainDataStore.fromSegmentId[vehicleID] = 0;
                            //MainDataStore.fromLaneIndex[vehicleID] = 0;
                            //MainDataStore.toSegmentId[vehicleID] = 0;
                            //MainDataStore.laneID[vehicleID] = 0;
                            //MainDataStore.offset[vehicleID] = 0;
                            //MainDataStore.finePathPosIndex[vehicleID] = 0;
                            //MainDataStore.nextLaneId[vehicleID] = 0;
                            //MainDataStore.nextSegOffset[vehicleID] = 0;
                            //MainDataStore.crossStopLine[vehicleID] = false;
                        }
                    }
                }
            }
            return false;
        }

        protected bool MayChangeSegment(ushort frontVehicleId, ref VehicleState vehicleState, ref Vehicle vehicleData, float sqrVelocity, ref PathUnit.Position prevPos, ref NetSegment prevSegment, ushort prevTargetNodeId, uint prevLaneID, ref PathUnit.Position position, ushort targetNodeId, ref NetNode targetNode, uint laneID, ref PathUnit.Position nextPosition, ushort nextTargetNodeId, out float maxSpeed)
        {
            //public bool MayChangeSegment(ushort frontVehicleId, ref VehicleState vehicleState, ref Vehicle vehicleData, float sqrVelocity, bool isRecklessDriver, ref PathUnit.Position prevPos, ref NetSegment prevSegment, ushort prevTargetNodeId, uint prevLaneID, ref PathUnit.Position position, ushort targetNodeId, ref NetNode targetNode, uint laneID, ref PathUnit.Position nextPosition, ushort nextTargetNodeId, out float maxSpeed) {
#if DEBUG
			bool debug = GlobalConfig.Instance.Debug.Switches[13] && (GlobalConfig.Instance.Debug.NodeId <= 0 || targetNodeId == GlobalConfig.Instance.Debug.NodeId);
#endif

#if BENCHMARK
			//using (var bm = new Benchmark(null, "MayDespawn")) {
#endif

            if (prevTargetNodeId != targetNodeId
                || (vehicleData.m_blockCounter == 255 && !VehicleBehaviorManager.Instance.MayDespawn(ref vehicleData)) // NON-STOCK CODE
            )
            {
                // method should only be called if targetNodeId == prevTargetNode
                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                maxSpeed = 0f;
                return true;
            }
#if BENCHMARK
			//}
#endif


            if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.Leave)
            {
                // vehicle was already allowed to leave the junction
                maxSpeed = 0f;
                return true;
            }

            if ((vehicleState.JunctionTransitState == VehicleJunctionTransitState.Stop || vehicleState.JunctionTransitState == VehicleJunctionTransitState.Blocked) &&
                vehicleState.lastTransitStateUpdate >> VehicleState.JUNCTION_RECHECK_SHIFT >= Constants.ServiceFactory.SimulationService.CurrentFrameIndex >> VehicleState.JUNCTION_RECHECK_SHIFT)
            {
                // reuse recent result
                maxSpeed = 0f;
                return false;
            }

            bool isRecklessDriver = vehicleState.recklessDriver;

            var netManager = Singleton<NetManager>.instance;

            bool hasActiveTimedSimulation = (Options.timedLightsEnabled && TrafficLightSimulationManager.Instance.HasActiveTimedSimulation(targetNodeId));
            bool hasTrafficLightFlag = (targetNode.m_flags & NetNode.Flags.TrafficLights) != NetNode.Flags.None;
            if (hasActiveTimedSimulation && !hasTrafficLightFlag)
            {
                TrafficLightManager.Instance.AddTrafficLight(targetNodeId, ref targetNode);
            }
            bool hasTrafficLight = hasTrafficLightFlag || hasActiveTimedSimulation;
            bool checkTrafficLights = true;
            bool isTargetStartNode = prevSegment.m_startNode == targetNodeId;
            bool isLevelCrossing = (targetNode.m_flags & NetNode.Flags.LevelCrossing) != NetNode.Flags.None;
            if ((vehicleData.Info.m_vehicleType & (VehicleInfo.VehicleType.Train | VehicleInfo.VehicleType.Metro | VehicleInfo.VehicleType.Monorail)) == VehicleInfo.VehicleType.None)
            {
                // check if to check space

#if DEBUG
				if (debug)
					Log._Debug($"CustomVehicleAI.MayChangeSegment: Vehicle {frontVehicleId} is not a train.");
#endif

                // stock priority signs
                if ((vehicleData.m_flags & Vehicle.Flags.Emergency2) == (Vehicle.Flags)0 &&
                    ((NetLane.Flags)netManager.m_lanes.m_buffer[prevLaneID].m_flags & (NetLane.Flags.YieldStart | NetLane.Flags.YieldEnd)) != NetLane.Flags.None &&
                    (targetNode.m_flags & (NetNode.Flags.Junction | NetNode.Flags.TrafficLights | NetNode.Flags.OneWayIn)) == NetNode.Flags.Junction)
                {
                    if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                    {
                        if ((vehicleData.m_flags2 & Vehicle.Flags2.Yielding) == (Vehicle.Flags2)0)
                        {
                            if (sqrVelocity < 0.01f)
                            {
                                vehicleData.m_flags2 |= Vehicle.Flags2.Yielding;
                            }
                            else
                            {
                                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                            }
                            maxSpeed = 0f;
                            return false;
                        }
                        else
                        {
                            vehicleData.m_waitCounter = (byte)Mathf.Min((int)(vehicleData.m_waitCounter + 1), 4);
                            if (vehicleData.m_waitCounter < 4)
                            {
                                maxSpeed = 0f;
                                return false;
                            }
                            vehicleData.m_flags2 &= ~Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }
                    }
                    else if (sqrVelocity > 0.01f)
                    {
                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                        maxSpeed = 0f;
                        return false;
                    }
                }

                // entering blocked junctions
                if (MustCheckSpace(prevPos.m_segment, isTargetStartNode, ref targetNode, isRecklessDriver))
                {
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckSpace")) {
#endif
                    // check if there is enough space
                    var len = vehicleState.totalLength + 4f;
                    if (!netManager.m_lanes.m_buffer[laneID].CheckSpace(len))
                    {
                        var sufficientSpace = false;
                        if (nextPosition.m_segment != 0 && netManager.m_lanes.m_buffer[laneID].m_length < 30f)
                        {
                            NetNode.Flags nextTargetNodeFlags = netManager.m_nodes.m_buffer[nextTargetNodeId].m_flags;
                            if ((nextTargetNodeFlags & (NetNode.Flags.Junction | NetNode.Flags.OneWayOut | NetNode.Flags.OneWayIn)) != NetNode.Flags.Junction ||
                                netManager.m_nodes.m_buffer[nextTargetNodeId].CountSegments() == 2)
                            {
                                uint nextLaneId = PathManager.GetLaneID(nextPosition);
                                if (nextLaneId != 0u)
                                {
                                    sufficientSpace = netManager.m_lanes.m_buffer[nextLaneId].CheckSpace(len);
                                }
                            }
                        }

                        if (!sufficientSpace)
                        {
                            maxSpeed = 0f;
#if DEBUG
							if (debug)
								Log._Debug($"Vehicle {frontVehicleId}: Setting JunctionTransitState to BLOCKED");
#endif

                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Blocked;
                            return false;
                        }
                    }
#if BENCHMARK
					//}
#endif
                }

                bool isJoinedJunction = ((NetLane.Flags)netManager.m_lanes.m_buffer[prevLaneID].m_flags & NetLane.Flags.JoinedJunction) != NetLane.Flags.None;
                checkTrafficLights = !isJoinedJunction || isLevelCrossing;
            }
            else
            {
#if DEBUG
				if (debug)
					Log._Debug($"CustomVehicleAI.MayChangeSegment: Vehicle {frontVehicleId} is a train/metro/monorail.");
#endif

                if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Monorail)
                {
                    // vanilla traffic light flags are not rendered on monorail tracks
                    checkTrafficLights = hasActiveTimedSimulation;
                }
                else if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                {
                    // vanilla traffic light flags are not rendered on train tracks, except for level crossings
                    checkTrafficLights = hasActiveTimedSimulation || isLevelCrossing;
                }
            }

            if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.Blocked)
            {
#if DEBUG
				if (debug)
					Log._Debug($"Vehicle {frontVehicleId}: Setting JunctionTransitState from BLOCKED to APPROACH");
#endif
                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Approach;
            }

            ITrafficPriorityManager prioMan = TrafficPriorityManager.Instance;
            ICustomSegmentLightsManager segLightsMan = CustomSegmentLightsManager.Instance;
            if ((vehicleData.m_flags & Vehicle.Flags.Emergency2) == 0 || isLevelCrossing)
            {
                if (hasTrafficLight && checkTrafficLights)
                {
#if DEBUG
					if (debug) {
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Node {targetNodeId} has a traffic light.");
					}
#endif
                    RoadBaseAI.TrafficLightState vehicleLightState;
                    bool stopCar = false;
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckTrafficLight")) {
#endif

                    var destinationInfo = targetNode.Info;

                    uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
                    uint targetNodeLower8Bits = (uint)((targetNodeId << 8) / 32768);

                    RoadBaseAI.TrafficLightState pedestrianLightState;
                    bool vehicles;
                    bool pedestrians;
                    CustomRoadAI.GetTrafficLightState(
#if DEBUG
							frontVehicleId, ref vehicleData,
#endif
                            targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, ref prevSegment, currentFrameIndex - targetNodeLower8Bits, out vehicleLightState, out pedestrianLightState, out vehicles, out pedestrians);

                    if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Car && isRecklessDriver && !isLevelCrossing)
                    {
                        vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                    }

#if DEBUG
					if (debug)
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle {frontVehicleId} has TL state {vehicleLightState} at node {targetNodeId}");
#endif

                    uint random = currentFrameIndex - targetNodeLower8Bits & 255u;
                    if (!vehicles && random >= 196u)
                    {
                        vehicles = true;
                        RoadBaseAI.SetTrafficLightState(targetNodeId, ref prevSegment, currentFrameIndex - targetNodeLower8Bits, vehicleLightState, pedestrianLightState, vehicles, pedestrians);
                    }

                    switch (vehicleLightState)
                    {
                        case RoadBaseAI.TrafficLightState.RedToGreen:
                            if (random < 60u)
                            {
                                if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                                {
                                    stopCar = true;
                                }
                            }
                            break;
                        case RoadBaseAI.TrafficLightState.Red:
                            if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                            {
                                stopCar = true;
                            }
                            break;
                        case RoadBaseAI.TrafficLightState.GreenToRed:
                            if (random >= 30u)
                            {
                                if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                                {
                                    stopCar = true;
                                }
                            }
                            break;
                    }
#if BENCHMARK
					//}
#endif

                    // Turn-on-red: Check if turning in the preferred direction, and if turning while it's red is allowed
                    if (
                        Options.turnOnRedEnabled &&
                        stopCar &&
                        sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxYieldVelocity * GlobalConfig.Instance.PriorityRules.MaxYieldVelocity &&
                        !isRecklessDriver
                    )
                    {
                        IJunctionRestrictionsManager junctionRestrictionsManager = Constants.ManagerFactory.JunctionRestrictionsManager;
                        ITurnOnRedManager turnOnRedMan = Constants.ManagerFactory.TurnOnRedManager;
                        bool lhd = Constants.ServiceFactory.SimulationService.LeftHandDrive;
                        int torIndex = turnOnRedMan.GetIndex(prevPos.m_segment, isTargetStartNode);
                        if (
                            (turnOnRedMan.TurnOnRedSegments[torIndex].leftSegmentId == position.m_segment &&
                            junctionRestrictionsManager.IsTurnOnRedAllowed(lhd, prevPos.m_segment, isTargetStartNode)) ||
                            (turnOnRedMan.TurnOnRedSegments[torIndex].rightSegmentId == position.m_segment &&
                            junctionRestrictionsManager.IsTurnOnRedAllowed(!lhd, prevPos.m_segment, isTargetStartNode))
                        )
                        {
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle may turn on red to target segment {position.m_segment}, lane {position.m_lane}");
#endif
                            stopCar = false;
                        }
                    }

                    // check priority rules at unprotected traffic lights
                    if (!stopCar && Options.prioritySignsEnabled && Options.trafficLightPriorityRules && segLightsMan.IsSegmentLight(prevPos.m_segment, isTargetStartNode))
                    {
                        bool hasPriority = true;
#if BENCHMARK
						//using (var bm = new Benchmark(null, "CheckPriorityRulesAtTTL")) {
#endif
                        hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if BENCHMARK
						//}
#endif

                        if (!hasPriority)
                        {
                            // green light but other cars are incoming and they have priority: stop
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Green traffic light (or turn on red allowed) but detected traffic with higher priority: stop.");
#endif
                            stopCar = true;
                        }
                    }

                    if (stopCar)
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP");
#endif

                        if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                        {
                            vehicleData.m_flags2 |= Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }

                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                        maxSpeed = 0f;
                        vehicleData.m_blockCounter = 0;
                        return false;
                    }
                    else
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE ({vehicleLightState})");
#endif
                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;

                        if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                        {
                            vehicleData.m_flags2 &= ~Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }
                    }
                }
                else if (Options.prioritySignsEnabled && vehicleData.Info.m_vehicleType != VehicleInfo.VehicleType.Monorail)
                {
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckPriorityRules")) {
#endif

#if DEBUG
					//bool debug = destinationNodeId == 10864;
					//bool debug = destinationNodeId == 13531;
					//bool debug = false;// targetNodeId == 5027;
#endif
                    //bool debug = false;
#if DEBUG
					if (debug)
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle is arriving @ seg. {prevPos.m_segment} ({position.m_segment}, {nextPosition.m_segment}), node {targetNodeId} which is not a traffic light.");
#endif

                    var sign = prioMan.GetPrioritySign(prevPos.m_segment, isTargetStartNode);
                    if (sign != PrioritySegment.PriorityType.None)
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle is arriving @ seg. {prevPos.m_segment} ({position.m_segment}, {nextPosition.m_segment}), node {targetNodeId} which is not a traffic light and is a priority segment.");
#endif

#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): JunctionTransitState={vehicleState.JunctionTransitState.ToString()}");
#endif

                        if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.None)
                        {
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to APPROACH (prio)");
#endif
                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Approach;
                        }

                        if (vehicleState.JunctionTransitState != VehicleJunctionTransitState.Leave)
                        {
                            bool hasPriority;
                            switch (sign)
                            {
                                case PriorityType.Stop:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): STOP sign. waittime={vehicleState.waitTime}, sqrVelocity={sqrVelocity}");
#endif

                                    maxSpeed = 0f;

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait) waitTime={vehicleState.waitTime}");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxStopVelocity * GlobalConfig.Instance.PriorityRules.MaxStopVelocity)
                                        {
                                            vehicleState.waitTime++;

                                            //float minStopWaitTime = Singleton<SimulationManager>.instance.m_randomizer.UInt32(3);
                                            if (vehicleState.waitTime >= 2)
                                            {
                                                if (Options.simAccuracy >= 4)
                                                {
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }
                                                else
                                                {
                                                    hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): hasPriority={hasPriority}");
#endif

                                                    if (!hasPriority)
                                                    {
                                                        vehicleData.m_blockCounter = 0;
                                                        return false;
                                                    }
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (min wait timeout)");
#endif
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }

#if DEBUG
												if (debug)
													Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle must first come to a full stop in front of the stop sign.");
#endif
                                                return false;
                                            }
                                        }
                                        else
                                        {
#if DEBUG
											if (debug)
												Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle has come to a full stop.");
#endif
                                            vehicleState.waitTime = 0;
                                            return false;
                                        }
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Max. wait time exceeded. Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    break;
                                case PriorityType.Yield:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): YIELD sign. waittime={vehicleState.waitTime}");
#endif

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
                                        vehicleState.waitTime++;
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxYieldVelocity * GlobalConfig.Instance.PriorityRules.MaxYieldVelocity || Options.simAccuracy <= 2)
                                        {
                                            if (Options.simAccuracy >= 4)
                                            {
                                                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                            }
                                            else
                                            {
                                                hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
												if (debug)
													Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): hasPriority: {hasPriority}");
#endif

                                                if (!hasPriority)
                                                {
                                                    vehicleData.m_blockCounter = 0;
                                                    maxSpeed = 0f;
                                                    return false;
                                                }
                                                else
                                                {
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (no incoming cars)");
#endif
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }
                                            }
                                        }
                                        else
                                        {
#if DEBUG
											if (debug)
												Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle has not yet reached yield speed (reduce {sqrVelocity} by {vehicleState.reduceSqrSpeedByValueToYield})");
#endif

                                            // vehicle has not yet reached yield speed
                                            maxSpeed = GlobalConfig.Instance.PriorityRules.MaxYieldVelocity;
                                            return false;
                                        }
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    break;
                                case PriorityType.Main:
                                default:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): MAIN sign. waittime={vehicleState.waitTime}");
#endif
                                    maxSpeed = 0f;

                                    if (Options.simAccuracy == 4)
                                        return true;

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
                                        vehicleState.waitTime++;
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): {hasPriority}");
#endif

                                        if (!hasPriority)
                                        {
                                            vehicleData.m_blockCounter = 0;
                                            return false;
                                        }
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (no conflicting car)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Max. wait time exceeded. Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    return true;
                            }
                        }
                        else if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxStopVelocity * GlobalConfig.Instance.PriorityRules.MaxStopVelocity && (vehicleState.vehicleType & ExtVehicleType.RoadVehicle) != ExtVehicleType.None)
                        {
                            // vehicle is not moving. reset allowance to leave junction
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState from LEAVE to BLOCKED (speed to low)");
#endif
                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Blocked;

                            maxSpeed = 0f;
                            return false;
                        }
                    }
#if BENCHMARK
					//}
#endif
                }
            }
            maxSpeed = 0f; // maxSpeed should be set by caller
            return true;
        }

        protected bool OldTMPEMayChangeSegment(ushort frontVehicleId, ref VehicleState vehicleState, ref Vehicle vehicleData, float sqrVelocity, ref PathUnit.Position prevPos, ref NetSegment prevSegment, ushort prevTargetNodeId, uint prevLaneID, ref PathUnit.Position position, ushort targetNodeId, ref NetNode targetNode, uint laneID, ref PathUnit.Position nextPosition, ushort nextTargetNodeId, out float maxSpeed)
        {
            //public bool MayChangeSegment(ushort frontVehicleId, ref VehicleState vehicleState, ref Vehicle vehicleData, float sqrVelocity, bool isRecklessDriver, ref PathUnit.Position prevPos, ref NetSegment prevSegment, ushort prevTargetNodeId, uint prevLaneID, ref PathUnit.Position position, ushort targetNodeId, ref NetNode targetNode, uint laneID, ref PathUnit.Position nextPosition, ushort nextTargetNodeId, out float maxSpeed) {
#if DEBUG
			bool debug = GlobalConfig.Instance.Debug.Switches[13] && (GlobalConfig.Instance.Debug.NodeId <= 0 || targetNodeId == GlobalConfig.Instance.Debug.NodeId);
#endif

#if BENCHMARK
			//using (var bm = new Benchmark(null, "MayDespawn")) {
#endif

            if (prevTargetNodeId != targetNodeId
                || (vehicleData.m_blockCounter == 255 && !VehicleBehaviorManager.Instance.MayDespawn(ref vehicleData)) // NON-STOCK CODE
            )
            {
                // method should only be called if targetNodeId == prevTargetNode
                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                maxSpeed = 0f;
                return true;
            }
#if BENCHMARK
			//}
#endif


            if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.Leave)
            {
                // vehicle was already allowed to leave the junction
                maxSpeed = 0f;
                return true;
            }

            if ((vehicleState.JunctionTransitState == VehicleJunctionTransitState.Stop || vehicleState.JunctionTransitState == VehicleJunctionTransitState.Blocked) &&
                vehicleState.lastTransitStateUpdate >> VehicleState.JUNCTION_RECHECK_SHIFT >= Constants.ServiceFactory.SimulationService.CurrentFrameIndex >> VehicleState.JUNCTION_RECHECK_SHIFT)
            {
                // reuse recent result
                maxSpeed = 0f;
                return false;
            }

            bool isRecklessDriver = vehicleState.recklessDriver;

            var netManager = Singleton<NetManager>.instance;

            bool hasActiveTimedSimulation = (Options.timedLightsEnabled && TrafficLightSimulationManager.Instance.HasActiveTimedSimulation(targetNodeId));
            bool hasTrafficLightFlag = (targetNode.m_flags & NetNode.Flags.TrafficLights) != NetNode.Flags.None;
            if (hasActiveTimedSimulation && !hasTrafficLightFlag)
            {
                TrafficLightManager.Instance.AddTrafficLight(targetNodeId, ref targetNode);
            }
            bool hasTrafficLight = hasTrafficLightFlag || hasActiveTimedSimulation;
            bool checkTrafficLights = true;
            bool isTargetStartNode = prevSegment.m_startNode == targetNodeId;
            bool isLevelCrossing = (targetNode.m_flags & NetNode.Flags.LevelCrossing) != NetNode.Flags.None;
            if ((vehicleData.Info.m_vehicleType & (VehicleInfo.VehicleType.Train | VehicleInfo.VehicleType.Metro | VehicleInfo.VehicleType.Monorail)) == VehicleInfo.VehicleType.None)
            {
                // check if to check space

#if DEBUG
				if (debug)
					Log._Debug($"CustomVehicleAI.MayChangeSegment: Vehicle {frontVehicleId} is not a train.");
#endif

                // stock priority signs
                if ((vehicleData.m_flags & Vehicle.Flags.Emergency2) == (Vehicle.Flags)0 &&
                    ((NetLane.Flags)netManager.m_lanes.m_buffer[prevLaneID].m_flags & (NetLane.Flags.YieldStart | NetLane.Flags.YieldEnd)) != NetLane.Flags.None &&
                    (targetNode.m_flags & (NetNode.Flags.Junction | NetNode.Flags.TrafficLights | NetNode.Flags.OneWayIn)) == NetNode.Flags.Junction)
                {
                    if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                    {
                        if ((vehicleData.m_flags2 & Vehicle.Flags2.Yielding) == (Vehicle.Flags2)0)
                        {
                            if (sqrVelocity < 0.01f)
                            {
                                vehicleData.m_flags2 |= Vehicle.Flags2.Yielding;
                            }
                            else
                            {
                                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                            }
                            maxSpeed = 0f;
                            return false;
                        }
                        else
                        {
                            vehicleData.m_waitCounter = (byte)Mathf.Min((int)(vehicleData.m_waitCounter + 1), 4);
                            if (vehicleData.m_waitCounter < 4)
                            {
                                maxSpeed = 0f;
                                return false;
                            }
                            vehicleData.m_flags2 &= ~Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }
                    }
                    else if (sqrVelocity > 0.01f)
                    {
                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                        maxSpeed = 0f;
                        return false;
                    }
                }

                // entering blocked junctions
                if (MustCheckSpace(prevPos.m_segment, isTargetStartNode, ref targetNode, isRecklessDriver))
                {
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckSpace")) {
#endif
                    // check if there is enough space
                    var len = vehicleState.totalLength + 4f;
                    if (!netManager.m_lanes.m_buffer[laneID].CheckSpace(len))
                    {
                        var sufficientSpace = false;
                        if (nextPosition.m_segment != 0 && netManager.m_lanes.m_buffer[laneID].m_length < 30f)
                        {
                            NetNode.Flags nextTargetNodeFlags = netManager.m_nodes.m_buffer[nextTargetNodeId].m_flags;
                            if ((nextTargetNodeFlags & (NetNode.Flags.Junction | NetNode.Flags.OneWayOut | NetNode.Flags.OneWayIn)) != NetNode.Flags.Junction ||
                                netManager.m_nodes.m_buffer[nextTargetNodeId].CountSegments() == 2)
                            {
                                uint nextLaneId = PathManager.GetLaneID(nextPosition);
                                if (nextLaneId != 0u)
                                {
                                    sufficientSpace = netManager.m_lanes.m_buffer[nextLaneId].CheckSpace(len);
                                }
                            }
                        }

                        if (!sufficientSpace)
                        {
                            maxSpeed = 0f;
#if DEBUG
							if (debug)
								Log._Debug($"Vehicle {frontVehicleId}: Setting JunctionTransitState to BLOCKED");
#endif

                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Blocked;
                            return false;
                        }
                    }
#if BENCHMARK
					//}
#endif
                }

                bool isJoinedJunction = ((NetLane.Flags)netManager.m_lanes.m_buffer[prevLaneID].m_flags & NetLane.Flags.JoinedJunction) != NetLane.Flags.None;
                checkTrafficLights = !isJoinedJunction || isLevelCrossing;
            }
            else
            {
#if DEBUG
				if (debug)
					Log._Debug($"CustomVehicleAI.MayChangeSegment: Vehicle {frontVehicleId} is a train/metro/monorail.");
#endif

                if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Monorail)
                {
                    // vanilla traffic light flags are not rendered on monorail tracks
                    checkTrafficLights = hasActiveTimedSimulation;
                }
                else if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                {
                    // vanilla traffic light flags are not rendered on train tracks, except for level crossings
                    checkTrafficLights = hasActiveTimedSimulation || isLevelCrossing;
                }
            }

            if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.Blocked)
            {
#if DEBUG
				if (debug)
					Log._Debug($"Vehicle {frontVehicleId}: Setting JunctionTransitState from BLOCKED to APPROACH");
#endif
                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Approach;
            }

            ITrafficPriorityManager prioMan = TrafficPriorityManager.Instance;
            ICustomSegmentLightsManager segLightsMan = CustomSegmentLightsManager.Instance;
            if ((vehicleData.m_flags & Vehicle.Flags.Emergency2) == 0 || isLevelCrossing)
            {
                if (hasTrafficLight && checkTrafficLights)
                {
#if DEBUG
					if (debug) {
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Node {targetNodeId} has a traffic light.");
					}
#endif
                    RoadBaseAI.TrafficLightState vehicleLightState;
                    bool stopCar = false;
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckTrafficLight")) {
#endif

                    var destinationInfo = targetNode.Info;

                    uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
                    uint targetNodeLower8Bits = (uint)((targetNodeId << 8) / 32768);

                    RoadBaseAI.TrafficLightState pedestrianLightState;
                    bool vehicles;
                    bool pedestrians;
                    CustomRoadAI.GetTrafficLightState(
#if DEBUG
							frontVehicleId, ref vehicleData,
#endif
                            targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, ref prevSegment, currentFrameIndex - targetNodeLower8Bits, out vehicleLightState, out pedestrianLightState, out vehicles, out pedestrians);

                    if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Car && isRecklessDriver && !isLevelCrossing)
                    {
                        vehicleLightState = RoadBaseAI.TrafficLightState.Green;
                    }

#if DEBUG
					if (debug)
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle {frontVehicleId} has TL state {vehicleLightState} at node {targetNodeId}");
#endif

                    uint random = currentFrameIndex - targetNodeLower8Bits & 255u;
                    if (!vehicles && random >= 196u)
                    {
                        vehicles = true;
                        RoadBaseAI.SetTrafficLightState(targetNodeId, ref prevSegment, currentFrameIndex - targetNodeLower8Bits, vehicleLightState, pedestrianLightState, vehicles, pedestrians);
                    }

                    switch (vehicleLightState)
                    {
                        case RoadBaseAI.TrafficLightState.RedToGreen:
                            if (random < 60u)
                            {
                                if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                                {
                                    stopCar = true;
                                }
                            }
                            break;
                        case RoadBaseAI.TrafficLightState.Red:
                            if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                            {
                                stopCar = true;
                            }
                            break;
                        case RoadBaseAI.TrafficLightState.GreenToRed:
                            if (random >= 30u)
                            {
                                if (!ProcessLeftWaiting(frontVehicleId, vehicleData, targetNodeId, prevPos.m_segment, prevPos.m_lane, position.m_segment, laneID))
                                {
                                    stopCar = true;
                                }
                            }
                            break;
                    }
#if BENCHMARK
					//}
#endif

                    // Turn-on-red: Check if turning in the preferred direction, and if turning while it's red is allowed
                    if (
                        Options.turnOnRedEnabled &&
                        stopCar &&
                        sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxYieldVelocity * GlobalConfig.Instance.PriorityRules.MaxYieldVelocity &&
                        !isRecklessDriver
                    )
                    {
                        IJunctionRestrictionsManager junctionRestrictionsManager = Constants.ManagerFactory.JunctionRestrictionsManager;
                        ITurnOnRedManager turnOnRedMan = Constants.ManagerFactory.TurnOnRedManager;
                        bool lhd = Constants.ServiceFactory.SimulationService.LeftHandDrive;
                        int torIndex = turnOnRedMan.GetIndex(prevPos.m_segment, isTargetStartNode);
                        if (
                            (turnOnRedMan.TurnOnRedSegments[torIndex].leftSegmentId == position.m_segment &&
                            junctionRestrictionsManager.IsTurnOnRedAllowed(lhd, prevPos.m_segment, isTargetStartNode)) ||
                            (turnOnRedMan.TurnOnRedSegments[torIndex].rightSegmentId == position.m_segment &&
                            junctionRestrictionsManager.IsTurnOnRedAllowed(!lhd, prevPos.m_segment, isTargetStartNode))
                        )
                        {
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle may turn on red to target segment {position.m_segment}, lane {position.m_lane}");
#endif
                            stopCar = false;
                        }
                    }

                    // check priority rules at unprotected traffic lights
                    if (!stopCar && Options.prioritySignsEnabled && Options.trafficLightPriorityRules && segLightsMan.IsSegmentLight(prevPos.m_segment, isTargetStartNode))
                    {
                        bool hasPriority = true;
#if BENCHMARK
						//using (var bm = new Benchmark(null, "CheckPriorityRulesAtTTL")) {
#endif
                        hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if BENCHMARK
						//}
#endif

                        if (!hasPriority)
                        {
                            // green light but other cars are incoming and they have priority: stop
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Green traffic light (or turn on red allowed) but detected traffic with higher priority: stop.");
#endif
                            stopCar = true;
                        }
                    }

                    if (stopCar)
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP");
#endif

                        if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                        {
                            vehicleData.m_flags2 |= Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }

                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;
                        maxSpeed = 0f;
                        vehicleData.m_blockCounter = 0;
                        return false;
                    }
                    else
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE ({vehicleLightState})");
#endif
                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;

                        if (vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Tram || vehicleData.Info.m_vehicleType == VehicleInfo.VehicleType.Train)
                        {
                            vehicleData.m_flags2 &= ~Vehicle.Flags2.Yielding;
                            vehicleData.m_waitCounter = 0;
                        }
                    }
                }
                else if (Options.prioritySignsEnabled && vehicleData.Info.m_vehicleType != VehicleInfo.VehicleType.Monorail)
                {
#if BENCHMARK
					//using (var bm = new Benchmark(null, "CheckPriorityRules")) {
#endif

#if DEBUG
					//bool debug = destinationNodeId == 10864;
					//bool debug = destinationNodeId == 13531;
					//bool debug = false;// targetNodeId == 5027;
#endif
                    //bool debug = false;
#if DEBUG
					if (debug)
						Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle is arriving @ seg. {prevPos.m_segment} ({position.m_segment}, {nextPosition.m_segment}), node {targetNodeId} which is not a traffic light.");
#endif

                    var sign = prioMan.GetPrioritySign(prevPos.m_segment, isTargetStartNode);
                    if (sign != PrioritySegment.PriorityType.None)
                    {
#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle is arriving @ seg. {prevPos.m_segment} ({position.m_segment}, {nextPosition.m_segment}), node {targetNodeId} which is not a traffic light and is a priority segment.");
#endif

#if DEBUG
						if (debug)
							Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): JunctionTransitState={vehicleState.JunctionTransitState.ToString()}");
#endif

                        if (vehicleState.JunctionTransitState == VehicleJunctionTransitState.None)
                        {
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to APPROACH (prio)");
#endif
                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Approach;
                        }

                        if (vehicleState.JunctionTransitState != VehicleJunctionTransitState.Leave)
                        {
                            bool hasPriority;
                            switch (sign)
                            {
                                case PriorityType.Stop:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): STOP sign. waittime={vehicleState.waitTime}, sqrVelocity={sqrVelocity}");
#endif

                                    maxSpeed = 0f;

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait) waitTime={vehicleState.waitTime}");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxStopVelocity * GlobalConfig.Instance.PriorityRules.MaxStopVelocity)
                                        {
                                            vehicleState.waitTime++;

                                            //float minStopWaitTime = Singleton<SimulationManager>.instance.m_randomizer.UInt32(3);
                                            if (vehicleState.waitTime >= 2)
                                            {
                                                if (Options.simAccuracy >= 4)
                                                {
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }
                                                else
                                                {
                                                    hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): hasPriority={hasPriority}");
#endif

                                                    if (!hasPriority)
                                                    {
                                                        vehicleData.m_blockCounter = 0;
                                                        return false;
                                                    }
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (min wait timeout)");
#endif
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }

#if DEBUG
												if (debug)
													Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle must first come to a full stop in front of the stop sign.");
#endif
                                                return false;
                                            }
                                        }
                                        else
                                        {
#if DEBUG
											if (debug)
												Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle has come to a full stop.");
#endif
                                            vehicleState.waitTime = 0;
                                            return false;
                                        }
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Max. wait time exceeded. Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    break;
                                case PriorityType.Yield:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): YIELD sign. waittime={vehicleState.waitTime}");
#endif

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
                                        vehicleState.waitTime++;
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxYieldVelocity * GlobalConfig.Instance.PriorityRules.MaxYieldVelocity || Options.simAccuracy <= 2)
                                        {
                                            if (Options.simAccuracy >= 4)
                                            {
                                                vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                            }
                                            else
                                            {
                                                hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
												if (debug)
													Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): hasPriority: {hasPriority}");
#endif

                                                if (!hasPriority)
                                                {
                                                    vehicleData.m_blockCounter = 0;
                                                    maxSpeed = 0f;
                                                    return false;
                                                }
                                                else
                                                {
#if DEBUG
													if (debug)
														Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (no incoming cars)");
#endif
                                                    vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                                }
                                            }
                                        }
                                        else
                                        {
#if DEBUG
											if (debug)
												Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Vehicle has not yet reached yield speed (reduce {sqrVelocity} by {vehicleState.reduceSqrSpeedByValueToYield})");
#endif

                                            // vehicle has not yet reached yield speed
                                            maxSpeed = GlobalConfig.Instance.PriorityRules.MaxYieldVelocity;
                                            return false;
                                        }
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    break;
                                case PriorityType.Main:
                                default:
#if DEBUG
									if (debug)
										Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): MAIN sign. waittime={vehicleState.waitTime}");
#endif
                                    maxSpeed = 0f;

                                    if (Options.simAccuracy == 4)
                                        return true;

                                    if (vehicleState.waitTime < GlobalConfig.Instance.PriorityRules.MaxPriorityWaitTime)
                                    {
                                        vehicleState.waitTime++;
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to STOP (wait)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Stop;

                                        hasPriority = prioMan.HasPriority(frontVehicleId, ref vehicleData, ref prevPos, targetNodeId, isTargetStartNode, ref position, ref targetNode);
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): {hasPriority}");
#endif

                                        if (!hasPriority)
                                        {
                                            vehicleData.m_blockCounter = 0;
                                            return false;
                                        }
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState to LEAVE (no conflicting car)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    else
                                    {
#if DEBUG
										if (debug)
											Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Max. wait time exceeded. Setting JunctionTransitState to LEAVE (max wait timeout)");
#endif
                                        vehicleState.JunctionTransitState = VehicleJunctionTransitState.Leave;
                                    }
                                    return true;
                            }
                        }
                        else if (sqrVelocity <= GlobalConfig.Instance.PriorityRules.MaxStopVelocity * GlobalConfig.Instance.PriorityRules.MaxStopVelocity && (vehicleState.vehicleType & ExtVehicleType.RoadVehicle) != ExtVehicleType.None)
                        {
                            // vehicle is not moving. reset allowance to leave junction
#if DEBUG
							if (debug)
								Log._Debug($"VehicleBehaviorManager.MayChangeSegment({frontVehicleId}): Setting JunctionTransitState from LEAVE to BLOCKED (speed to low)");
#endif
                            vehicleState.JunctionTransitState = VehicleJunctionTransitState.Blocked;

                            maxSpeed = 0f;
                            return false;
                        }
                    }
#if BENCHMARK
					//}
#endif
                }
            }
            maxSpeed = 0f; // maxSpeed should be set by caller
            return true;
        }

        protected bool MustCheckSpace(ushort segmentId, bool startNode, ref NetNode node, bool isRecklessDriver)
        {
            bool result;
            if (isRecklessDriver)
            {
                result = ((node.m_flags & NetNode.Flags.LevelCrossing) > NetNode.Flags.None);
            }
            else if (Options.junctionRestrictionsEnabled)
            {
                result = !JunctionRestrictionsManager.Instance.IsEnteringBlockedJunctionAllowed(segmentId, startNode);
            }
            else
            {
                result = ((node.m_flags & (NetNode.Flags.Junction | NetNode.Flags.OneWayOut | NetNode.Flags.OneWayIn)) == NetNode.Flags.Junction && node.CountSegments() != 2);
            }
            return result;
        }

        public static void LeftTurnWaitingPre(ushort vehicleID, ref Vehicle vehicleData)
        {
            if (MainDataStore.isLeftWaiting[vehicleID])
            {
                uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
                uint targetNodeLower8Bits = (uint)((MainDataStore.nodeId[vehicleID] << 8) / 32768);
                uint random = currentFrameIndex - targetNodeLower8Bits & 255u;
                SegmentGeometry segmentGeometry = SegmentGeometry.Get(MainDataStore.fromSegmentId[vehicleID], false);
                if (segmentGeometry == null)
                {

                }
                else
                {
                    bool startNode = segmentGeometry.StartNodeId() == MainDataStore.nodeId[vehicleID];
                    ICustomSegmentLights segmentLights = CustomSegmentLightsManager.Instance.GetSegmentLights(MainDataStore.fromSegmentId[vehicleID], startNode, false, RoadBaseAI.TrafficLightState.Red);
                    ICustomSegmentLight customSegmentLight = (segmentLights == null) ? null : segmentLights.GetCustomLight(MainDataStore.fromLaneIndex[vehicleID]);
                    if (customSegmentLight == null)
                    {
                        MainDataStore.isLeftWaiting[vehicleID] = false;
                        MainDataStore.crossStopLine[vehicleID] = false;
                        MainDataStore.nodeId[vehicleID] = 0;
                        MainDataStore.fromSegmentId[vehicleID] = 0;
                        MainDataStore.fromLaneIndex[vehicleID] = 0;
                        MainDataStore.toSegmentId[vehicleID] = 0;
                        MainDataStore.laneID[vehicleID] = 0;
                        MainDataStore.offset[vehicleID] = 0;
                        MainDataStore.nextLaneId[vehicleID] = 0;
                        MainDataStore.nextSegOffset[vehicleID] = 0;
                        MainDataStore.additionWaitingTime[vehicleID] = 0;
                    }
                    else if ((customSegmentLight.LightLeft == RoadBaseAI.TrafficLightState.Red && customSegmentLight.LightMain == RoadBaseAI.TrafficLightState.Green) || (customSegmentLight.LightLeft == RoadBaseAI.TrafficLightState.GreenToRed) || (customSegmentLight.LightLeft == RoadBaseAI.TrafficLightState.RedToGreen))
                    {
                    }
                    else
                    {
                        if (MainDataStore.additionWaitingTime[vehicleID] > 0)
                        {
                            MainDataStore.additionWaitingTime[vehicleID]--;
                        }
                        else
                        {
                            MainDataStore.isLeftWaiting[vehicleID] = false;
                            MainDataStore.crossStopLine[vehicleID] = false;
                            MainDataStore.nodeId[vehicleID] = 0;
                            MainDataStore.fromSegmentId[vehicleID] = 0;
                            MainDataStore.fromLaneIndex[vehicleID] = 0;
                            MainDataStore.toSegmentId[vehicleID] = 0;
                            MainDataStore.laneID[vehicleID] = 0;
                            MainDataStore.offset[vehicleID] = 0;
                            MainDataStore.nextLaneId[vehicleID] = 0;
                            MainDataStore.nextSegOffset[vehicleID] = 0;
                        }
                    }
                }
            }
            else
            {
                MainDataStore.isLeftWaiting[vehicleID] = false;
                MainDataStore.crossStopLine[vehicleID] = false;
                MainDataStore.nodeId[vehicleID] = 0;
                MainDataStore.fromSegmentId[vehicleID] = 0;
                MainDataStore.fromLaneIndex[vehicleID] = 0;
                MainDataStore.toSegmentId[vehicleID] = 0;
                MainDataStore.laneID[vehicleID] = 0;
                MainDataStore.offset[vehicleID] = 0;
                MainDataStore.nextLaneId[vehicleID] = 0;
                MainDataStore.nextSegOffset[vehicleID] = 0;
            }
        }

        public static void LeftTurnWaitingPost(ushort vehicleID, ref Vehicle vehicleData)
        {
            if (MainDataStore.isLeftWaiting[vehicleID])
            {
                uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
                uint targetNodeLower8Bits = (uint)((MainDataStore.nodeId[vehicleID] << 8) / 32768);
                uint random = currentFrameIndex - targetNodeLower8Bits & 255u;
                SegmentGeometry segmentGeometry = SegmentGeometry.Get(MainDataStore.fromSegmentId[vehicleID], false);
                if (segmentGeometry == null)
                {

                }
                else
                {
                    NetManager instance = Singleton<NetManager>.instance;
                    Vector4 targetPos0 = vehicleData.m_targetPos0;
                    Vector4 targetPos1 = vehicleData.m_targetPos1;
                    Vector4 targetPos2 = vehicleData.m_targetPos2;
                    Vector4 targetPos3 = vehicleData.m_targetPos3;
                    Vehicle.Frame lastFrameData = vehicleData.GetLastFrameData();
                    Vector3 lastFrameVehiclePos = lastFrameData.m_position;
                    Vector3 startPos = instance.m_lanes.m_buffer[MainDataStore.laneID[vehicleID]].CalculatePosition((MainDataStore.offset[vehicleID]) * 0.003921569f);
                    Vector3 endPos = instance.m_lanes.m_buffer[MainDataStore.nextLaneId[vehicleID]].CalculatePosition((MainDataStore.nextSegOffset[vehicleID]) * 0.003921569f);
                    Vector3 tmp1 = Vector3.zero;
                    Vector3 tmp = Vector3.zero;
                    Vector3 startDir = Vector3.zero;
                    Vector3 endDir = Vector3.zero;


                    if (MainDataStore.offset[vehicleID] == 0)
                    {
                        startDir = startPos - instance.m_lanes.m_buffer[MainDataStore.laneID[vehicleID]].CalculatePosition((MainDataStore.offset[vehicleID] + 64) * 0.003921569f);
                    }
                    else if (MainDataStore.offset[vehicleID] == 255)
                    {
                        startDir = startPos - instance.m_lanes.m_buffer[MainDataStore.laneID[vehicleID]].CalculatePosition((MainDataStore.offset[vehicleID] - 64) * 0.003921569f);
                    }

                    startDir = VectorUtils.NormalizeXZ(startDir);

                    if (MainDataStore.nextSegOffset[vehicleID] == 0)
                    {
                        endDir = endPos - instance.m_lanes.m_buffer[MainDataStore.nextLaneId[vehicleID]].CalculatePosition((MainDataStore.nextSegOffset[vehicleID] + 64) * 0.003921569f);
                    }
                    else if (MainDataStore.nextSegOffset[vehicleID] == 255)
                    {
                        endDir = endPos - instance.m_lanes.m_buffer[MainDataStore.nextLaneId[vehicleID]].CalculatePosition((MainDataStore.nextSegOffset[vehicleID] - 64) * 0.003921569f);
                    }

                    endDir = VectorUtils.NormalizeXZ(endDir);

                    NetSegment.CalculateMiddlePoints(startPos, startDir, endPos, endDir, true, true, out tmp1, out tmp);

                    Vector3 leftStopPosition = (tmp1 + tmp + startPos) / 3f;


                    targetPos0.w = 2f;
                    targetPos0.Set(targetPos0.x, targetPos0.y, targetPos0.z, targetPos0.w);
                    targetPos1.w = 2f;
                    targetPos1.Set(targetPos1.x, targetPos1.y, targetPos1.z, targetPos1.w);
                    targetPos2.w = 2f;
                    targetPos2.Set(targetPos2.x, targetPos2.y, targetPos2.z, targetPos2.w);
                    targetPos3.w = 2f;
                    targetPos3.Set(targetPos3.x, targetPos3.y, targetPos3.z, targetPos3.w);
                    vehicleData.SetTargetPos(0, targetPos0);
                    vehicleData.SetTargetPos(1, targetPos1);
                    vehicleData.SetTargetPos(2, targetPos2);
                    vehicleData.SetTargetPos(3, targetPos3);

                    //DebugLog.LogToFileOnly("vehicleID = " + vehicleID.ToString());
                    //DebugLog.LogToFileOnly("leftStopPosition = " + leftStopPosition.ToString());
                    //DebugLog.LogToFileOnly("lastFrameVehiclePos = " + lastFrameVehiclePos.ToString());
                    //DebugLog.LogToFileOnly("stopPos = " + startPos.ToString());
                    //DebugLog.LogToFileOnly("endPos = " + endPos.ToString());

                    float distDiff = 0.5f + (vehicleData.Info.m_generatedInfo.m_size.z / 2) - Vector3.Distance(leftStopPosition, lastFrameVehiclePos);
                    if (distDiff > 0)
                    {
                        /*DebugLog.LogToFileOnly("+++++++++++++++++++++++++++++++++++");
                        DebugLog.LogToFileOnly("vehicleID = " + vehicleData.Info.name.ToString());
                        DebugLog.LogToFileOnly("Flag = " + vehicleData.m_flags.ToString());
                        DebugLog.LogToFileOnly("targetPos0 = " + targetPos0.ToString());
                        DebugLog.LogToFileOnly("targetPos1 = " + targetPos1.ToString());
                        DebugLog.LogToFileOnly("targetPos2 = " + targetPos2.ToString());
                        DebugLog.LogToFileOnly("targetPos3 = " + targetPos3.ToString());
                        DebugLog.LogToFileOnly("leftStopPosition = " + leftStopPosition.ToString());
                        DebugLog.LogToFileOnly("lastFrameVehiclePos = " + lastFrameVehiclePos.ToString());
                        DebugLog.LogToFileOnly("startPos = " + startPos.ToString());
                        DebugLog.LogToFileOnly("endPos = " + endPos.ToString());
                        DebugLog.LogToFileOnly("+++++++++++++++++++++++++++++++++++");*/
                        if (!MainDataStore.crossStopLine[vehicleID])
                        {
                            NetManager instance1 = Singleton<NetManager>.instance;
                            Vector3 tmpPosition1 = instance.m_lanes.m_buffer[MainDataStore.laneID[vehicleID]].CalculatePosition((MainDataStore.offset[vehicleID]) * 0.003921569f);
                            Vector3 tmpPosition2 = instance.m_nodes.m_buffer[MainDataStore.nodeId[vehicleID]].m_position;
                            float distance = Vector3.Distance(tmpPosition1, tmpPosition2);
                            MainDataStore.additionWaitingTime[vehicleID] = (byte)(distance/4f);
                            MainDataStore.crossStopLine[vehicleID] = true;
                        }
                        targetPos0.w = 0f;
                        targetPos0.Set(targetPos0.x, targetPos0.y, targetPos0.z, targetPos0.w);
                        targetPos1.w = 0f;
                        targetPos1.Set(targetPos1.x, targetPos1.y, targetPos1.z, targetPos1.w);
                        targetPos2.w = 0f;
                        targetPos2.Set(targetPos2.x, targetPos2.y, targetPos2.z, targetPos2.w);
                        targetPos3.w = 0f;
                        targetPos3.Set(targetPos3.x, targetPos3.y, targetPos3.z, targetPos3.w);
                        vehicleData.SetTargetPos(0, targetPos0);
                        vehicleData.SetTargetPos(1, targetPos1);
                        vehicleData.SetTargetPos(2, targetPos2);
                        vehicleData.SetTargetPos(3, targetPos3);
                    }
                }
            }
            else
            {
                MainDataStore.additionWaitingTime[vehicleID] = 0;
                MainDataStore.isLeftWaiting[vehicleID] = false;
                MainDataStore.crossStopLine[vehicleID] = false;
                MainDataStore.nodeId[vehicleID] = 0;
                MainDataStore.fromSegmentId[vehicleID] = 0;
                MainDataStore.fromLaneIndex[vehicleID] = 0;
                MainDataStore.toSegmentId[vehicleID] = 0;
                MainDataStore.laneID[vehicleID] = 0;
                MainDataStore.offset[vehicleID] = 0;
                MainDataStore.nextLaneId[vehicleID] = 0;
                MainDataStore.nextSegOffset[vehicleID] = 0;

            }
        }

        private void SimulationStepBlown(ushort vehicleID, ref Vehicle vehicleData, ref Vehicle.Frame frameData, ushort leaderID, ref Vehicle leaderData, int lodPhysics)
        {
            uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
            frameData.m_position += frameData.m_velocity * 0.5f;
            frameData.m_velocity.y = frameData.m_velocity.y - 2.4525f;
            frameData.m_velocity *= 0.99f;
            frameData.m_position += frameData.m_velocity * 0.5f;
            float num = Singleton<TerrainManager>.instance.SampleDetailHeight(frameData.m_position);
            if (num > frameData.m_position.y)
            {
                frameData.m_velocity = Vector3.zero;
                frameData.m_position.y = num;
                vehicleData.m_blockCounter = (byte)Mathf.Min((int)(vehicleData.m_blockCounter + 1), 255);
            }
            else
            {
                vehicleData.m_blockCounter = 0;
                frameData.m_travelDistance += 0.1f;
                Randomizer randomizer = new Randomizer((int)vehicleID);
                float num2 = (float)randomizer.Int32(100, 1000) * 6.283185E-05f * currentFrameIndex;
                Vector3 axis;
                axis.x = Mathf.Sin((float)randomizer.Int32(1000u) * 0.00628318544f + num2);
                axis.y = Mathf.Sin((float)randomizer.Int32(1000u) * 0.00628318544f + num2);
                axis.z = Mathf.Sin((float)randomizer.Int32(1000u) * 0.00628318544f + num2);
                if (axis.sqrMagnitude > 0.001f)
                {
                    Quaternion b = Quaternion.AngleAxis((float)randomizer.Int32(360u), axis);
                    frameData.m_rotation = Quaternion.Lerp(frameData.m_rotation, b, 0.2f);
                }
            }
            bool flag = (currentFrameIndex + (uint)leaderID & 16u) != 0u;
            leaderData.m_flags &= ~(Vehicle.Flags.OnGravel | Vehicle.Flags.Underground | Vehicle.Flags.Transition | Vehicle.Flags.InsideBuilding);
            frameData.m_swayVelocity = Vector3.zero;
            frameData.m_swayPosition = Vector3.zero;
            frameData.m_steerAngle = 0f;
            frameData.m_lightIntensity.x = 5f;
            frameData.m_lightIntensity.y = 5f;
            frameData.m_lightIntensity.z = ((!flag) ? 0f : 5f);
            frameData.m_lightIntensity.w = ((!flag) ? 0f : 5f);
            frameData.m_underground = false;
            frameData.m_transition = false;
        }

        private void SimulationStepFloating(ushort vehicleID, ref Vehicle vehicleData, ref Vehicle.Frame frameData, ushort leaderID, ref Vehicle leaderData, int lodPhysics)
        {
            uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
            Vector3 vector = frameData.m_rotation * Vector3.forward;
            frameData.m_position += frameData.m_velocity * 0.5f;
            frameData.m_swayPosition += frameData.m_swayVelocity * 0.5f;
            float num = -Mathf.Atan2(vector.x, vector.z);
            num += frameData.m_angleVelocity * 0.5f;
            frameData.m_rotation = Quaternion.AngleAxis(num * 57.29578f, Vector3.down);
            vehicleData.m_blockCounter = (byte)Mathf.Min((int)(vehicleData.m_blockCounter + 1), 255);
            float num2;
            float num3;
            Vector3 vector2;
            Vector3 a;
            Singleton<TerrainManager>.instance.SampleWaterData(VectorUtils.XZ(frameData.m_position), out num2, out num3, out vector2, out a);
            if (num3 - num2 > 1f)
            {
                frameData.m_angleVelocity = frameData.m_angleVelocity * 0.9f + (float)Singleton<SimulationManager>.instance.m_randomizer.Int32(-1000, 1000) * 0.0001f;
                vector2 *= 0.266666681f;
                frameData.m_velocity = Vector3.MoveTowards(frameData.m_velocity, vector2, 1f);
                float num4 = Mathf.Clamp((float)vehicleData.m_blockCounter * 0.05f, 1f, num3 - num2);
                frameData.m_velocity.y = frameData.m_velocity.y * 0.5f + (num3 - num4 - frameData.m_position.y) * 0.5f;
            }
            else
            {
                num2 = Singleton<TerrainManager>.instance.SampleDetailHeight(frameData.m_position);
                frameData.m_angleVelocity = 0f;
                frameData.m_velocity = Vector3.MoveTowards(frameData.m_velocity, Vector3.zero, 1f);
                frameData.m_velocity.y = frameData.m_velocity.y * 0.5f + (num2 - frameData.m_position.y) * 0.5f;
            }
            a.y = 0f;
            bool flag = (currentFrameIndex + (uint)leaderID & 16u) != 0u;
            leaderData.m_flags &= ~(Vehicle.Flags.OnGravel | Vehicle.Flags.Underground | Vehicle.Flags.Transition | Vehicle.Flags.InsideBuilding);
            frameData.m_position += frameData.m_velocity * 0.5f;
            num += frameData.m_angleVelocity * 0.5f;
            frameData.m_rotation = Quaternion.AngleAxis(num * 57.29578f, Vector3.down);
            frameData.m_swayVelocity += a * 0.2f - frameData.m_swayVelocity * 0.5f - frameData.m_swayPosition * 0.5f;
            frameData.m_swayPosition += frameData.m_swayVelocity * 0.5f;
            frameData.m_steerAngle = 0f;
            //frameData.m_travelDistance = frameData.m_travelDistance;
            frameData.m_lightIntensity.x = 5f;
            frameData.m_lightIntensity.y = 5f;
            frameData.m_lightIntensity.z = ((!flag) ? 0f : 5f);
            frameData.m_lightIntensity.w = ((!flag) ? 0f : 5f);
            frameData.m_underground = false;
            frameData.m_transition = false;
        }

        public static void VehicleStatusForRealGasStation(ushort vehicleID, ref Vehicle vehicleData)
        {
            DebugLog.LogToFileOnly("Error: should be detoured by RealGasStation");
        }

        public static void VehicleStatusForRealCity(ushort vehicleID, ref Vehicle vehicleData)
        {
            DebugLog.LogToFileOnly("Error: should be detoured by RealCity");
        }

        public static void VehicleStatusForTrafficCongestionReport(ushort vehicleID, ref Vehicle vehicleData)
        {
            DebugLog.LogToFileOnly("Error: should be detoured by TrafficCongestionReport");
        }

        public void CustomSimulationStep(ushort vehicleID, ref Vehicle vehicleData, ref Vehicle.Frame frameData, ushort leaderID, ref Vehicle leaderData, int lodPhysics)
        {
            if (Loader.isRealGasStationRunning)
            {
                VehicleStatusForRealGasStation(vehicleID, ref vehicleData);
            }

            if (Loader.isRealCityRunning)
            {
                VehicleStatusForRealCity(vehicleID, ref vehicleData);
            }

            if (Loader.isTrafficCongestionReportRunning)
            {
                VehicleStatusForTrafficCongestionReport(vehicleID, ref vehicleData);
            }

            if ((leaderData.m_flags2 & Vehicle.Flags2.Blown) != (Vehicle.Flags2)0)
            {
                this.SimulationStepBlown(vehicleID, ref vehicleData, ref frameData, leaderID, ref leaderData, lodPhysics);
                return;
            }
            if ((leaderData.m_flags2 & Vehicle.Flags2.Floating) != (Vehicle.Flags2)0)
            {
                this.SimulationStepFloating(vehicleID, ref vehicleData, ref frameData, leaderID, ref leaderData, lodPhysics);
                return;
            }
            uint currentFrameIndex = Singleton<SimulationManager>.instance.m_currentFrameIndex;
            frameData.m_position += frameData.m_velocity * 0.5f;
            frameData.m_swayPosition += frameData.m_swayVelocity * 0.5f;
            float num = this.m_info.m_acceleration;
            float num2 = this.m_info.m_braking;
            if ((vehicleData.m_flags & Vehicle.Flags.Emergency2) != (Vehicle.Flags)0)
            {
                num *= 2f;
                num2 *= 2f;
            }
            float magnitude = frameData.m_velocity.magnitude;
            Vector3 vector = (Vector3)vehicleData.m_targetPos0 - frameData.m_position;
            float sqrMagnitude = vector.sqrMagnitude;
            float num3 = (magnitude + num) * (0.5f + 0.5f * (magnitude + num) / num2) + this.m_info.m_generatedInfo.m_size.z * 0.5f;
            float num4 = Mathf.Max(magnitude + num, 5f);
            if (lodPhysics >= 2 && (ulong)(currentFrameIndex >> 4 & 3u) == (ulong)((long)(vehicleID & 3)))
            {
                num4 *= 2f;
            }
            float num5 = Mathf.Max((num3 - num4) / 3f, 1f);
            float num6 = num4 * num4;
            float num7 = num5 * num5;
            int i = 0;
            bool flag = false;
            // NON-STOCK CODE START
            NewCarAI.LeftTurnWaitingPre(vehicleID, ref vehicleData);
            /// NON-STOCK CODE END
            if (!MainDataStore.crossStopLine[vehicleID] && (sqrMagnitude < num6 || vehicleData.m_targetPos3.w < 0.01f) && (leaderData.m_flags & (Vehicle.Flags.WaitingPath | Vehicle.Flags.Stopped)) == (Vehicle.Flags)0)
            {
                if (leaderData.m_path != 0u)
                {
                    base.UpdatePathTargetPositions(vehicleID, ref vehicleData, frameData.m_position, ref i, 4, num6, num7);
                    // NON-STOCK CODE START
                    NewCarAI.LeftTurnWaitingPost(vehicleID, ref vehicleData);
                    /// NON-STOCK CODE END

                    if ((leaderData.m_flags & Vehicle.Flags.Spawned) == (Vehicle.Flags)0)
                    {
                        frameData = vehicleData.m_frame0;
                        return;
                    }
                }
                if ((leaderData.m_flags & Vehicle.Flags.WaitingPath) == (Vehicle.Flags)0)
                {
                    while (i < 4)
                    {
                        float minSqrDistance;
                        Vector3 refPos;
                        if (i == 0)
                        {
                            minSqrDistance = num6;
                            refPos = frameData.m_position;
                            flag = true;
                        }
                        else
                        {
                            minSqrDistance = num7;
                            refPos = vehicleData.GetTargetPos(i - 1);
                        }
                        int num8 = i;
                        this.UpdateBuildingTargetPositions(vehicleID, ref vehicleData, refPos, leaderID, ref leaderData, ref i, minSqrDistance);
                        if (i == num8)
                        {
                            break;
                        }
                    }
                    if (i != 0)
                    {
                        Vector4 targetPos = vehicleData.GetTargetPos(i - 1);
                        while (i < 4)
                        {
                            vehicleData.SetTargetPos(i++, targetPos);
                        }
                    }
                }
                vector = (Vector3)vehicleData.m_targetPos0 - frameData.m_position;
                sqrMagnitude = vector.sqrMagnitude;
            }
            if (leaderData.m_path != 0u && (leaderData.m_flags & Vehicle.Flags.WaitingPath) == (Vehicle.Flags)0)
            {
                NetManager instance = Singleton<NetManager>.instance;
                byte b = leaderData.m_pathPositionIndex;
                byte lastPathOffset = leaderData.m_lastPathOffset;
                if (b == 255)
                {
                    b = 0;
                }
                int noise;
                float num9 = 1f + leaderData.CalculateTotalLength(leaderID, out noise);
                PathManager instance2 = Singleton<PathManager>.instance;
                PathUnit.Position pathPos;
                if (instance2.m_pathUnits.m_buffer[(int)((UIntPtr)leaderData.m_path)].GetPosition(b >> 1, out pathPos))
                {
                    if ((instance.m_segments.m_buffer[(int)pathPos.m_segment].m_flags & NetSegment.Flags.Flooded) != NetSegment.Flags.None && Singleton<TerrainManager>.instance.HasWater(VectorUtils.XZ(frameData.m_position)))
                    {
                        leaderData.m_flags2 |= Vehicle.Flags2.Floating;
                    }
                    instance.m_segments.m_buffer[(int)pathPos.m_segment].AddTraffic(Mathf.RoundToInt(num9 * 2.5f), noise);
                    bool flag2 = false;
                    if ((b & 1) == 0 || lastPathOffset == 0)
                    {
                        uint laneID = PathManager.GetLaneID(pathPos);
                        if (laneID != 0u)
                        {
                            Vector3 b2 = instance.m_lanes.m_buffer[(int)((UIntPtr)laneID)].CalculatePosition((float)pathPos.m_offset * 0.003921569f);
                            float num10 = 0.5f * magnitude * magnitude / num2 + this.m_info.m_generatedInfo.m_size.z * 0.5f;
                            if (Vector3.Distance(frameData.m_position, b2) >= num10 - 1f)
                            {
                                instance.m_lanes.m_buffer[(int)((UIntPtr)laneID)].ReserveSpace(num9);
                                flag2 = true;
                            }
                        }
                    }
                    if (!flag2 && instance2.m_pathUnits.m_buffer[(int)((UIntPtr)leaderData.m_path)].GetNextPosition(b >> 1, out pathPos))
                    {
                        uint laneID2 = PathManager.GetLaneID(pathPos);
                        if (laneID2 != 0u)
                        {
                            instance.m_lanes.m_buffer[(int)((UIntPtr)laneID2)].ReserveSpace(num9);
                        }
                    }
                }
                if ((ulong)(currentFrameIndex >> 4 & 15u) == (ulong)((long)(leaderID & 15)))
                {
                    bool flag3 = false;
                    uint path = leaderData.m_path;
                    int num11 = b >> 1;
                    int j = 0;
                    while (j < 5)
                    {
                        bool flag4;
                        if (PathUnit.GetNextPosition(ref path, ref num11, out pathPos, out flag4))
                        {
                            uint laneID3 = PathManager.GetLaneID(pathPos);
                            if (laneID3 != 0u && !instance.m_lanes.m_buffer[(int)((UIntPtr)laneID3)].CheckSpace(num9))
                            {
                                j++;
                                continue;
                            }
                        }
                        if (flag4)
                        {
                            this.InvalidPath(vehicleID, ref vehicleData, leaderID, ref leaderData);
                        }
                        flag3 = true;
                        break;
                    }
                    if (!flag3)
                    {
                        leaderData.m_flags |= Vehicle.Flags.Congestion;
                    }
                }
            }
            float num12;
            if ((leaderData.m_flags & Vehicle.Flags.Stopped) != (Vehicle.Flags)0)
            {
                num12 = 0f;
            }
            else
            {
                num12 = vehicleData.m_targetPos0.w;
                if ((leaderData.m_flags & Vehicle.Flags.DummyTraffic) == (Vehicle.Flags)0)
                {
                    VehicleManager instance3 = Singleton<VehicleManager>.instance;
                    float f = magnitude * 100f / Mathf.Max(1f, vehicleData.m_targetPos0.w);
                    instance3.m_totalTrafficFlow += (uint)Mathf.RoundToInt(f);
                    instance3.m_maxTrafficFlow += 100u;
                }
            }
            Quaternion rotation = Quaternion.Inverse(frameData.m_rotation);
            vector = rotation * vector;
            Vector3 vector2 = rotation * frameData.m_velocity;
            Vector3 a = Vector3.forward;
            Vector3 vector3 = Vector3.zero;
            Vector3 zero = Vector3.zero;
            float num13 = 0f;
            float num14 = 0f;
            bool flag5 = false;
            float num15 = 0f;
            if (sqrMagnitude > 1f)
            {
                a = VectorUtils.NormalizeXZ(vector, out num15);
                if (num15 > 1f)
                {
                    Vector3 vector4 = vector;
                    num4 = Mathf.Max(magnitude, 2f);
                    num6 = num4 * num4;
                    if (sqrMagnitude > num6)
                    {
                        vector4 *= num4 / Mathf.Sqrt(sqrMagnitude);
                    }
                    bool flag6 = false;
                    if (vector4.z < Mathf.Abs(vector4.x))
                    {
                        if (vector4.z < 0f)
                        {
                            flag6 = true;
                        }
                        float num16 = Mathf.Abs(vector4.x);
                        if (num16 < 1f)
                        {
                            vector4.x = Mathf.Sign(vector4.x);
                            if (vector4.x == 0f)
                            {
                                vector4.x = 1f;
                            }
                            num16 = 1f;
                        }
                        vector4.z = num16;
                    }
                    float b3;
                    a = VectorUtils.NormalizeXZ(vector4, out b3);
                    num15 = Mathf.Min(num15, b3);
                    float num17 = 1.57079637f * (1f - a.z);
                    if (num15 > 1f)
                    {
                        num17 /= num15;
                    }
                    float num18 = num15;
                    if (vehicleData.m_targetPos0.w < 0.1f)
                    {
                        num12 = this.CalculateTargetSpeed(vehicleID, ref vehicleData, 1000f, num17);
                        num12 = Mathf.Min(num12, CalculateMaxSpeed(num18, Mathf.Min(vehicleData.m_targetPos0.w, vehicleData.m_targetPos1.w), num2 * 0.9f));
                    }
                    else
                    {
                        num12 = Mathf.Min(num12, this.CalculateTargetSpeed(vehicleID, ref vehicleData, 1000f, num17));
                        num12 = Mathf.Min(num12, CalculateMaxSpeed(num18, vehicleData.m_targetPos1.w, num2 * 0.9f));
                    }
                    num18 += VectorUtils.LengthXZ(vehicleData.m_targetPos1 - vehicleData.m_targetPos0);
                    num12 = Mathf.Min(num12, CalculateMaxSpeed(num18, vehicleData.m_targetPos2.w, num2 * 0.9f));
                    num18 += VectorUtils.LengthXZ(vehicleData.m_targetPos2 - vehicleData.m_targetPos1);
                    num12 = Mathf.Min(num12, CalculateMaxSpeed(num18, vehicleData.m_targetPos3.w, num2 * 0.9f));
                    num18 += VectorUtils.LengthXZ(vehicleData.m_targetPos3 - vehicleData.m_targetPos2);
                    if (vehicleData.m_targetPos3.w < 0.01f)
                    {
                        num18 = Mathf.Max(0f, num18 - this.m_info.m_generatedInfo.m_size.z * 0.5f);
                    }
                    num12 = Mathf.Min(num12, CalculateMaxSpeed(num18, 0f, num2 * 0.9f));
                    if (!DisableCollisionCheck(vehicleID, ref vehicleData))
                    {
                        CarAI.CheckOtherVehicles(vehicleID, ref vehicleData, ref frameData, ref num12, ref flag5, ref zero, num3, num2 * 0.9f, lodPhysics);
                        // NON-STOCK CODE START
                        // Stop here
                        if (MainDataStore.crossStopLine[vehicleID])
                        {
                            flag5 = true;
                            num12 = 0;
                            zero = Vector3.zero;
                        }
                        //Do not push
                        if (MainDataStore.isLeftWaiting[vehicleID])
                        {
                            if (flag5)
                            {
                                zero = Vector3.zero;
                                num12 = 0;
                            }
                        }
                        /// NON-STOCK CODE END
                    }
                    if (flag6)
                    {
                        num12 = -num12;
                    }
                    if (num12 < magnitude)
                    {
                        float num19 = Mathf.Max(num, Mathf.Min(num2, magnitude));
                        num13 = Mathf.Max(num12, magnitude - num19);
                    }
                    else
                    {
                        float num20 = Mathf.Max(num, Mathf.Min(num2, -magnitude));
                        num13 = Mathf.Min(num12, magnitude + num20);
                    }
                }
            }
            else if (magnitude < 0.1f && flag && this.ArriveAtDestination(leaderID, ref leaderData))
            {
                leaderData.Unspawn(leaderID);
                if (leaderID == vehicleID)
                {
                    frameData = leaderData.m_frame0;
                }
                return;
            }
            if ((leaderData.m_flags & Vehicle.Flags.Stopped) == (Vehicle.Flags)0 && num12 < 0.1f)
            {
                flag5 = true;
            }
            if (flag5)
            {
                vehicleData.m_blockCounter = (byte)Mathf.Min((int)(vehicleData.m_blockCounter + 1), 255);
            }
            else
            {
                vehicleData.m_blockCounter = 0;
            }
            if (num15 > 1f)
            {
                num14 = Mathf.Asin(a.x) * Mathf.Sign(num13);
                vector3 = a * num13;
            }
            else
            {
                num13 = 0f;
                Vector3 b4 = Vector3.ClampMagnitude(vector * 0.5f - vector2, num2);
                vector3 = vector2 + b4;
            }
            bool flag7 = (currentFrameIndex + (uint)leaderID & 16u) != 0u;
            Vector3 a2 = vector3 - vector2;
            Vector3 vector5 = frameData.m_rotation * vector3;
            frameData.m_velocity = vector5 + zero;
            frameData.m_position += frameData.m_velocity * 0.5f;
            frameData.m_swayVelocity = frameData.m_swayVelocity * (1f - this.m_info.m_dampers) - a2 * (1f - this.m_info.m_springs) - frameData.m_swayPosition * this.m_info.m_springs;
            frameData.m_swayPosition += frameData.m_swayVelocity * 0.5f;
            frameData.m_steerAngle = num14;
            frameData.m_travelDistance += vector3.z;
            frameData.m_lightIntensity.x = 5f;
            frameData.m_lightIntensity.y = ((a2.z >= -0.1f) ? 0.5f : 5f);
            frameData.m_lightIntensity.z = ((num14 >= -0.1f || !flag7) ? 0f : 5f);
            frameData.m_lightIntensity.w = ((num14 <= 0.1f || !flag7) ? 0f : 5f);
            frameData.m_underground = ((vehicleData.m_flags & Vehicle.Flags.Underground) != (Vehicle.Flags)0);
            frameData.m_transition = ((vehicleData.m_flags & Vehicle.Flags.Transition) != (Vehicle.Flags)0);
            if ((vehicleData.m_flags & Vehicle.Flags.Parking) != (Vehicle.Flags)0 && num15 <= 1f && flag)
            {
                Vector3 forward = vehicleData.m_targetPos1 - vehicleData.m_targetPos0;
                if (forward.sqrMagnitude > 0.01f)
                {
                    frameData.m_rotation = Quaternion.LookRotation(forward);
                }
            }
            else if (num13 > 0.1f)
            {
                if (vector5.sqrMagnitude > 0.01f)
                {
                    frameData.m_rotation = Quaternion.LookRotation(vector5);
                }
            }
            else if (num13 < -0.1f && vector5.sqrMagnitude > 0.01f)
            {
                frameData.m_rotation = Quaternion.LookRotation(-vector5);
            }
            base.SimulationStep(vehicleID, ref vehicleData, ref frameData, leaderID, ref leaderData, lodPhysics);
        }

        private static bool DisableCollisionCheck(ushort vehicleID, ref Vehicle vehicleData)
        {          
            if ((vehicleData.m_flags & Vehicle.Flags.Arriving) != (Vehicle.Flags)0)
            {
                float num = Mathf.Max(Mathf.Abs(vehicleData.m_targetPos3.x), Mathf.Abs(vehicleData.m_targetPos3.z));
                float num2 = 8640f;
                if (num > num2 - 100f)
                {
                    return true;
                }
            }
            return false;
        }

        private static float CalculateMaxSpeed(float targetDistance, float targetSpeed, float maxBraking)
        {
            float num = 0.5f * maxBraking;
            float num2 = num + targetSpeed;
            return Mathf.Sqrt(Mathf.Max(0f, num2 * num2 + 2f * targetDistance * maxBraking)) - num;
        }

    }

}
