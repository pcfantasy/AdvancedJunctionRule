using ColossalFramework;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;

namespace AdvancedJunctionRule.CustomAI
{
    public class NewLaneConnectorTool
    {
        private bool CheckSegmentsTurningAngle(ushort sourceSegmentId, ref NetSegment sourceSegment, bool sourceStartNode, ushort targetSegmentId, ref NetSegment targetSegment, bool targetStartNode)
        {
            NetManager expr_05 = Singleton<NetManager>.instance;
            NetInfo info = expr_05.m_segments.m_buffer[(int)sourceSegmentId].Info;
            NetInfo info2 = expr_05.m_segments.m_buffer[(int)targetSegmentId].Info;
            float num = 0.01f - Mathf.Min(info.m_maxTurnAngleCos, info2.m_maxTurnAngleCos);
            if (info.m_laneTypes == NetInfo.LaneType.Vehicle && !info.m_vehicleTypes.IsFlagSet(VehicleInfo.VehicleType.Car))
            {
                if (num < 1f)
                {
                    Vector3 vector;
                    if (sourceStartNode)
                    {
                        vector = sourceSegment.m_startDirection;
                    }
                    else
                    {
                        vector = sourceSegment.m_endDirection;
                    }
                    Vector3 vector2;
                    if (targetStartNode)
                    {
                        vector2 = targetSegment.m_startDirection;
                    }
                    else
                    {
                        vector2 = targetSegment.m_endDirection;
                    }
                    return vector.x * vector2.x + vector.z * vector2.z < num;
                }
            }
            return true;
        }

    }
}
