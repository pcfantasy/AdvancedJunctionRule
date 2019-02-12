using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AdvancedJunctionRule
{
    public class MainDataStore
    {
        public static ushort[] nodeId = new ushort[16384];             //2
        public static ushort[] fromSegmentId = new ushort[16384];      //2
        public static ushort[] toSegmentId = new ushort[16384];        //2
        public static byte[] fromLaneIndex = new byte[16384];          //1
        public static byte[] offset = new byte[16384];                 //1
        public static byte[] additionWaitingTime = new byte[16384];    //1
        public static byte[] nextSegOffset = new byte[16384];          //1
        public static uint[] laneID = new uint[16384];                 //4
        public static uint[] nextLaneId = new uint[16384];             //4
        public static bool[] isLeftWaiting = new bool[16384];          //1
        public static bool[] crossStopLine = new bool[16384];          //1

        //20*16384 + 2*36864
        public static bool[] canLeftWaiting = new bool[36864];         //1
        public static bool[] reserved1 = new bool[36864];           //1
        public static bool[] canUTurn = new bool[36864];           //1


        public static byte[] saveData = new byte[438272];

        public static byte lastLanguage = 0;
        public static byte lastRoad = 0;
        //private void ProcessLeftWaiting(ushort vehicleID, ushort nodeId, ushort fromSegmentId, byte fromLaneIndex, ushort toSegmentId, uint laneID, byte offset)

        public static void DataInit()
        {
            for (int i = 0; i < MainDataStore.nodeId.Length; i++)
            {
                nodeId[i] = 0;
                fromSegmentId[i] = 0;
                toSegmentId[i] = 0;
                fromLaneIndex[i] = 0;
                laneID[i] = 0;
                offset[i] = 0;
                nextLaneId[i] = 0;
                additionWaitingTime[i] = 0;
                nextSegOffset[i] = 0;
                isLeftWaiting[i] = false;
                crossStopLine[i] = false;
            }

            for (int i = 0; i < MainDataStore.canLeftWaiting.Length; i++)
            {
                reserved1[i] = false;
                canLeftWaiting[i] = false;
                canUTurn[i] = false;
            }
        }

        public static void save()
        {
            int i = 0;
            SaveAndRestore.save_ushorts(ref i, nodeId, ref saveData);
            SaveAndRestore.save_ushorts(ref i, fromSegmentId, ref saveData);
            SaveAndRestore.save_ushorts(ref i, toSegmentId, ref saveData);
            SaveAndRestore.save_bytes(ref i, fromLaneIndex, ref saveData);
            SaveAndRestore.save_bytes(ref i, offset, ref saveData);
            SaveAndRestore.save_bytes(ref i, additionWaitingTime, ref saveData);
            SaveAndRestore.save_bytes(ref i, nextSegOffset, ref saveData);
            SaveAndRestore.save_uints(ref i, laneID, ref saveData);
            SaveAndRestore.save_uints(ref i, nextLaneId, ref saveData);
            SaveAndRestore.save_bools(ref i, isLeftWaiting, ref saveData);
            SaveAndRestore.save_bools(ref i, crossStopLine, ref saveData);

            SaveAndRestore.save_bools(ref i, canLeftWaiting, ref saveData);
            SaveAndRestore.save_bools(ref i, reserved1, ref saveData);
            SaveAndRestore.save_bools(ref i, canUTurn, ref saveData);
        }

        public static void load()
        {
            int i = 0;
            nodeId = SaveAndRestore.load_ushorts(ref i, saveData, nodeId.Length);
            fromSegmentId = SaveAndRestore.load_ushorts(ref i, saveData, fromSegmentId.Length);
            toSegmentId = SaveAndRestore.load_ushorts(ref i, saveData, toSegmentId.Length);

            fromLaneIndex = SaveAndRestore.load_bytes(ref i, saveData, fromLaneIndex.Length);
            offset = SaveAndRestore.load_bytes(ref i, saveData, offset.Length);
            additionWaitingTime = SaveAndRestore.load_bytes(ref i, saveData, additionWaitingTime.Length);
            nextSegOffset = SaveAndRestore.load_bytes(ref i, saveData, nextSegOffset.Length);

            laneID = SaveAndRestore.load_uints(ref i, saveData, laneID.Length);
            nextLaneId = SaveAndRestore.load_uints(ref i, saveData, nextLaneId.Length);


            isLeftWaiting = SaveAndRestore.load_bools(ref i, saveData, isLeftWaiting.Length);
            crossStopLine = SaveAndRestore.load_bools(ref i, saveData, crossStopLine.Length);
            canLeftWaiting = SaveAndRestore.load_bools(ref i, saveData, canLeftWaiting.Length);
            reserved1 = SaveAndRestore.load_bools(ref i, saveData, reserved1.Length);
            canUTurn = SaveAndRestore.load_bools(ref i, saveData, canUTurn.Length);

        }
    }
}
