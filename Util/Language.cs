using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace AdvancedJunctionRule.Util
{
    public class Language
    {
        public static string[] English =
        {
            "Vehicle can turn right with red light",                                                                                      //0
            "Vehicle can enter left waiting area",
            "Right Turn Rule Tips:",
            "Left Turn Rule Tips:",
            "Find TMPE traffic light in the junction, use it for right turn",//1
            "No traffic light in this segment",//1
            "Need TMPE traffic light for left waiting rule",
            "Language",                                                                                      //0
            "Language_Select",
            "Vehicle can U-turn with red light",
            "U Turn Rule Tips:",
        };



        public static string[] Chinese =
            {
            "允许车辆红灯右转",                                                   //0
            "允许车辆左转待行",
            "右转功能规则提示:",
            "左转功能规则提示:",
            "路口有TMPE定制红绿灯,用它来控制右转即可",//1
            "这段路没有红绿灯",//1
            "需要TMPE定制红绿灯来完成左转待行设置",//1
            "语言",                                                                                      //0
            "语言选择",
            "允许车辆红灯调头",
            "掉头规则提示:",
        };


        public static string[] Strings = new string[English.Length];

        public static byte currentLanguage = 255;

        public static void LanguageSwitch(byte language)
        {
            if (language == 1)
            {
                for (int i = 0; i < English.Length; i++)
                {
                    Strings[i] = Chinese[i];
                }
                currentLanguage = 1;
            }
            else if (language == 0)
            {
                for (int i = 0; i < English.Length; i++)
                {
                    Strings[i] = English[i];
                }
                currentLanguage = 0;
            }
            else
            {
                DebugLog.LogToFileOnly("unknow language!! use English");
                for (int i = 0; i < English.Length; i++)
                {
                    Strings[i] = English[i];
                }
                currentLanguage = 0;
            }

            MainDataStore.lastLanguage = currentLanguage;
        }
    }
}
