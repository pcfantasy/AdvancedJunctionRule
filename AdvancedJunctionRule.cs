using ColossalFramework.UI;
using ICities;
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;

namespace AdvancedJunctionRule
{
    public class AdvancedJunctionRule : IUserMod
    {
        public static bool IsEnabled = false;
        public static int language_idex = 0;

        public string Name
        {
            get { return "AdvancedJunctionRule"; }
        }

        public string Description
        {
            get { return "Add more advanced junction rules for Left Right and Uturn"; }
        }

        public void OnEnabled()
        {
            IsEnabled = true;
            FileStream fs = File.Create("AdvancedJunctionRule.txt");
            fs.Close();
            LoadSetting();
            SaveSetting();
            Language.LanguageSwitch((byte)language_idex);
        }

        public void OnDisabled()
        {
            IsEnabled = false;
            Language.LanguageSwitch((byte)language_idex);
        }

        public static void SaveSetting()
        {
            //save langugae
            FileStream fs = File.Create("AdvancedJunctionRule_setting.txt");
            StreamWriter streamWriter = new StreamWriter(fs);
            streamWriter.WriteLine(MainDataStore.lastLanguage);
            streamWriter.Flush();
            fs.Close();
        }

        public static void LoadSetting()
        {
            if (File.Exists("AdvancedJunctionRule_setting.txt"))
            {
                FileStream fs = new FileStream("AdvancedJunctionRule_setting.txt", FileMode.Open);
                StreamReader sr = new StreamReader(fs);
                string strLine = sr.ReadLine();

                if (strLine == "1")
                {
                    MainDataStore.lastLanguage = 1;
                }
                else
                {
                    MainDataStore.lastLanguage = 0;
                }

                strLine = sr.ReadLine();
                sr.Close();
                fs.Close();
            }
        }


        public void OnSettingsUI(UIHelperBase helper)
        {

            LoadSetting();
            Language.LanguageSwitch(MainDataStore.lastLanguage);
            UIHelperBase group = helper.AddGroup(Language.Strings[7]);
            group.AddDropdown(Language.Strings[8], new string[] { "English", "简体中文" }, MainDataStore.lastLanguage, (index) => GetLanguageIdex(index));
            SaveSetting();
        }

        public void GetLanguageIdex(int index)
        {
            language_idex = index;
            Language.LanguageSwitch((byte)language_idex);
            SaveSetting();
            MethodInfo method = typeof(OptionsMainPanel).GetMethod("OnLocaleChanged", BindingFlags.Instance | BindingFlags.NonPublic);
            method.Invoke(UIView.library.Get<OptionsMainPanel>("OptionsPanel"), new object[0]);
            Loader.RemoveGui();
            if (Loader.CurrentLoadMode == LoadMode.LoadGame || Loader.CurrentLoadMode == LoadMode.NewGame)
            {
                if (AdvancedJunctionRule.IsEnabled)
                {
                    Loader.SetupRoadGui();
                }
            }
        }
    }
}
