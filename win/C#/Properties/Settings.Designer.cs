﻿//------------------------------------------------------------------------------
// <auto-generated>
//     This code was generated by a tool.
//     Runtime Version:2.0.50727.3082
//
//     Changes to this file may cause incorrect behavior and will be lost if
//     the code is regenerated.
// </auto-generated>
//------------------------------------------------------------------------------

namespace Handbrake.Properties {
    
    
    [global::System.Runtime.CompilerServices.CompilerGeneratedAttribute()]
    [global::System.CodeDom.Compiler.GeneratedCodeAttribute("Microsoft.VisualStudio.Editors.SettingsDesigner.SettingsSingleFileGenerator", "9.0.0.0")]
    internal sealed partial class Settings : global::System.Configuration.ApplicationSettingsBase {
        
        private static Settings defaultInstance = ((Settings)(global::System.Configuration.ApplicationSettingsBase.Synchronized(new Settings())));
        
        public static Settings Default {
            get {
                return defaultInstance;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("Below Normal")]
        public string processPriority {
            get {
                return ((string)(this["processPriority"]));
            }
            set {
                this["processPriority"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("Automatic")]
        public string Processors {
            get {
                return ((string)(this["Processors"]));
            }
            set {
                this["Processors"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool defaultSettings {
            get {
                return ((bool)(this["defaultSettings"]));
            }
            set {
                this["defaultSettings"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool updateStatus {
            get {
                return ((bool)(this["updateStatus"]));
            }
            set {
                this["updateStatus"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("{hb_version}")]
        public string hb_version {
            get {
                return ((string)(this["hb_version"]));
            }
            set {
                this["hb_version"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool tooltipEnable {
            get {
                return ((bool)(this["tooltipEnable"]));
            }
            set {
                this["tooltipEnable"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("Do Nothing")]
        public string CompletionOption {
            get {
                return ((string)(this["CompletionOption"]));
            }
            set {
                this["CompletionOption"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("")]
        public string defaultPreset {
            get {
                return ((string)(this["defaultPreset"]));
            }
            set {
                this["defaultPreset"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("0")]
        public int hb_build {
            get {
                return ((int)(this["hb_build"]));
            }
            set {
                this["hb_build"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("0")]
        public int skipversion {
            get {
                return ((int)(this["skipversion"]));
            }
            set {
                this["skipversion"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool autoNaming {
            get {
                return ((bool)(this["autoNaming"]));
            }
            set {
                this["autoNaming"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("")]
        public string autoNamePath {
            get {
                return ((string)(this["autoNamePath"]));
            }
            set {
                this["autoNamePath"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("http://handbrake.fr/appcast.xml")]
        public string appcast {
            get {
                return ((string)(this["appcast"]));
            }
            set {
                this["appcast"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("http://handbrake.fr/appcast_unstable.xml")]
        public string appcast_unstable {
            get {
                return ((string)(this["appcast_unstable"]));
            }
            set {
                this["appcast_unstable"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool drive_detection {
            get {
                return ((bool)(this["drive_detection"]));
            }
            set {
                this["drive_detection"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool cli_minimized {
            get {
                return ((bool)(this["cli_minimized"]));
            }
            set {
                this["cli_minimized"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("{source}-{title}")]
        public string autoNameFormat {
            get {
                return ((string)(this["autoNameFormat"]));
            }
            set {
                this["autoNameFormat"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool saveLogToSpecifiedPath {
            get {
                return ((bool)(this["saveLogToSpecifiedPath"]));
            }
            set {
                this["saveLogToSpecifiedPath"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("")]
        public string saveLogPath {
            get {
                return ((string)(this["saveLogPath"]));
            }
            set {
                this["saveLogPath"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool saveLogWithVideo {
            get {
                return ((bool)(this["saveLogWithVideo"]));
            }
            set {
                this["saveLogWithVideo"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("C:\\Program Files\\VideoLAN\\vlc\\vlc.exe")]
        public string VLC_Path {
            get {
                return ((string)(this["VLC_Path"]));
            }
            set {
                this["VLC_Path"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool MainWindowMinimize {
            get {
                return ((bool)(this["MainWindowMinimize"]));
            }
            set {
                this["MainWindowMinimize"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool QueryEditorTab {
            get {
                return ((bool)(this["QueryEditorTab"]));
            }
            set {
                this["QueryEditorTab"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("0.25")]
        public double x264cqstep {
            get {
                return ((double)(this["x264cqstep"]));
            }
            set {
                this["x264cqstep"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("1")]
        public int verboseLevel {
            get {
                return ((int)(this["verboseLevel"]));
            }
            set {
                this["verboseLevel"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool dvdnav {
            get {
                return ((bool)(this["dvdnav"]));
            }
            set {
                this["dvdnav"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool presetNotification {
            get {
                return ((bool)(this["presetNotification"]));
            }
            set {
                this["presetNotification"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool enocdeStatusInGui {
            get {
                return ((bool)(this["enocdeStatusInGui"]));
            }
            set {
                this["enocdeStatusInGui"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool trayIconAlerts {
            get {
                return ((bool)(this["trayIconAlerts"]));
            }
            set {
                this["trayIconAlerts"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        public global::System.DateTime lastUpdateCheckDate {
            get {
                return ((global::System.DateTime)(this["lastUpdateCheckDate"]));
            }
            set {
                this["lastUpdateCheckDate"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("7")]
        public int daysBetweenUpdateCheck {
            get {
                return ((int)(this["daysBetweenUpdateCheck"]));
            }
            set {
                this["daysBetweenUpdateCheck"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool useM4v {
            get {
                return ((bool)(this["useM4v"]));
            }
            set {
                this["useM4v"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("True")]
        public bool PromptOnUnmatchingQueries {
            get {
                return ((bool)(this["PromptOnUnmatchingQueries"]));
            }
            set {
                this["PromptOnUnmatchingQueries"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("Any")]
        public string NativeLanguage {
            get {
                return ((string)(this["NativeLanguage"]));
            }
            set {
                this["NativeLanguage"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool DubAudio {
            get {
                return ((bool)(this["DubAudio"]));
            }
            set {
                this["DubAudio"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("")]
        public string hb_platform {
            get {
                return ((string)(this["hb_platform"]));
            }
            set {
                this["hb_platform"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool disableResCalc {
            get {
                return ((bool)(this["disableResCalc"]));
            }
            set {
                this["disableResCalc"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool growlQueue {
            get {
                return ((bool)(this["growlQueue"]));
            }
            set {
                this["growlQueue"] = value;
            }
        }
        
        [global::System.Configuration.UserScopedSettingAttribute()]
        [global::System.Diagnostics.DebuggerNonUserCodeAttribute()]
        [global::System.Configuration.DefaultSettingValueAttribute("False")]
        public bool growlEncode {
            get {
                return ((bool)(this["growlEncode"]));
            }
            set {
                this["growlEncode"] = value;
            }
        }
    }
}
