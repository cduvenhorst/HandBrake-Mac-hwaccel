﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="UserSettingConstants.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   Constants for the User Settings Service
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrakeWPF
{
    /// <summary>
    /// Constants for the User Settings Service
    /// </summary>
    public class UserSettingConstants
    {
        #region Constants and Fields

        /// <summary>
        /// HandBrakes build
        /// </summary>
        public const string HandBrakeBuild = "HandBrakeBuild";

        /// <summary>
        /// Add Only One Per Langage
        /// </summary>
        public const string AddOnlyOneAudioPerLanguage = "addOnlyOneAudioPerLanguage";

        /// <summary>
        /// Auto name format
        /// </summary>
        public const string AutoNameFormat = "autoNameFormat";

        /// <summary>
        /// Autoname path
        /// </summary>
        public const string AutoNamePath = "autoNamePath";

        /// <summary>
        /// Auto Name Remove underscore
        /// </summary>
        public const string AutoNameRemoveUnderscore = "AutoNameRemoveUnderscore";

        /// <summary>
        /// Auto Name Title Case
        /// </summary>
        public const string AutoNameTitleCase = "AutoNameTitleCase";

        /// <summary>
        /// Auto Naming
        /// </summary>
        public const string AutoNaming = "autoNaming";

        /// <summary>
        /// Clear old logs
        /// </summary>
        public const string ClearOldLogs = "clearOldLogs";

        /// <summary>
        /// Update check interval
        /// </summary>
        public const string DaysBetweenUpdateCheck = "daysBetweenUpdateCheck";

        /// <summary>
        /// Use Default Player
        /// </summary>
        public const string DefaultPlayer = "defaultPlayer";

        /// <summary>
        /// Dub mode
        /// </summary>
        public const string DubMode = "DubMode";

        /// <summary>
        /// Dub Mode Audio
        /// </summary>
        public const string DubModeAudio = "DubModeAudio";

        /// <summary>
        /// Dub Mode Subs
        /// </summary>
        public const string DubModeSubtitle = "DubModeSubtitle";

        /// <summary>
        /// Last Update Check
        /// </summary>
        public const string LastUpdateCheckDate = "lastUpdateCheckDate";

        /// <summary>
        /// Main Window Minimise
        /// </summary>
        public const string MainWindowMinimize = "MainWindowMinimize";

        /// <summary>
        /// Min Title Length
        /// </summary>
        public const string MinTitleLength = "MinTitleLength";

        /// <summary>
        /// Native Language Audio
        /// </summary>
        public const string NativeLanguage = "NativeLanguage";

        /// <summary>
        /// Native Language Subs
        /// </summary>
        public const string NativeLanguageForSubtitles = "NativeLanguageSubtitles";

        /// <summary>
        /// Preset Notification
        /// </summary>
        public const string PresetNotification = "presetNotification";

        /// <summary>
        /// Selected Languages
        /// </summary>
        public const string SelectedLanguages = "SelectedLanguages";

        /// <summary>
        /// AUudio Passthru
        /// </summary>
        public const string ShowAdvancedAudioPassthruOpts = "ShowAdvancedAudioPassthruOpts";

        /// <summary>
        /// Skip Version
        /// </summary>
        public const string Skipversion = "skipversion";

        /// <summary>
        /// Update Status
        /// </summary>
        public const string UpdateStatus = "updateStatus";

        /// <summary>
        /// Closed Captions
        /// </summary>
        public const string UseClosedCaption = "useClosedCaption";

        /// <summary>
        /// Use m4v
        /// </summary>
        public const string UseM4v = "useM4v";

        /// <summary>
        /// Vlc Path
        /// </summary>
        public const string VLC_Path = "VLC_Path";

        /// <summary>
        /// The enable process isolation.
        /// </summary>
        public const string EnableProcessIsolation = "EnableProcessIsolation";

        /// <summary>
        /// The server port.
        /// </summary>
        public const string ServerPort = "ServerPort";

        /// <summary>
        /// Enable the use of LibHb instead of HandBrakeCLI
        /// </summary>
        public const string EnableLibHb = "EnableLibHb";

        /// <summary>
        /// Growl Encodes
        /// </summary>
        public const string GrowlEncode = "GrowlEncode";

        /// <summary>
        /// Growl Queues
        /// </summary>
        public const string GrowlQueue = "GrowlQueue";

        /// <summary>
        /// HandBrakes CLI Exe SHA1 Hash
        /// </summary>
        public const string HandBrakeExeHash = "HandBrakeExeHash";

        /// <summary>
        /// The Instance Id
        /// </summary>
        public const string InstanceId = "InstanceId";

        /// <summary>
        /// The X264 Stepper 
        /// </summary>
        public const string X264Step = "X264Step";

        /// <summary>
        /// The show advanced tab.
        /// </summary>
        public const string ShowAdvancedTab = "ShowAdvancedTab";

        /// <summary>
        /// The last preview duration
        /// </summary>
        public const string LastPreviewDuration = "LastPreviewDuration";
    
        /// <summary>
        /// When Complete Action
        /// </summary>
        public const string WhenCompleteAction = "WhenCompleteAction";

        /// <summary>
        /// Send file enabled.
        /// </summary>
        public const string SendFile = "SendFile";

        /// <summary>
        /// Send file to application path
        /// </summary>
        public const string SendFileTo = "SendFileTo";

        /// <summary>
        /// Send file to arguments
        /// </summary>
        public const string SendFileToArgs = "SendFileToArgs";

        /// <summary>
        /// Prevent Sleep
        /// </summary>
        public const string PreventSleep = "PreventSleep";

        /// <summary>
        /// The remove punctuation.
        /// </summary>
        public const string RemovePunctuation = "RemovePunctuation";

        /// <summary>
        /// The Show Preset Panel
        /// </summary>
        public const string ShowPresetPanel = "ShowPresetPanel";

        /// <summary>
        /// The use system colours.
        /// </summary>
        public const string UseSystemColours = "UseSystemColours";

        /// <summary>
        /// The reset when done action.
        /// </summary>
        public const string ResetWhenDoneAction = "ResetWhenDoneAction";

        /// <summary>
        /// The enable quick sync.
        /// </summary>
        public const string EnableQuickSync = "EnableQuickSync";

        /// <summary>
        /// The add foreign audio scan track.
        /// </summary>
        public const string AddForeignAudioScanTrack = "AddForeignAudioScanTrack";

        /// <summary>
        /// The disable lib dvd nav.
        /// </summary>
        public const string DisableLibDvdNav = "DisableLibDvdNav";

        /// <summary>
        /// The disable quick sync decoding.
        /// </summary>
        public const string DisableQuickSyncDecoding = "DisableQuickSyncDecoding";

        /// <summary>
        /// The enable dxva.
        /// </summary>
        public const string EnableDxva = "EnableDxva";

        /// <summary>
        /// The scaling mode.
        /// </summary>
        public const string ScalingMode = "ScalingMode";

        /// <summary>
        /// Preview Scan Count
        /// </summary>
        public const string PreviewScanCount = "previewScanCount";

        /// <summary>
        /// The Verbosity
        /// </summary>
        public const string Verbosity = "Verbosity";

        /// <summary>
        /// Min Title Scan Duration
        /// </summary>
        public const string MinScanDuration = "MinTitleScanDuration";

        /// <summary>
        /// Process Priority
        /// </summary>
        public const string ProcessPriority = "ProcessPriority";

        /// <summary>
        /// Save Log Directory
        /// </summary>
        public const string SaveLogToCopyDirectory = "SaveLogToCopyDirectory";

        /// <summary>
        /// Save log with video
        /// </summary>
        public const string SaveLogWithVideo = "SaveLogWithVideo";

        /// <summary>
        /// Save copy of the log to a directory
        /// </summary>
        public const string SaveLogCopyDirectory = "SaveLogCopyDirectory";

        /// <summary>
        /// The clear completed from queue.
        /// </summary>
        public const string ClearCompletedFromQueue = "ClearCompletedFromQueue";

        #endregion
    }
}