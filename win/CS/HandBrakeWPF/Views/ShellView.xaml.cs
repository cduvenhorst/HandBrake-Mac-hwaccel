﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="ShellView.xaml.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   Interaction logic for ShellView.xaml
// </summary> 
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrakeWPF.Views
{
    using System;
    using System.IO;
    using System.Windows;
    using System.Windows.Forms;
    using System.Windows.Input;
    using System.Windows.Resources;

    using Caliburn.Micro;

    using HandBrake.ApplicationServices.Services.Interfaces;
    using HandBrake.ApplicationServices.Utilities;

    using HandBrakeWPF.Commands;
    using HandBrakeWPF.Services.Interfaces;
    using HandBrakeWPF.ViewModels.Interfaces;

    using Application = System.Windows.Application;

    /// <summary>
    /// Interaction logic for ShellView.xaml
    /// </summary>
    public partial class ShellView
    {
        /// <summary>
        /// The my notify icon.
        /// </summary>
        private readonly System.Windows.Forms.NotifyIcon notifyIcon;

        /// <summary>
        /// Initializes a new instance of the <see cref="ShellView"/> class.
        /// </summary>
        public ShellView()
        {
            this.InitializeComponent();

            IUserSettingService userSettingService = IoC.Get<IUserSettingService>();
            bool minimiseToTray = userSettingService.GetUserSetting<bool>(UserSettingConstants.MainWindowMinimize);

            if (minimiseToTray)
            {
                this.notifyIcon = new System.Windows.Forms.NotifyIcon();
                this.notifyIcon.ContextMenu = new ContextMenu(new[] { new MenuItem("Restore", NotifyIconClick) });

                StreamResourceInfo streamResourceInfo = Application.GetResourceStream(new Uri("pack://application:,,,/handbrakepineapple.ico"));
                if (streamResourceInfo != null)
                {
                    Stream iconStream = streamResourceInfo.Stream;
                    this.notifyIcon.Icon = new System.Drawing.Icon(iconStream);
                }
                this.notifyIcon.DoubleClick += this.NotifyIconClick;
                this.StateChanged += this.ShellViewStateChanged;
            }

            // Start Encode (Ctrl+S)
            // Stop Encode (Ctrl+K)
            // Open Log Window (Ctrl+L)
            // Open Queue Window (Ctrl+Q)
            // Add to Queue (Ctrl+A)
            // Scan a File (Ctrl+F)
            // Scan a Folder (Ctrl+R)
            // Show CLI Query (Ctrl+Shift+D)

            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.S, ModifierKeys.Control)), new KeyGesture(Key.S, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.K, ModifierKeys.Control)), new KeyGesture(Key.K, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.L, ModifierKeys.Control)), new KeyGesture(Key.L, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.Q, ModifierKeys.Control)), new KeyGesture(Key.Q, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.A, ModifierKeys.Control)), new KeyGesture(Key.A, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.F, ModifierKeys.Control)), new KeyGesture(Key.F, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.O, ModifierKeys.Control)), new KeyGesture(Key.O, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.R, ModifierKeys.Control)), new KeyGesture(Key.R, ModifierKeys.Control)));
            this.InputBindings.Add(new InputBinding(new ProcessShortcutCommand(new KeyGesture(Key.D, ModifierKeys.Control | ModifierKeys.Shift)), new KeyGesture(Key.D, ModifierKeys.Control | ModifierKeys.Shift)));

            // Enable Windows 7 Taskbar progress indication.
            if (this.TaskbarItemInfo == null)
            {
                this.TaskbarItemInfo = Win7.WindowsTaskbar;
            }

            // Window Sizing
            if (AppArguments.IsInstantHandBrake)
            {
                this.SizeToContent = SizeToContent.WidthAndHeight;
                this.MinHeight = 380;
                this.MinWidth = 600;
            }
        }

        /// <summary>
        /// The notify icon_ click.
        /// </summary>
        /// <param name="sender">
        /// The sender.
        /// </param>
        /// <param name="e">
        /// The e.
        /// </param>
        private void NotifyIconClick(object sender, EventArgs e)
        {
            this.WindowState = WindowState.Normal;
        }

        /// <summary>
        /// The shell view state changed.
        /// </summary>
        /// <param name="sender">
        /// The sender.
        /// </param>
        /// <param name="e">
        /// The e.
        /// </param>
        private void ShellViewStateChanged(object sender, EventArgs e)
        {
            if (this.notifyIcon != null)
            {
                if (this.WindowState == WindowState.Minimized)
                {
                    this.ShowInTaskbar = false;
                    notifyIcon.Visible = true;
                    // notifyIcon.ShowBalloonTip(5000, "HandBrake", "Application Minimised", ToolTipIcon.Info);          
                }
                else if (this.WindowState == WindowState.Normal)
                {
                    notifyIcon.Visible = false;
                    this.ShowInTaskbar = true;
                }
            }
        }

        /// <summary>
        /// Check with the user before closing.
        /// </summary>
        /// <param name="e">
        /// The CancelEventArgs.
        /// </param>
        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            IShellViewModel shellViewModel = this.DataContext as IShellViewModel;

            if (shellViewModel != null)
            {
                bool canClose = shellViewModel.CanClose();
                if (!canClose)
                {
                    e.Cancel = true;
                }
            }

            if (this.notifyIcon != null)
            {
                this.notifyIcon.Visible = false;
            }

            base.OnClosing(e);
        }
    }
}
