﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="DriveMenu.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   The drive menu.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrakeWPF.AttachedProperties
{
    using System;
    using System.Collections.Generic;
    using System.Linq;
    using System.Threading;
    using System.Windows;
    using System.Windows.Controls;
    using System.Windows.Media.Imaging;

    using HandBrake.ApplicationServices.Exceptions;
    using HandBrake.ApplicationServices.Utilities;

    using HandBrakeWPF.Commands;
    using HandBrakeWPF.Model;
    using HandBrakeWPF.ViewModels;

    /// <summary>
    /// The drive menu.
    /// </summary>
    public class DriveMenu
    {
        /// <summary>
        /// The show available drives property.
        /// </summary>
        public static readonly DependencyProperty ShowAvailableDrivesProperty = DependencyProperty.RegisterAttached(
            "ShowAvailableDrives",
            typeof(bool),
            typeof(DriveMenu),
            new PropertyMetadata(false, OnShowAvailableDrivesChanged));

        /// <summary>
        /// The get show available drives.
        /// </summary>
        /// <param name="element">
        /// The element.
        /// </param>
        /// <returns>
        /// The <see cref="bool"/>.
        /// </returns>
        public static Boolean GetShowAvailableDrives(MenuItem element)
        {
            bool result;
            return bool.TryParse(element.GetValue(ShowAvailableDrivesProperty).ToString(), out result) && result;
        }

        /// <summary>
        /// The set show available drives.
        /// </summary>
        /// <param name="element">
        /// The element.
        /// </param>
        /// <param name="value">
        /// The value.
        /// </param>
        public static void SetShowAvailableDrives(MenuItem element, Boolean value)
        {
            element.SetValue(ShowAvailableDrivesProperty, value);
        }

        /// <summary>
        /// The on show available drives changed.
        /// </summary>
        /// <param name="d">
        /// The d.
        /// </param>
        /// <param name="e">
        /// The e.
        /// </param>
        private static void OnShowAvailableDrivesChanged(DependencyObject d, DependencyPropertyChangedEventArgs e)
        {
            MenuItem menu = d as MenuItem;
            if (menu != null)
            {
                menu.SubmenuOpened -= MenuMouseDown;
                menu.SubmenuOpened += MenuMouseDown;
            }
        }

        /// <summary>
        /// The menu_ mouse down.
        /// </summary>
        /// <param name="sender">
        /// The sender.
        /// </param>
        /// <param name="e">
        /// The e.
        /// </param>
        private static void MenuMouseDown(object sender, RoutedEventArgs e)
        {
            MenuItem menu = sender as MenuItem;
            MenuItem childMenuItem = e.OriginalSource as MenuItem;
            if (childMenuItem != null && "Title Specific Scan".Equals(childMenuItem.Header))
            {
                return; // Skip, it's just a child menu.
            }

            if (menu != null)
            {
                MainViewModel mvm = menu.DataContext as MainViewModel;
                if (mvm != null)
                {
                    List<SourceMenuItem> remove = mvm.SourceMenu.Where(s => s.IsDrive).ToList();
                    foreach (var item in remove)
                    {
                        mvm.SourceMenu.Remove(item);
                    }

                    foreach (SourceMenuItem menuItem in from item in GeneralUtilities.GetDrives()
                                                        let driveInformation = item
                                                        select new SourceMenuItem
                                                                   {
                                                                       Image = new Image { Source = new BitmapImage(new Uri("pack://application:,,,/HandBrake;component/Views/Images/disc_small.png")), Width = 16, Height = 16 },
                                                                       Text = string.Format("{0} ({1})", item.RootDirectory, item.VolumeLabel),
                                                                       Command = new SourceMenuCommand(() => mvm.ProcessDrive(driveInformation)),
                                                                       Tag = item,
                                                                       IsDrive = true
                                                                   })
                    {
                       mvm.SourceMenu.Add(menuItem);
                    }
                }
                else
                {
                    throw new GeneralApplicationException(
                        "DEBUG - Datacontext wasn't set!", "Please report this on the forum.", null);
                }
            }
            else
            {
                throw new GeneralApplicationException(
                    "DEBUG - Source Menu wasn't set!", "Please report this on the forum.", null);
            }
        }
    }
}
