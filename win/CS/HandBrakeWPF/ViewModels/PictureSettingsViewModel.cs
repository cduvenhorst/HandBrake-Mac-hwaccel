﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="PictureSettingsViewModel.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   The Picture Settings View Model
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrakeWPF.ViewModels
{
    using System;
    using System.Collections.Generic;
    using System.Globalization;

    using Caliburn.Micro;

    using HandBrake.ApplicationServices.Model;
    using HandBrake.ApplicationServices.Parsing;
    using HandBrake.ApplicationServices.Services.Interfaces;
    using HandBrake.Interop.Model;
    using HandBrake.Interop.Model.Encoding;

    using HandBrakeWPF.Helpers;
    using HandBrakeWPF.ViewModels.Interfaces;

    using Size = System.Drawing.Size;

    /// <summary>
    /// The Picture Settings View Model
    /// </summary>
    public class PictureSettingsViewModel : ViewModelBase, IPictureSettingsViewModel
    {
        /*
         * TODO:
         * - We are not handling cropping correctly within the UI.
         * - The Height is not correctly set when using no Anamorphic
         * - Maintain Aspect ratio needs corrected.
         * - Custom Anamorphic.
         * 
         */
        #region Constants and Fields

        /// <summary>
        /// The display size.
        /// </summary>
        private string displaySize;

        /// <summary>
        ///  Backing field for for height control enabled
        /// </summary>
        private bool heightControlEnabled = true;

        /// <summary>
        ///  Backing field for show custom anamorphic controls
        /// </summary>
        private bool showCustomAnamorphicControls;

        /// <summary>
        /// The source info.
        /// </summary>
        private string sourceInfo;

        /// <summary>
        /// Source Par Values
        /// </summary>
        private Size sourceParValues;

        /// <summary>
        /// Source Resolution
        /// </summary>
        private Size sourceResolution;

        /// <summary>
        /// Backing field for width control enabled.
        /// </summary>
        private bool widthControlEnabled = true;

        /// <summary>
        /// Backing field for the show modulus field
        /// </summary>
        private bool showModulus;

        /// <summary>
        /// Backing field for showing display size.
        /// </summary>
        private bool showDisplaySize;

        /// <summary>
        /// Backing field for max height
        /// </summary>
        private int maxHeight;

        /// <summary>
        /// Backing field for max width
        /// </summary>
        private int maxWidth;

        /// <summary>
        /// The show keep ar backing field.
        /// </summary>
        private bool showKeepAr = true;

        #endregion

        #region Constructors and Destructors

        /// <summary>
        /// Initializes a new instance of the <see cref="HandBrakeWPF.ViewModels.PictureSettingsViewModel"/> class.
        /// </summary>
        /// <param name="windowManager">
        /// The window manager.
        /// </param>
        /// <param name="userSettingService">
        /// The user Setting Service.
        /// </param>
        public PictureSettingsViewModel(IWindowManager windowManager, IUserSettingService userSettingService)
        {
            this.Task = new EncodeTask();
            this.SelectedModulus = 16;
            this.MaintainAspectRatio = true;

            // Default the Max Width / Height to 1080p format
            this.MaxHeight = 1080;
            this.MaxWidth = 1920;
        }

        #endregion

        #region Properties

        /// <summary>
        /// Gets AnamorphicModes.
        /// </summary>
        public IEnumerable<Anamorphic> AnamorphicModes
        {
            get
            {
                return new List<Anamorphic> { Anamorphic.None, Anamorphic.Strict, Anamorphic.Loose, Anamorphic.Custom };
            }
        }

        /// <summary>
        /// Gets or sets CropBottom.
        /// </summary>
        public int CropBottom
        {
            get
            {
                return this.Task.Cropping.Bottom;
            }

            set
            {
                this.Task.Cropping.Bottom = value;
                this.NotifyOfPropertyChange(() => this.CropBottom);
                this.CropAdjust();
                this.SetDisplaySize();
            }
        }

        /// <summary>
        /// Gets or sets CropLeft.
        /// </summary>
        public int CropLeft
        {
            get
            {
                return this.Task.Cropping.Left;
            }

            set
            {
                this.Task.Cropping.Left = value;
                this.NotifyOfPropertyChange(() => this.CropLeft);
                this.CropAdjust();
                this.SetDisplaySize();
            }
        }

        /// <summary>
        /// Gets or sets CropRight.
        /// </summary>
        public int CropRight
        {
            get
            {
                return this.Task.Cropping.Right;
            }

            set
            {
                this.Task.Cropping.Right = value;
                this.NotifyOfPropertyChange(() => this.CropRight);
                this.CropAdjust();
                this.SetDisplaySize();
            }
        }

        /// <summary>
        /// Gets or sets CropTop.
        /// </summary>
        public int CropTop
        {
            get
            {
                return this.Task.Cropping.Top;
            }

            set
            {
                this.Task.Cropping.Top = value;
                this.NotifyOfPropertyChange(() => this.CropTop);
                this.CropAdjust();
                this.SetDisplaySize();
            }
        }

        /// <summary>
        /// Gets or sets DisplaySize.
        /// </summary>
        public string DisplaySize
        {
            get
            {
                return this.displaySize;
            }

            set
            {
                this.displaySize = value;
                this.NotifyOfPropertyChange(() => this.DisplaySize);
            }
        }

        /// <summary>
        /// Gets or sets DisplayWidth.
        /// </summary>
        public int DisplayWidth
        {
            get
            {
                return this.Task.DisplayWidth.HasValue
                           ? int.Parse(Math.Round(this.Task.DisplayWidth.Value, 0).ToString(CultureInfo.InvariantCulture))
                           : 0;
            }

            set
            {
                if (!object.Equals(this.Task.DisplayWidth, value))
                {
                    this.Task.DisplayWidth = value;
                    this.CustomAnamorphicAdjust();
                    this.NotifyOfPropertyChange(() => this.DisplayWidth);
                }
            }
        }

        /// <summary>
        /// Gets or sets Height.
        /// </summary>
        public int Height
        {
            get
            {
                return this.Task.Height.HasValue ? this.Task.Height.Value : 0;
            }

            set
            {
                if (!object.Equals(this.Task.Height, value))
                {
                    this.Task.Height = value;
                    this.HeightAdjust();
                    this.NotifyOfPropertyChange(() => this.Height);
                }
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether HeightControlEnabled.
        /// </summary>
        public bool HeightControlEnabled
        {
            get
            {
                return this.heightControlEnabled;
            }

            set
            {
                this.heightControlEnabled = value;
                this.NotifyOfPropertyChange(() => this.HeightControlEnabled);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether IsCustomCrop.
        /// </summary>
        public bool IsCustomCrop
        {
            get
            {
                return this.Task.HasCropping;
            }

            set
            {
                this.Task.HasCropping = value;
                this.NotifyOfPropertyChange(() => this.IsCustomCrop);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether MaintainAspectRatio.
        /// </summary>
        public bool MaintainAspectRatio
        {
            get
            {
                return this.Task.KeepDisplayAspect;
            }

            set
            {
                this.Task.KeepDisplayAspect = value;
                this.WidthAdjust();
                this.NotifyOfPropertyChange(() => this.MaintainAspectRatio);
            }
        }

        /// <summary>
        /// Gets ModulusValues.
        /// </summary>
        public IEnumerable<int> ModulusValues
        {
            get
            {
                return new List<int> { 16, 8, 4, 2 };
            }
        }

        /// <summary>
        /// Gets or sets ParHeight.
        /// </summary>
        public int ParHeight
        {
            get
            {
                return this.Task.PixelAspectY;
            }

            set
            {
                if (!object.Equals(this.Task.PixelAspectY, value))
                {
                    this.Task.PixelAspectY = value;
                    this.CustomAnamorphicAdjust();
                    this.NotifyOfPropertyChange(() => this.ParHeight);
                }
            }
        }

        /// <summary>
        /// Gets or sets ParWidth.
        /// </summary>
        public int ParWidth
        {
            get
            {
                return this.Task.PixelAspectX;
            }

            set
            {
                if (!object.Equals(this.Task.PixelAspectX, value))
                {
                    this.Task.PixelAspectX = value;
                    this.CustomAnamorphicAdjust();
                    this.NotifyOfPropertyChange(() => this.ParWidth);
                }
            }
        }

        /// <summary>
        /// Gets or sets SelectedAnamorphicMode.
        /// </summary>
        public Anamorphic SelectedAnamorphicMode
        {
            get
            {
                return this.Task.Anamorphic;
            }

            set
            {
                if (!object.Equals(this.SelectedAnamorphicMode, value))
                {
                    this.Task.Anamorphic = value;
                    this.AnamorphicAdjust();
                    this.NotifyOfPropertyChange(() => this.SelectedAnamorphicMode);
                }
            }
        }

        /// <summary>
        /// Gets or sets SelectedModulus.
        /// </summary>
        public int? SelectedModulus
        {
            get
            {
                return this.Task.Modulus;
            }

            set
            {
                this.Task.Modulus = value;
                this.ModulusAdjust();
                this.NotifyOfPropertyChange(() => this.SelectedModulus);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether ShowCustomAnamorphicControls.
        /// </summary>
        public bool ShowCustomAnamorphicControls
        {
            get
            {
                return this.showCustomAnamorphicControls;
            }

            set
            {
                this.showCustomAnamorphicControls = value;
                this.NotifyOfPropertyChange(() => this.ShowCustomAnamorphicControls);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether ShowModulus.
        /// </summary>
        public bool ShowModulus
        {
            get
            {
                return this.showModulus;
            }
            set
            {
                this.showModulus = value;
                this.NotifyOfPropertyChange(() => this.ShowModulus);
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether ShowDisplaySize.
        /// </summary>
        public bool ShowDisplaySize
        {
            get
            {
                return this.showDisplaySize;
            }
            set
            {
                this.showDisplaySize = value;
                this.NotifyOfPropertyChange(() => this.ShowDisplaySize);
            }
        }

        /// <summary>
        /// Gets or sets SourceInfo.
        /// </summary>
        public string SourceInfo
        {
            get
            {
                return this.sourceInfo;
            }

            set
            {
                this.sourceInfo = value;
                this.NotifyOfPropertyChange(() => this.SourceInfo);
            }
        }

        /// <summary>
        /// Gets or sets Task.
        /// </summary>
        public EncodeTask Task { get; set; }

        /// <summary>
        /// Gets or sets Width.
        /// </summary>
        public int Width
        {
            get
            {
                return this.Task.Width.HasValue ? this.Task.Width.Value : 0;
            }

            set
            {
                if (!object.Equals(this.Task.Width, value))
                {
                    this.Task.Width = value;
                    this.WidthAdjust();
                    this.NotifyOfPropertyChange(() => this.Width);
                }
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether WidthControlEnabled.
        /// </summary>
        public bool WidthControlEnabled
        {
            get
            {
                return this.widthControlEnabled;
            }

            set
            {
                this.widthControlEnabled = value;
                this.NotifyOfPropertyChange(() => this.WidthControlEnabled);
            }
        }

        /// <summary>
        /// Gets or sets MaxHeight.
        /// </summary>
        public int MaxHeight
        {
            get
            {
                return this.maxHeight;
            }
            set
            {
                this.maxHeight = value;
                this.NotifyOfPropertyChange(() => this.MaxHeight);
            }
        }

        /// <summary>
        /// Gets or sets MinHeight.
        /// </summary>
        public int MaxWidth
        {
            get
            {
                return this.maxWidth;
            }
            set
            {
                this.maxWidth = value;
                this.NotifyOfPropertyChange(() => this.MaxWidth);
            }
        }

        /// <summary>
        /// Gets SourceAspect.
        /// </summary>
        private Size SourceAspect
        {
            get
            {
                // display aspect = (width * par_width) / (height * par_height)
                return new Size(
                    (this.sourceParValues.Width * this.sourceResolution.Width),
                    (this.sourceParValues.Height * this.sourceResolution.Height));
            }
        }

        /// <summary>
        /// Gets or sets a value indicating whether show keep ar.
        /// </summary>
        public bool ShowKeepAR
        {
            get
            {
                return this.showKeepAr;
            }
            set
            {
                this.showKeepAr = value;
                this.NotifyOfPropertyChange(() => this.ShowKeepAR);
            }
        }

        #endregion

        #region Implemented Interfaces

        #region ITabInterface

        /// <summary>
        /// Setup this tab for the specified preset.
        /// </summary>
        /// <param name="preset">
        /// The preset.
        /// </param>
        /// <param name="task">
        /// The task.
        /// </param>
        public void SetPreset(Preset preset, EncodeTask task)
        {
            this.Task = task;

            // Handle built-in presets.
            if (preset.IsBuildIn)
            {
                preset.PictureSettingsMode = PresetPictureSettingsMode.Custom;
            }

            // Setup the Picture Sizes
            switch (preset.PictureSettingsMode)
            {
                default:
                case PresetPictureSettingsMode.Custom:
                case PresetPictureSettingsMode.SourceMaximum:

                    // Anamorphic Mode
                    this.SelectedAnamorphicMode = preset.Task.Anamorphic;

                    // Modulus
                    if (preset.Task.Modulus.HasValue)
                    {
                        this.SelectedModulus = preset.Task.Modulus;
                    }

                    // Set the Maintain Aspect ratio.
                    this.MaintainAspectRatio = preset.Task.KeepDisplayAspect;

                    // Set the width, then check the height doesn't breach the max height and correct if necessary.
                    int width = this.GetModulusValue(this.getRes((this.sourceResolution.Width - this.CropLeft - this.CropRight), preset.Task.MaxWidth));
                    this.Width = width;

                    // If we have a max height, make sure we havn't breached it.
                    int height = this.GetModulusValue(this.getRes((this.sourceResolution.Height - this.CropTop - this.CropBottom), preset.Task.MaxHeight));
                    if (preset.Task.MaxHeight.HasValue && this.Height > preset.Task.MaxHeight.Value)
                    {
                        this.Height = height;
                    }

                    this.MaxWidth = width;
                    this.MaxHeight = height;
                    break;
                case PresetPictureSettingsMode.None:
                    // Do Nothing except reset the Max Width/Height
                    this.MaxWidth = this.sourceResolution.Width;
                    this.MaxHeight = this.sourceResolution.Height;
                    this.SelectedAnamorphicMode = preset.Task.Anamorphic;
                    break;
            }

            // Custom Anamorphic
            if (preset.Task.Anamorphic == Anamorphic.Custom)
            {
                this.DisplayWidth = preset.Task.DisplayWidth != null ? int.Parse(preset.Task.DisplayWidth.ToString()) : 0;
                this.ParWidth = preset.Task.PixelAspectX;
                this.ParHeight = preset.Task.PixelAspectY;
            }

            // Cropping
            if (preset.Task.HasCropping)
            {
                this.IsCustomCrop = true;
                this.CropLeft = preset.Task.Cropping.Left;
                this.CropRight = preset.Task.Cropping.Right;
                this.CropTop = preset.Task.Cropping.Top;
                this.CropBottom = preset.Task.Cropping.Bottom;
            }
            else
            {
                this.IsCustomCrop = false;
            }

            this.NotifyOfPropertyChange(() => this.Task);
        }

        /// <summary>
        /// Update all the UI controls based on the encode task passed in.
        /// </summary>
        /// <param name="task">
        /// The task.
        /// </param>
        public void UpdateTask(EncodeTask task)
        {
            this.Task = task;
            this.NotifyOfPropertyChange(() => this.Width);
            this.NotifyOfPropertyChange(() => this.Height);
            this.NotifyOfPropertyChange(() => this.SelectedAnamorphicMode);
            this.NotifyOfPropertyChange(() => this.SelectedModulus);
        }

        /// <summary>
        /// Setup this window for a new source
        /// </summary>
        /// <param name="title">
        /// The title.
        /// </param>
        /// <param name="preset">
        /// The preset.
        /// </param>
        /// <param name="task">
        /// The task.
        /// </param>
        public void SetSource(Title title, Preset preset, EncodeTask task)
        {
            this.Task = task;

            if (title != null)
            {
                // Set cached info
                this.sourceParValues = title.ParVal;
                this.sourceResolution = title.Resolution;

                // Update the cropping values, preffering those in the presets.
                if (!preset.Task.HasCropping)
                {
                    this.CropTop = title.AutoCropDimensions.Top;
                    this.CropBottom = title.AutoCropDimensions.Bottom;
                    this.CropLeft = title.AutoCropDimensions.Left;
                    this.CropRight = title.AutoCropDimensions.Right;
                    this.IsCustomCrop = false;
                }
                else
                {
                    this.CropLeft = preset.Task.Cropping.Left;
                    this.CropRight = preset.Task.Cropping.Right;
                    this.CropTop = preset.Task.Cropping.Top;
                    this.CropBottom = preset.Task.Cropping.Bottom;
                    this.IsCustomCrop = true;
                }

                if (preset.PictureSettingsMode == PresetPictureSettingsMode.None)
                {
                    // We have no instructions, so simply set it to the source.
                    this.Width = this.GetModulusValue(this.sourceResolution.Width - this.CropLeft - this.CropRight);
                    this.MaintainAspectRatio = true;
                }
                else
                {
                    // Set the Max Width / Height available to the user controls
                    if (this.sourceResolution.Width < this.MaxWidth)
                    {
                        this.MaxWidth = this.sourceResolution.Width;
                    }
                    else if (this.sourceResolution.Width > this.MaxWidth)
                    {
                        this.MaxWidth = preset.Task.MaxWidth ?? this.sourceResolution.Width;
                    }

                    if (this.sourceResolution.Height < this.MaxHeight)
                    {
                        this.MaxHeight = this.sourceResolution.Height;
                    }
                    else if (this.sourceResolution.Height > this.MaxHeight)
                    {
                        this.MaxHeight = preset.Task.MaxHeight ?? this.sourceResolution.Height;
                    }

                    // Set the Width, and Maintain Aspect ratio. That should calc the Height for us.
                    if (this.SelectedAnamorphicMode == Anamorphic.None)
                    {
                        this.Width = preset.Task.Width ?? (this.MaxWidth - this.CropLeft - this.CropRight);
                            // Note: This will be auto-corrected in the property if it's too large.
                    }
                    else
                    {
                        this.Width = preset.Task.Width ?? this.MaxWidth;
                    }

                    // If our height is too large, let it downscale the width for us by setting the height to the lower value.
                    if (!this.MaintainAspectRatio && this.Height > this.MaxHeight)
                    {
                        this.Height = this.MaxHeight;
                    }

                    if (this.SelectedAnamorphicMode == Anamorphic.Custom)
                    {
                        this.AnamorphicAdjust(); // Refresh the values
                    }
                }

                // Set Screen Controls
                this.SourceInfo = string.Format(
                    "{0}x{1}, Aspect Ratio: {2:0.00}, PAR: {3}/{4}",
                    title.Resolution.Width,
                    title.Resolution.Height,
                    title.AspectRatio,
                    title.ParVal.Width,
                    title.ParVal.Height);
            }

            this.NotifyOfPropertyChange(() => this.Task);
        }

        #endregion

        #endregion

        #region Methods

        /// <summary>
        /// The crop adjust.
        /// </summary>
        private void CropAdjust()
        {
            PictureSize.AnamorphicResult result = PictureSize.hb_set_anamorphic_size(this.GetPictureSettings(), this.GetPictureTitleInfo());
            switch (this.SelectedAnamorphicMode)
            {
                case Anamorphic.None:
                    //this.Width = result.OutputWidth;
                    //this.Height = result.OutputHeight;
                    break;
                case Anamorphic.Strict:
                case Anamorphic.Loose:
                case Anamorphic.Custom:
                    double dispWidth = Math.Round((result.OutputWidth * result.OutputParWidth / result.OutputParHeight), 0);
                    this.DisplaySize = this.sourceResolution.IsEmpty
                                   ? string.Empty
                                   : string.Format("Storage: {0}x{1}, Display: {2}x{3}", result.OutputWidth, result.OutputHeight, dispWidth, result.OutputHeight);
                    break;
            }
        }

        /// <summary>
        /// Adjust other values after the user has altered the anamorphic.
        /// </summary>
        private void AnamorphicAdjust()
        {
            this.ShowDisplaySize = true;
            this.ShowKeepAR = true;
            switch (this.SelectedAnamorphicMode)
            {
                case Anamorphic.None:
                    this.WidthControlEnabled = true;
                    this.HeightControlEnabled = true;
                    this.ShowCustomAnamorphicControls = false;
                    this.ShowModulus = true;
                    this.ShowDisplaySize = true;
                    this.ShowKeepAR = true;
                    this.SelectedModulus = 16; // Reset
                    if (this.Width == 0)
                    {
                        this.Width = this.GetModulusValue(this.sourceResolution.Width - this.CropLeft - this.CropRight);
                    }

                    if (!this.MaintainAspectRatio && this.Height == 0)
                    {
                        this.Height = this.GetModulusValue(this.sourceResolution.Height - this.CropTop - this.CropBottom);
                    }

                    this.MaintainAspectRatio = true;

                    this.SetDisplaySize();
                    break;
                case Anamorphic.Strict:
                    this.WidthControlEnabled = false;
                    this.HeightControlEnabled = false;
                    this.ShowCustomAnamorphicControls = false;
                    this.ShowModulus = false;
                    this.SelectedModulus = 16; // Reset
                    this.ShowKeepAR = false;



                    this.Width = 0;
                    this.Height = 0;
                    this.SetDisplaySize();
                    break;

                case Anamorphic.Loose:
                    this.WidthControlEnabled = true;
                    this.HeightControlEnabled = false;
                    this.ShowCustomAnamorphicControls = false;
                    this.ShowModulus = true;

                    // Reset to the source size.
                    this.Width = this.sourceResolution.Width;
                    this.Height = 0; //this.sourceResolution.Height - this.CropTop - this.CropBottom;
                    this.ShowKeepAR = false;

                    this.SetDisplaySize();
                    break;

                case Anamorphic.Custom:
                    this.WidthControlEnabled = true;
                    this.HeightControlEnabled = true;
                    this.ShowCustomAnamorphicControls = true;
                    this.MaintainAspectRatio = false;  // TODO Fix when implementing custom
                    this.ShowModulus = true;
                    this.ShowDisplaySize = false; // Disabled for Custom until we implement it properly. TODO
                    this.ShowKeepAR = false;

                    // Ignore any of the users current settings and reset to source to make things easier.
                    this.Width = this.sourceResolution.Width;
                    this.Height = this.sourceResolution.Height - this.CropTop - this.CropBottom;

                    // Set the Display Width and set the Par X/Y to the source values initially.      
                    this.ParWidth = this.sourceParValues.Width;
                    this.ParHeight = this.sourceParValues.Height;
                    if (this.ParHeight != 0)
                    {
                        this.DisplayWidth = (this.Width * this.ParWidth / this.ParHeight);
                    }

                    //this.SetDisplaySize();
                    break;
            }
        }

        /// <summary>
        /// Adjust other values after the user has altered one of the custom anamorphic settings
        /// </summary>
        private void CustomAnamorphicAdjust()
        {
            if (this.SelectedAnamorphicMode == Anamorphic.Custom)
            {
                if (this.MaintainAspectRatio && this.DisplayWidth != 0)
                {
                    this.ParWidth = this.DisplayWidth;
                    this.ParHeight = this.Width;
                }

                this.SetDisplaySize();
            }
        }

        /// <summary>
        /// For a given value, correct so that it matches the users currently selected modulus value
        /// </summary>
        /// <param name="value">
        /// The value.
        /// </param>
        /// <returns>
        /// Value corrected so that value % selected modulus == 0
        /// </returns>
        private int GetModulusValue(double value)
        {
            if (this.SelectedModulus == null)
            {
                return 0;
            }

            double remainder = value % this.SelectedModulus.Value;

            if (remainder.Equals(0.0d))
            {
                return (int)Math.Abs(value);
            }

            double result = remainder >= ((double)this.SelectedModulus.Value / 2)
                       ? value + (this.SelectedModulus.Value - remainder)
                       : value - remainder;

            return (int)Math.Abs(result);
        }

        /// <summary>
        /// Adjust other values after the user has altered the height
        /// </summary>
        private void HeightAdjust()
        {
            if (this.sourceResolution == new Size(0, 0))
            {
                return;
            }

            if (this.Height > this.sourceResolution.Height)
            {
                this.Task.Height = this.sourceResolution.Height;
                this.NotifyOfPropertyChange(() => this.Task.Height);
            }

            switch (this.SelectedAnamorphicMode)
            {
                case Anamorphic.None:
                    if (this.MaintainAspectRatio)
                    {
                        double cropWidth = this.sourceResolution.Width - this.CropLeft - this.CropRight;
                        double cropHeight = this.sourceResolution.Height - this.CropTop - this.CropBottom;

                        double newWidth = ((double)this.Height * this.sourceResolution.Height * this.SourceAspect.Width *
                                            cropWidth) /
                                           ((double)this.sourceResolution.Width * this.SourceAspect.Height * cropHeight);

                        this.Task.Width = this.GetModulusValue(newWidth);
                        this.NotifyOfPropertyChange(() => this.Task.Width);
                    }

                    break;
                case Anamorphic.Custom:
                    this.SetDisplaySize();
                    break;
            }
        }

        /// <summary>
        /// Adjust other values after the user has altered the modulus
        /// </summary>
        private void ModulusAdjust()
        {
            this.WidthAdjust();
        }

        /// <summary>
        /// Set the display size text
        /// </summary>
        private void SetDisplaySize()
        {
            /*
            * Handle Anamorphic Display
            */
            if (this.SelectedAnamorphicMode != Anamorphic.None)
            {
                PictureSize.AnamorphicResult result = PictureSize.hb_set_anamorphic_size(this.GetPictureSettings(), this.GetPictureTitleInfo());
                double dispWidth = Math.Round((result.OutputWidth * result.OutputParWidth / result.OutputParHeight), 0);

                this.DisplaySize = this.sourceResolution.IsEmpty
                                     ? string.Empty
                                     : string.Format("Output: {0}x{1}, Anamorphic: {2}x{3}", result.OutputWidth, result.OutputHeight, dispWidth, result.OutputHeight);
            }
            else
            {
                this.DisplaySize = this.sourceResolution.IsEmpty
                     ? string.Empty
                     : string.Format("Output: {0}x{1}", this.Width, this.Height);
            }
        }

        /// <summary>
        /// Adjust other values after the user has altered the width
        /// </summary>
        private void WidthAdjust()
        {
            if (this.Width > this.sourceResolution.Width)
            {
                this.Task.Width = this.sourceResolution.Width;
                this.NotifyOfPropertyChange(() => this.Task.Width);
            }

            switch (this.SelectedAnamorphicMode)
            {
                case Anamorphic.None:
                    if (this.MaintainAspectRatio)
                    {
                        double cropWidth = this.sourceResolution.Width - this.CropLeft - this.CropRight;
                        double cropHeight = this.sourceResolution.Height - this.CropTop - this.CropBottom;

                        if (this.SourceAspect.Width == 0 && this.SourceAspect.Height == 0)
                        {
                            break;
                        }

                        double newHeight = ((double)this.Width * this.sourceResolution.Width * this.SourceAspect.Height *
                                            cropHeight) /
                                           ((double)this.sourceResolution.Height * this.SourceAspect.Width * cropWidth);

                        this.Task.Height = this.GetModulusValue(newHeight);
                        this.NotifyOfPropertyChange(() => this.Height);
                    }
                    this.SetDisplaySize();
                    break;
                case Anamorphic.Strict:
                    this.Task.Width = 0;
                    this.Task.Height = 0;

                    this.NotifyOfPropertyChange(() => this.Width);
                    this.NotifyOfPropertyChange(() => this.Height);
                    this.SetDisplaySize();
                    break;
                case Anamorphic.Loose:
                    this.Task.Height = 0;
                    this.NotifyOfPropertyChange(() => this.Width);
                    this.NotifyOfPropertyChange(() => this.Height);
                    this.SetDisplaySize();
                    break;
                case Anamorphic.Custom:
                    if (this.MaintainAspectRatio)
                    {
                        this.ParWidth = this.DisplayWidth;
                        this.ParHeight = this.Width;
                    }

                    this.SetDisplaySize();
                    break;
            }
        }

        /// <summary>
        /// Quick function to get the max resolution value
        ///  </summary>
        /// <param name="value">
        /// The value.
        /// </param>
        /// <param name="max">
        /// The max.
        /// </param>
        /// <returns>
        /// An Int
        /// </returns>
        private int getRes(int value, int? max)
        {
            return max.HasValue ? (value > max.Value ? max.Value : value) : value;
        }

        /// <summary>
        /// The get picture title info.
        /// </summary>
        /// <returns>
        /// The <see cref="PictureSize.PictureSettingsTitle"/>.
        /// </returns>
        private PictureSize.PictureSettingsTitle GetPictureTitleInfo()
        {
            PictureSize.PictureSettingsTitle title = new PictureSize.PictureSettingsTitle
            {
                Width = this.sourceResolution.Width,
                Height = this.sourceResolution.Height,
                ParW = this.sourceParValues.Width,
                ParH = this.sourceParValues.Height,
                Aspect = 0 // TODO
            };

            return title;
        }

        /// <summary>
        /// The get picture settings.
        /// </summary>
        /// <returns>
        /// The <see cref="PictureSize.PictureSettingsJob"/>.
        /// </returns>
        private PictureSize.PictureSettingsJob GetPictureSettings()
        {
            PictureSize.PictureSettingsJob job = new PictureSize.PictureSettingsJob
            {
                Width = this.Width,
                Height = this.Height,
                ItuPar = false,
                Modulus = this.SelectedModulus,
                ParW = this.ParWidth,
                ParH = this.ParHeight,
                MaxWidth = this.MaxWidth,
                MaxHeight = this.MaxHeight,
                KeepDisplayAspect = this.MaintainAspectRatio,
                AnamorphicMode = this.SelectedAnamorphicMode,
                DarWidth = 0,
                DarHeight = 0,
                Crop = new Cropping(this.CropTop, this.CropBottom, this.CropLeft, this.CropRight),
            };

            if (this.SelectedAnamorphicMode == Anamorphic.Loose)
            {
                job.ParW = sourceParValues.Width;
                job.ParH = sourceParValues.Height;
            }

            return job;
        }

        #endregion
    }
}