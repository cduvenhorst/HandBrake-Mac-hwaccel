﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="Preset.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   A Preset for encoding with.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrake.ApplicationServices.Model
{
    using Caliburn.Micro;

    using HandBrake.ApplicationServices.Model.Audio;
    using HandBrake.ApplicationServices.Model.Subtitle;

    /// <summary>
    /// A Preset for encoding with.
    /// </summary>
    public class Preset : PropertyChangedBase
    {
        #region Constants and Fields

        /// <summary>
        /// The is default.
        /// </summary>
        private bool isDefault;

        #endregion


        #region Properties

        /// <summary>
        /// Gets or sets the category which the preset resides under
        /// </summary>
        public string Category { get; set; }

        /// <summary>
        /// Gets or sets the Description for the preset
        /// </summary>
        public string Description { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether this is a built in preset
        /// </summary>
        public bool IsBuildIn { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether IsDefault.
        /// </summary>
        public bool IsDefault
        {
            get
            {
                return this.isDefault;
            }
            set
            {
                this.isDefault = value;
                this.NotifyOfPropertyChange(() => this.IsDefault);
            }
        }

        /// <summary>
        /// Gets or sets the preset name
        /// </summary>
        public string Name { get; set; }

        /// <summary>
        /// Gets or sets PictureSettingsMode.
        /// Source Maximum, Custom or None
        /// </summary>
        public PresetPictureSettingsMode PictureSettingsMode { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether use deinterlace.
        /// </summary>
        public bool UseDeinterlace { get; set; }

        /// <summary>
        /// Gets or sets task.
        /// </summary>
        public EncodeTask Task { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether Picture Filters are used with this preset.
        /// </summary>
        public bool UsePictureFilters { get; set; }

        /// <summary>
        /// Gets or sets The version number which associates this preset with a HB build
        /// </summary>
        public string Version { get; set; }

        /// <summary>
        /// Gets or sets the audio track behaviours.
        /// </summary>
        public AudioBehaviours AudioTrackBehaviours { get; set; }

        /// <summary>
        /// Gets or sets the subtitle track behaviours.
        /// </summary>
        public SubtitleBehaviours SubtitleTrackBehaviours { get; set; }

        #endregion

        #region Public Methods

        /// <summary>
        ///  Override the ToString Method
        /// </summary>
        /// <returns>
        /// The Preset Name
        /// </returns>
        public override string ToString()
        {
            return this.Name;
        }

        #endregion
    }
}