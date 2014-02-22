﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="EncodingProfile.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   Defines the EncodingProfile type.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrake.Interop.Model.Encoding
{
    using System.Collections.Generic;

	using HandBrake.Interop.Model;

    /// <summary>
    /// The encoding profile.
    /// </summary>
    public class EncodingProfile
	{
        /// <summary>
        /// Initializes a new instance of the <see cref="EncodingProfile"/> class.
        /// </summary>
        public EncodingProfile()
		{
			this.Cropping = new Cropping();
		}

        #region Destination and Output Settings

        /// <summary>
        /// Gets or sets the container name.
        /// </summary>
        public string ContainerName { get; set; }

        /// <summary>
        /// Gets or sets the preferred extension.
        /// </summary>
        public OutputExtension PreferredExtension { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether include chapter markers.
        /// </summary>
        public bool IncludeChapterMarkers { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether large file.
        /// </summary>
        public bool LargeFile { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether optimize.
        /// </summary>
        public bool Optimize { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether i pod 5 g support.
        /// </summary>
        public bool IPod5GSupport { get; set; }
        #endregion

        #region Picture Settings

        /// <summary>
        /// Gets or sets the width.
        /// </summary>
        public int Width { get; set; }

        /// <summary>
        /// Gets or sets the height.
        /// </summary>
        public int Height { get; set; }

        /// <summary>
        /// Gets or sets the max width.
        /// </summary>
        public int MaxWidth { get; set; }

        /// <summary>
        /// Gets or sets the max height.
        /// </summary>
        public int MaxHeight { get; set; }

        /// <summary>
        /// Gets or sets the scale method.
        /// </summary>
        public ScaleMethod ScaleMethod { get; set; }

        /// <summary>
        /// Gets or sets the cropping type.
        /// </summary>
        public CroppingType CroppingType { get; set; }

        /// <summary>
        /// Gets or sets the cropping.
        /// </summary>
        public Cropping Cropping { get; set; }

        /// <summary>
        /// Gets or sets the anamorphic.
        /// </summary>
        public Anamorphic Anamorphic { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether use display width.
        /// </summary>
        public bool UseDisplayWidth { get; set; }

        /// <summary>
        /// Gets or sets the display width.
        /// </summary>
        public int DisplayWidth { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether keep display aspect.
        /// </summary>
        public bool KeepDisplayAspect { get; set; }

        /// <summary>
        /// Gets or sets the pixel aspect x.
        /// </summary>
        public int PixelAspectX { get; set; }

        /// <summary>
        /// Gets or sets the pixel aspect y.
        /// </summary>
        public int PixelAspectY { get; set; }

        /// <summary>
        /// Gets or sets the modulus.
        /// </summary>
        public int Modulus { get; set; }
        #endregion

        #region Filters

        /// <summary>
        /// Gets or sets the deinterlace.
        /// </summary>
        public Deinterlace Deinterlace { get; set; }

        /// <summary>
        /// Gets or sets the custom deinterlace.
        /// </summary>
        public string CustomDeinterlace { get; set; }

        /// <summary>
        /// Gets or sets the decomb.
        /// </summary>
        public Decomb Decomb { get; set; }

        /// <summary>
        /// Gets or sets the custom decomb.
        /// </summary>
        public string CustomDecomb { get; set; }

        /// <summary>
        /// Gets or sets the detelecine.
        /// </summary>
        public Detelecine Detelecine { get; set; }

        /// <summary>
        /// Gets or sets the custom detelecine.
        /// </summary>
        public string CustomDetelecine { get; set; }

        /// <summary>
        /// Gets or sets the denoise.
        /// </summary>
        public Denoise Denoise { get; set; }

        /// <summary>
        /// Gets or sets the custom denoise.
        /// </summary>
        public string CustomDenoise { get; set; }

        /// <summary>
        /// Gets or sets the deblock.
        /// </summary>
        public int Deblock { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether grayscale.
        /// </summary>
        public bool Grayscale { get; set; }
        #endregion

        #region Video

        /// <summary>
        /// Gets or sets the video encoder.
        /// </summary>
        public string VideoEncoder { get; set; }

        /// <summary>
        /// Gets or sets the x 264 options.
        /// </summary>
        public string X264Options { get; set; }

        /// <summary>
        /// Gets or sets the x 264 profile.
        /// </summary>
        public string X264Profile { get; set; }

        /// <summary>
        /// Gets or sets the x 264 preset.
        /// </summary>
        public string X264Preset { get; set; }

        /// <summary>
        /// Gets or sets the x 264 tunes.
        /// </summary>
        public List<string> X264Tunes { get; set; }

        /// <summary>
        /// Gets or sets the qsv preset.
        /// </summary>
        public string QsvPreset { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether qsv decode.
        /// </summary>
        public bool QsvDecode { get; set; }

        /// <summary>
        /// Gets or sets the h 264 level.
        /// </summary>
        public string H264Level { get; set; }

        /// <summary>
        /// Gets or sets the video encode rate type.
        /// </summary>
        public VideoEncodeRateType VideoEncodeRateType { get; set; }

        /// <summary>
        /// Gets or sets the quality.
        /// </summary>
        public double Quality { get; set; }

        /// <summary>
        /// Gets or sets the target size.
        /// </summary>
        public int TargetSize { get; set; }

        /// <summary>
        /// Gets or sets the video bitrate.
        /// </summary>
        public int VideoBitrate { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether two pass.
        /// </summary>
        public bool TwoPass { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether turbo first pass.
        /// </summary>
        public bool TurboFirstPass { get; set; }

        /// <summary>
        /// Gets or sets the framerate.
        /// </summary>
        public double Framerate { get; set; }

        /// <summary>
        /// Gets or sets a value indicating whether constant framerate.
        /// </summary>
        public bool ConstantFramerate { get; set; }
        #endregion

        #region Audio

        /// <summary>
        /// Gets or sets the audio encodings.
        /// </summary>
        public List<AudioEncoding> AudioEncodings { get; set; }

        /// <summary>
        /// Gets or sets the audio encoder fallback.
        /// </summary>
        public string AudioEncoderFallback { get; set; }
        #endregion

        /// <summary>
	    /// The clone.
	    /// </summary>
	    /// <returns>
	    /// The <see cref="EncodingProfile"/>.
	    /// </returns>
	    public EncodingProfile Clone()
		{
			var profile = new EncodingProfile
			{
				ContainerName = this.ContainerName,
				PreferredExtension = this.PreferredExtension,
				IncludeChapterMarkers = this.IncludeChapterMarkers,
				LargeFile = this.LargeFile,
				Optimize = this.Optimize,
				IPod5GSupport = this.IPod5GSupport,

				Width = this.Width,
				Height = this.Height,
				MaxWidth = this.MaxWidth,
				MaxHeight = this.MaxHeight,
				ScaleMethod = this.ScaleMethod,
				CroppingType = this.CroppingType,
				Cropping = this.Cropping.Clone(),
				Anamorphic = this.Anamorphic,
				UseDisplayWidth = this.UseDisplayWidth,
				DisplayWidth = this.DisplayWidth,
				KeepDisplayAspect = this.KeepDisplayAspect,
				PixelAspectX = this.PixelAspectX,
				PixelAspectY = this.PixelAspectY,
				Modulus = this.Modulus,

				Deinterlace = this.Deinterlace,
				CustomDeinterlace = this.CustomDeinterlace,
				Decomb = this.Decomb,
				CustomDecomb = this.CustomDecomb,
				Detelecine = this.Detelecine,
				CustomDetelecine = this.CustomDetelecine,
				Denoise = this.Denoise,
				CustomDenoise = this.CustomDenoise,
				Deblock = this.Deblock,
				Grayscale = this.Grayscale,

				VideoEncoder = this.VideoEncoder,
				X264Options = this.X264Options,
				X264Profile = this.X264Profile,
				X264Preset = this.X264Preset,
				X264Tunes = this.X264Tunes,
				QsvPreset = this.QsvPreset,
				QsvDecode = this.QsvDecode,
				H264Level = this.H264Level,
				VideoEncodeRateType = this.VideoEncodeRateType,
				Quality = this.Quality,
				TargetSize = this.TargetSize,
				VideoBitrate = this.VideoBitrate,
				TwoPass = this.TwoPass,
				TurboFirstPass = this.TurboFirstPass,
				Framerate = this.Framerate,
				ConstantFramerate = this.ConstantFramerate,

				AudioEncodings = new List<AudioEncoding>(this.AudioEncodings),
				AudioEncoderFallback = this.AudioEncoderFallback
			};

			return profile;
		}
	}
}
