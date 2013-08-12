﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="OutputFormat.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   The Output format.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrake.ApplicationServices.Model.Encoding
{
    using System.ComponentModel;

    /// <summary>
    /// The Output format.
    /// </summary>
    public enum OutputFormat
    {
        [Description("MP4")]
        Mp4,

        [Description("M4V")]
        M4V,

        [Description("MKV")]
        Mkv,

        [Description("av_mkv")]
        av_mkv,

        [Description("av_mp4")]
        av_mp4,
    }
}
