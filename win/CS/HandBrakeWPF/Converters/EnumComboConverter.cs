﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="EnumComboConverter.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   Defines the EnumComboConverter type.
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrakeWPF.Converters
{
    using System.Collections.Generic;
    using System.Globalization;
    using System.Windows.Data;
    using System;

    using HandBrake.ApplicationServices.Model;
    using HandBrake.ApplicationServices.Utilities;
    using HandBrake.Interop.Model.Encoding;
    using HandBrake.Interop.Model.Encoding.x264;

    using HandBrakeWPF.Model;

    /// <summary>
    /// Enum Combo Converter
    /// </summary>
    public sealed class EnumComboConverter : IValueConverter
    {
        /// <summary>
        /// Convert an Enum to it's display value (attribute)
        /// </summary>
        /// <param name="value">
        /// The value.
        /// </param>
        /// <param name="targetType">
        /// The target type.
        /// </param>
        /// <param name="parameter">
        /// The parameter. (A boolean which inverts the output)
        /// </param>
        /// <param name="culture">
        /// The culture.
        /// </param>
        /// <returns>
        /// Visibility property
        /// </returns>
        public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
        {
            // Lists
            if (value is IEnumerable<x264Preset>)
            {
                return EnumHelper<x264Preset>.GetEnumDisplayValues(typeof(x264Preset));
            }
            if (value is IEnumerable<x264Profile>)
            {
                return EnumHelper<x264Profile>.GetEnumDisplayValues(typeof(x264Profile));
            }
            if (value is IEnumerable<x264Tune>)
            {
                return EnumHelper<x264Tune>.GetEnumDisplayValues(typeof(x264Tune));
            }
            if (value is IEnumerable<VideoEncoder>)
            {
                return EnumHelper<VideoEncoder>.GetEnumDisplayValues(typeof(VideoEncoder));
            }
            if (value is IEnumerable<Mixdown>)
            {
                return EnumHelper<Mixdown>.GetEnumDisplayValues(typeof(Mixdown));
            }
            if (targetType == typeof(QsvPreset) || value.GetType() == typeof(QsvPreset))
            {
                return EnumHelper<QsvPreset>.GetDisplay((QsvPreset)value);
            }
 
            if (value is IEnumerable<PresetPictureSettingsMode>)
            {
                return EnumHelper<PresetPictureSettingsMode>.GetEnumDisplayValues(typeof(PresetPictureSettingsMode));
            }
            if (value is IEnumerable<Decomb>)
            {
                return EnumHelper<Decomb>.GetEnumDisplayValues(typeof(Decomb));
            }
            if (value is IEnumerable<Deinterlace>)
            {
                return EnumHelper<Deinterlace>.GetEnumDisplayValues(typeof(Deinterlace));
            }
            if (value is IEnumerable<Detelecine>)
            {
                return EnumHelper<Detelecine>.GetEnumDisplayValues(typeof(Detelecine));
            }
            if (value is IEnumerable<Denoise>)
            {
                return EnumHelper<Denoise>.GetEnumDisplayValues(typeof(Denoise));
            }

            if (value is IEnumerable<VideoScaler>)
            {
                return EnumHelper<VideoScaler>.GetEnumDisplayValues(typeof(VideoScaler));
            }

            // Single Items
            if (targetType == typeof(x264Preset) || value.GetType() == typeof(x264Preset))
            {
                return EnumHelper<x264Preset>.GetDisplay((x264Preset)value);
            }
            if (targetType == typeof(x264Profile) || value.GetType() == typeof(x264Profile))
            {
                return EnumHelper<x264Profile>.GetDisplay((x264Profile)value);
            }
            if (targetType == typeof(x264Tune) || value.GetType() == typeof(x264Tune))
            {
                return EnumHelper<x264Tune>.GetDisplay((x264Tune)value);
            }
            if (targetType == typeof(VideoEncoder) || value.GetType() == typeof(VideoEncoder))
            {
                return EnumHelper<VideoEncoder>.GetDisplay((VideoEncoder)value);
            }
            if (targetType == typeof(Mixdown) || value.GetType() == typeof(Mixdown))
            {
                return EnumHelper<Mixdown>.GetDisplay((Mixdown)value);
            }
  
            if (targetType == typeof(PresetPictureSettingsMode) || value.GetType() == typeof(PresetPictureSettingsMode))
            {
                return EnumHelper<PresetPictureSettingsMode>.GetDisplay((PresetPictureSettingsMode)value);
            }
            if (targetType == typeof(Deinterlace) || value.GetType() == typeof(Deinterlace))
            {
                return EnumHelper<Deinterlace>.GetDisplay((Deinterlace)value);
            }
            if (targetType == typeof(Detelecine) || value.GetType() == typeof(Detelecine))
            {
                return EnumHelper<Detelecine>.GetDisplay((Detelecine)value);
            }
            if (targetType == typeof(Decomb) || value.GetType() == typeof(Decomb))
            {
                return EnumHelper<Decomb>.GetDisplay((Decomb)value);
            }
            if (targetType == typeof(Denoise) || value.GetType() == typeof(Denoise))
            {
                return EnumHelper<Denoise>.GetDisplay((Denoise)value);
            }
            if (targetType == typeof(QueueItemStatus) || value.GetType() == typeof(QueueItemStatus))
            {
                return EnumHelper<QueueItemStatus>.GetDisplay((QueueItemStatus)value);
            }

            if (targetType == typeof(VideoScaler) || value.GetType() == typeof(VideoScaler))
            {
                return EnumHelper<VideoScaler>.GetDisplay((VideoScaler)value);
            }

            return null;
        }

        /// <summary>
        /// Convert Back for the IValueConverter Interface. 
        /// </summary>
        /// <param name="value">
        /// The value.
        /// </param>
        /// <param name="targetType">
        /// The target type.
        /// </param>
        /// <param name="parameter">
        /// The parameter.
        /// </param>
        /// <param name="culture">
        /// The culture.
        /// </param>
        /// <returns>
        /// Nothing
        /// </returns>
        /// <exception cref="NotImplementedException">
        /// This method is not used!
        /// </exception>
        public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture)
        {
            if (targetType == typeof(x264Preset) || value.GetType() == typeof(x264Preset))
            {
                return EnumHelper<x264Preset>.GetValue(value.ToString());
            }
            if (targetType == typeof(x264Profile) || value.GetType() == typeof(x264Profile))
            {
                return EnumHelper<x264Profile>.GetValue(value.ToString());
            }
            if (targetType == typeof(x264Tune) || value.GetType() == typeof(x264Tune))
            {
                return EnumHelper<x264Tune>.GetValue(value.ToString());
            }
            if (targetType == typeof(VideoEncoder) || value.GetType() == typeof(VideoEncoder))
            {
                return EnumHelper<VideoEncoder>.GetValue(value.ToString());
            }
            if (targetType == typeof(Mixdown) || value.GetType() == typeof(Mixdown))
            {
                return EnumHelper<Mixdown>.GetValue(value.ToString());
            }
            if (targetType == typeof(QsvPreset) || value.GetType() == typeof(QsvPreset))
            {
                return EnumHelper<QsvPreset>.GetValue(value.ToString());
            }
            if (targetType == typeof(PresetPictureSettingsMode) || value.GetType() == typeof(PresetPictureSettingsMode))
            {
                return EnumHelper<PresetPictureSettingsMode>.GetValue(value.ToString());
            }
            if (targetType == typeof(Denoise) || value.GetType() == typeof(Denoise))
            {
                return EnumHelper<Denoise>.GetValue(value.ToString());
            }
            if (targetType == typeof(Decomb) || value.GetType() == typeof(Decomb))
            {
                return EnumHelper<Decomb>.GetValue(value.ToString());
            }
            if (targetType == typeof(Deinterlace) || value.GetType() == typeof(Deinterlace))
            {
                return EnumHelper<Deinterlace>.GetValue(value.ToString());
            }
            if (targetType == typeof(Detelecine) || value.GetType() == typeof(Detelecine))
            {
                return EnumHelper<Detelecine>.GetValue(value.ToString());
            }
            if (targetType == typeof(VideoScaler) || value.GetType() == typeof(VideoScaler))
            {
                return EnumHelper<VideoScaler>.GetValue(value.ToString());
            }

            return null;
        }
    }
}
