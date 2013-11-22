﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="InteropModelCreator.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   A Utility Class to Convert a
// </summary>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrake.ApplicationServices.Utilities
{
    using System;
    using System.Collections.Generic;
    using System.Linq;

    using HandBrake.ApplicationServices.Model;
    using HandBrake.ApplicationServices.Model.Encoding;
    using HandBrake.Interop.Model;
    using HandBrake.Interop.Model.Encoding;
    using HandBrake.Interop.Model.Encoding.x264;

    /// <summary>
    /// A Utility Class to Convert a 
    /// </summary>
    public class InteropModelCreator
    {
        /// <summary>
        /// The get encode job.
        /// </summary>
        /// <param name="task">
        /// The task.
        /// </param>
        /// <returns>
        /// The <see cref="EncodeJob"/>.
        /// </returns>
        public static EncodeJob GetEncodeJob(QueueTask task)
        {
            // Sanity Checking
            if (task == null || task.Task == null)
            {
                return null;
            }

            return GetEncodeJob(task.Task);
        }

        /// <summary>
        /// Get an EncodeJob model for a LibHB Encode.
        /// </summary>
        /// <param name="task">
        /// The task.
        /// </param>
        /// <returns>
        /// An Interop.EncodeJob model.
        /// </returns>
        public static EncodeJob GetEncodeJob(EncodeTask task)
        {
            // The current Job Configuration
            EncodeTask work = task;

            // Which will be converted to this EncodeJob Model.
            EncodeJob job = new EncodeJob();
            EncodingProfile profile = new EncodingProfile();
            job.EncodingProfile = profile;

            // Audio Settings
            profile.AudioEncodings = new List<AudioEncoding>();
            job.ChosenAudioTracks = new List<int>();
            foreach (AudioTrack track in work.AudioTracks)
            {
                AudioEncoding newTrack = new AudioEncoding
                    {
                        Bitrate = track.Bitrate,
                        Drc = track.DRC,
                        Gain = track.Gain,
                        Encoder = Converters.GetCliAudioEncoder(track.Encoder),
                        InputNumber = track.Track.HasValue ? track.Track.Value : 0,
                        Mixdown = Converters.GetCliMixDown(track.MixDown),
                        SampleRateRaw = GetSampleRateRaw(track.SampleRate),
                    };

                profile.AudioEncodings.Add(newTrack);
                if (track.Track != null)
                {
                    job.ChosenAudioTracks.Add(track.Track.Value);
                }
            }

            // Title Settings
            job.OutputPath = work.Destination;
            job.SourcePath = work.Source;
            job.Title = work.Title;
            // job.SourceType = work.Type;
            switch (work.PointToPointMode)
            {
                case PointToPointMode.Chapters:
                    job.RangeType = VideoRangeType.Chapters;
                    break;
                case PointToPointMode.Seconds:
                    job.RangeType = VideoRangeType.Seconds;
                    break;
                case PointToPointMode.Frames:
                    job.RangeType = VideoRangeType.Frames;
                    break;
            }

            if (work.PointToPointMode == PointToPointMode.Seconds)
            {
                job.SecondsEnd = work.EndPoint;
                job.SecondsStart = work.StartPoint;
            }
            if (work.PointToPointMode == PointToPointMode.Chapters)
            {
                job.ChapterStart = work.StartPoint;
                job.ChapterEnd = work.EndPoint;
            }
            if (work.PointToPointMode == PointToPointMode.Frames)
            {
                job.FramesEnd = work.EndPoint;
                job.FramesStart = work.StartPoint;
            }

            job.Angle = work.Angle;
            job.EncodingProfile = profile;

            // Output Settings
            profile.IPod5GSupport = work.IPod5GSupport;
            profile.Optimize = work.OptimizeMP4;
            switch (work.OutputFormat)
            {
                case OutputFormat.Mp4:
                case OutputFormat.M4V:
                    profile.OutputFormat = Container.Mp4;
                    break;
                case OutputFormat.Mkv:
                    profile.OutputFormat = Container.Mkv;
                    break;
            }

            // Picture Settings
            profile.Anamorphic = work.Anamorphic;
            profile.Cropping = new Cropping
                {
                    Top = work.Cropping.Top,
                    Bottom = work.Cropping.Bottom,
                    Left = work.Cropping.Left,
                    Right = work.Cropping.Right
                };
            profile.CroppingType = CroppingType.Custom; // TODO deal with this better
            profile.DisplayWidth = work.DisplayWidth.HasValue
                           ? int.Parse(Math.Round(work.DisplayWidth.Value, 0).ToString())
                           : 0;
            profile.PixelAspectX = work.PixelAspectX;
            profile.PixelAspectY = work.PixelAspectY;
            profile.Height = work.Height.HasValue ? work.Height.Value : 0;
            profile.KeepDisplayAspect = work.KeepDisplayAspect;
            profile.MaxHeight = work.MaxHeight.HasValue ? work.MaxHeight.Value : 0;
            profile.MaxWidth = work.MaxWidth.HasValue ? work.MaxWidth.Value : 0;
            profile.Modulus = work.Modulus.HasValue ? work.Modulus.Value : 16;
            profile.UseDisplayWidth = true;
            profile.Width = work.Width.HasValue ? work.Width.Value : 0;

            // Filter Settings
            profile.CustomDecomb = work.CustomDecomb;
            profile.CustomDeinterlace = work.CustomDeinterlace;
            profile.CustomDenoise = work.CustomDenoise;
            profile.CustomDetelecine = work.CustomDetelecine;
            if (work.Deblock > 4)
                profile.Deblock = work.Deblock;
            profile.Decomb = work.Decomb;
            profile.Deinterlace = work.Deinterlace;
            profile.Denoise = work.Denoise;
            profile.Detelecine = work.Detelecine;
            profile.Grayscale = work.Grayscale;

            // Video Settings
            profile.Framerate = work.Framerate.HasValue ? work.Framerate.Value : 0;
            profile.ConstantFramerate = work.FramerateMode == FramerateMode.CFR;
            profile.Quality = work.Quality.HasValue ? work.Quality.Value : 0;
            profile.VideoBitrate = work.VideoBitrate.HasValue ? work.VideoBitrate.Value : 0;
            profile.VideoEncodeRateType = work.VideoEncodeRateType;
            profile.VideoEncoder = Converters.GetVideoEncoder(work.VideoEncoder);

            profile.H264Level = work.H264Level;
            profile.X264Profile = work.H264Profile.ToString().ToLower().Replace(" ", string.Empty); // TODO change these away from strings.
            profile.X264Preset = work.X264Preset.ToString().ToLower().Replace(" ", string.Empty);
            profile.X264Tunes = new List<string>();

            if (work.X264Tune != x264Tune.None)
            {
                profile.X264Tunes.Add(work.X264Tune.ToString().ToLower().Replace(" ", string.Empty));
            }

            if (work.FastDecode)
            {
                profile.X264Tunes.Add("fastdecode");
            }

            // Chapter Markers
            profile.IncludeChapterMarkers = work.IncludeChapterMarkers;
            job.CustomChapterNames = work.ChapterNames.Select(item => item.ChapterName).ToList();
            job.UseDefaultChapterNames = work.IncludeChapterMarkers;

            // Advanced Settings
            profile.X264Options = work.AdvancedEncoderOptions;

            // Subtitles
            job.Subtitles = new Subtitles { SourceSubtitles = new List<SourceSubtitle>(), SrtSubtitles = new List<SrtSubtitle>() };
            foreach (SubtitleTrack track in work.SubtitleTracks)
            {
                if (track.IsSrtSubtitle)
                {
                    job.Subtitles.SrtSubtitles.Add(
                        new SrtSubtitle
                            {
                                CharacterCode = track.SrtCharCode,
                                Default = track.Default,
                                FileName = track.SrtFileName,
                                LanguageCode = track.SrtLang,
                                Offset = track.SrtOffset
                            });
                }
                else
                {
                    if (track.SourceTrack != null)
                    {
                        job.Subtitles.SourceSubtitles.Add(
                            new SourceSubtitle
                                {
                                    BurnedIn = track.Burned,
                                    Default = track.Default,
                                    Forced = track.Forced,
                                    TrackNumber = track.SourceTrack.TrackNumber
                                });
                    }
                }
            }

            return job;
        }

        /// <summary>
        /// Get the Raw Sample Rate
        /// </summary>
        /// <param name="rate">
        /// The rate.
        /// </param>
        /// <returns>
        /// The Raw sample rate as an int
        /// </returns>
        private static int GetSampleRateRaw(double rate)
        {
            if (rate == 22.05)
                return 22050;
            else if (rate == 24)
                return 24000;
            else if (rate == 44.1)
                return 32000;
            else if (rate == 48)
                return 48000;
            else return 48000;
        }
    }
}
