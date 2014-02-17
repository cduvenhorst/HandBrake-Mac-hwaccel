﻿// --------------------------------------------------------------------------------------------------------------------
// <copyright file="hb_job_s.cs" company="HandBrake Project (http://handbrake.fr)">
//   This file is part of the HandBrake source code - It may be used under the terms of the GNU General Public License.
// </copyright>
// <summary>
//   Defines the hb_job_s type.
// </summary>
// <auto-generated> Disable Stylecop Warnings for this file  </auto-generated>
// --------------------------------------------------------------------------------------------------------------------

namespace HandBrake.Interop.HbLib
{
	using System;
	using System.Runtime.InteropServices;

	[StructLayout(LayoutKind.Sequential)]
	public struct hb_job_s
	{
		/// int
		public int sequence_id;

		/// hb_title_t*
		public IntPtr title;

		public int feature;

		/// int
		public int chapter_start;

		/// int
		public int chapter_end;

		/// int
		public int chapter_markers;

		/// int[4]
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4, ArraySubType = UnmanagedType.I4)]
		public int[] crop;

		/// int
		public int deinterlace;

		/// hb_list_t*
		public IntPtr list_filter;

		/// int
		public int width;

		/// int
		public int height;

		/// int
		public int keep_ratio;

		/// int
		public int grayscale;

		public hb_anamorphic_substruct anamorphic;

		public int modulus;

		/// int
		public int maxWidth;

		/// int
		public int maxHeight;

		/// int
		public int vcodec;

		/// float
		public float vquality;

		/// int
		public int vbitrate;

		/// int
		public int vrate;

		/// int
		public int vrate_base;

		/// int
		public int cfr;

		/// int
		public int pass;

		public int fastfirstpass;

        public IntPtr encoder_preset;

        public IntPtr encoder_tune;

        public IntPtr encoder_options;

        public IntPtr encoder_profile;

        public IntPtr encoder_level;

		/// int
		public int areBframes;

		public int color_matrix_code;

		public int color_prim;

		public int color_transfer;

		/// int
		public int color_matrix;

		public IntPtr list_chapter;

		/// hb_list_t*
		public IntPtr list_audio;

		public int acodec_copy_mask;
		public int acodec_fallback;

		/// hb_list_t*
		public IntPtr list_subtitle;

		public IntPtr list_attachment;

		public IntPtr metadata;

		/// int
		public int mux;

		/// char*
		/// UTF-8 encoded
		public IntPtr file;

		/// int
		public int largeFileSize;

		/// int
		public int mp4_optimize;

		/// int
		public int ipod_atom;

		/// int
		public int indepth_scan;

		/// hb_subtitle_config_t->hb_subtitle_config_s
		public hb_subtitle_config_s select_subtitle_config;

		/// int
		public int angle;

		public int frame_to_start;

		public long pts_to_start;

		/// int
		public int frame_to_stop;

		/// int64_t->int
		public long pts_to_stop;

		/// int
		public int start_at_preview;

		/// int
		public int seek_points;

		/// uint32_t->unsigned int
		public uint frames_to_skip;

		public int use_opencl;

		public int use_hwd;

		public int use_decomb;

		public int use_detelecine;

		public qsv_s qsv;

		// Padding for the part of the struct we don't care about marshaling.
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = MarshalingConstants.JobPaddingBytes, ArraySubType = UnmanagedType.U1)]
		public byte[] padding;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct hb_anamorphic_substruct
	{
		/// int
		public int mode;

		/// int
		public int itu_par;

		/// int
		public int par_width;

		/// int
		public int par_height;

		/// int
		public int dar_width;

		/// int
		public int dar_height;

		/// int
		public int keep_display_aspect;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct qsv_enc_info_s
	{
		public int pic_struct;
		public int align_width;
		public int align_height;
		public int is_init_done;
	}

	[StructLayout(LayoutKind.Sequential)]
	public struct qsv_s
	{
		public int decode;

		public int async_depth;

		/// const char*
		public IntPtr preset;

		/// av_qsv_context* 
		public IntPtr ctx;

		public qsv_enc_info_s enc_info;
	} 
}
