/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * audiohandler.c
 * Copyright (C) John Stebbins 2008-2013 <stebbins@stebbins>
 *
 * audiohandler.c is free software.
 *
 * You may redistribute it and/or modify it under the terms of the
 * GNU General Public License, as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <glib/gi18n.h>
#include "ghbcompat.h"
#include "hb.h"
#include "settings.h"
#include "hb-backend.h"
#include "values.h"
#include "callbacks.h"
#include "preview.h"
#include "audiohandler.h"
#include "presets.h"

static void audio_add_to_settings(GValue *settings, GValue *asettings);
static void ghb_add_audio_to_ui(signal_user_data_t *ud, const GValue *settings);
static GValue* audio_get_selected_settings(signal_user_data_t *ud, int *index);
static void ghb_clear_audio_list_settings(GValue *settings);
static void ghb_clear_audio_list_ui(GtkBuilder *builder);

static gboolean block_updates = FALSE;

static void enable_quality_widget(signal_user_data_t *ud, int acodec)
{
    GtkWidget *widget1, *widget2;

    widget1 = GHB_WIDGET(ud->builder, "AudioTrackQualityEnable");
    widget2 = GHB_WIDGET(ud->builder, "AudioTrackBitrateEnable");
    if (hb_audio_quality_get_default(acodec) == HB_INVALID_AUDIO_QUALITY)
    {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(widget2), TRUE);
        gtk_widget_set_sensitive(widget1, FALSE);
    }
    else
    {
        gtk_widget_set_sensitive(widget1, TRUE);
    }
}

static void audio_deps(signal_user_data_t *ud, GValue *asettings, GtkWidget *widget)
{
    ghb_adjust_audio_rate_combos(ud);
    ghb_grey_combo_options (ud);
    if (widget != NULL)
        ghb_check_dependency(ud, widget, NULL);

    gint track = -1, encoder = 0;
    hb_audio_config_t *aconfig = NULL;
    gint titleindex = ghb_settings_combo_int(ud->settings, "title");

    if (asettings != NULL)
    {
        track = ghb_settings_get_int(asettings, "AudioTrack");
        encoder = ghb_settings_combo_int(asettings, "AudioEncoder");
        aconfig = ghb_get_scan_audio_info(titleindex, track);
    }

    gboolean is_passthru = (encoder & HB_ACODEC_PASS_FLAG);
    gboolean enable_drc = TRUE;
    if (aconfig != NULL)
    {
        enable_drc = hb_audio_can_apply_drc(aconfig->in.codec,
                                            aconfig->in.codec_param, encoder) &&
                     !is_passthru;
    }

    widget = GHB_WIDGET(ud->builder, "AudioTrackDRCSlider");
    gtk_widget_set_sensitive(widget, enable_drc);
    widget = GHB_WIDGET(ud->builder, "AudioTrackDRCSliderLabel");
    gtk_widget_set_sensitive(widget, enable_drc);
    widget = GHB_WIDGET(ud->builder, "AudioTrackDRCValue");
    gtk_widget_set_sensitive(widget, enable_drc);

    enable_quality_widget(ud, encoder);


    widget = GHB_WIDGET(ud->builder, "AudioBitrate");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioTrackQualityEnableBox");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioTrackQualityBox");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioMixdown");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioSamplerate");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioTrackGainSlider");
    gtk_widget_set_sensitive(widget, !is_passthru);
    widget = GHB_WIDGET(ud->builder, "AudioTrackGainValue");
    gtk_widget_set_sensitive(widget, !is_passthru);
}

gint
ghb_select_audio_codec(gint mux, hb_audio_config_t *aconfig, gint acodec, gint fallback, gint copy_mask)
{
    guint32 in_codec = aconfig != NULL ? aconfig->in.codec : 0;

    if (acodec == HB_ACODEC_AUTO_PASS)
    {
        return hb_autopassthru_get_encoder(in_codec, copy_mask, fallback, mux);
    }

    // Sanitize fallback
    const hb_encoder_t *enc;
    for (enc = hb_audio_encoder_get_next(NULL); enc != NULL;
         enc = hb_audio_encoder_get_next(enc))
    {
        if (enc->codec == fallback &&
            !(enc->muxers & mux))
        {
            if ( mux & HB_MUX_MASK_MKV )
                fallback = HB_ACODEC_LAME;
            else
                fallback = HB_ACODEC_FAAC;
            break;
        }
    }
    if ((acodec & HB_ACODEC_PASS_FLAG) &&
        !(acodec & in_codec & HB_ACODEC_PASS_MASK))
    {
        return fallback;
    }
    for (enc = hb_audio_encoder_get_next(NULL); enc != NULL;
         enc = hb_audio_encoder_get_next(enc))
    {
        if (enc->codec == acodec &&
            !(enc->muxers & mux))
        {
            return fallback;
        }
    }
    return acodec;
}

int ghb_get_copy_mask(GValue *settings)
{
    gint mask = 0;

    if (ghb_settings_get_boolean(settings, "AudioAllowMP3Pass"))
    {
        mask |= HB_ACODEC_MP3;
    }
    if (ghb_settings_get_boolean(settings, "AudioAllowAACPass"))
    {
        mask |= HB_ACODEC_FFAAC;
    }
    if (ghb_settings_get_boolean(settings, "AudioAllowAC3Pass"))
    {
        mask |= HB_ACODEC_AC3;
    }
    if (ghb_settings_get_boolean(settings, "AudioAllowDTSPass"))
    {
        mask |= HB_ACODEC_DCA;
    }
    if (ghb_settings_get_boolean(settings, "AudioAllowDTSHDPass"))
    {
        mask |= HB_ACODEC_DCA_HD;
    }
    return mask;
}

int ghb_select_fallback(GValue *settings, int acodec)
{
    gint fallback = 0;

    switch ( acodec )
    {
        case HB_ACODEC_MP3_PASS:
            return HB_ACODEC_LAME;

        case HB_ACODEC_AAC_PASS:
            return HB_ACODEC_FAAC;

        case HB_ACODEC_AC3_PASS:
            return HB_ACODEC_AC3;

        default:
        {
            int mux = ghb_settings_combo_int(settings, "FileFormat");
            fallback = ghb_settings_combo_int(settings, "AudioEncoderFallback");
            return hb_autopassthru_get_encoder(acodec, 0, fallback, mux);
        }
    }
}

void
audio_sanitize_settings(GValue *settings, GValue *asettings)
{
    gint titleindex, track, acodec, select_acodec, mix;
    hb_audio_config_t *aconfig;
    gint mux;
    gint bitrate;
    gint sr;

    g_debug("ghb_santiize_audio ()");
    mux = ghb_settings_combo_int(settings, "FileFormat");
    titleindex = ghb_settings_get_int(settings, "title_no");
    track = ghb_settings_get_int(asettings, "AudioTrack");
    acodec = ghb_settings_combo_int(asettings, "AudioEncoder");
    mix = ghb_settings_combo_int(asettings, "AudioMixdown");
    bitrate = ghb_settings_combo_int(asettings, "AudioBitrate");
    sr = ghb_settings_combo_int(asettings, "AudioSamplerate");

    aconfig = ghb_get_scan_audio_info(titleindex, track);
    if (sr == 0)
    {
        sr = aconfig ? aconfig->in.samplerate : 48000;
    }
    gint fallback = ghb_select_fallback(settings, acodec);
    gint copy_mask = ghb_get_copy_mask(settings);
    select_acodec = ghb_select_audio_codec(mux, aconfig, acodec,
                                           fallback, copy_mask);
    if (ghb_audio_is_passthru (select_acodec))
    {
        if (aconfig)
        {
            bitrate = aconfig->in.bitrate / 1000;

            // Set the values for bitrate and samplerate to the input rates
            mix = 0;
            ghb_settings_set_string(asettings, "AudioMixdown",
                ghb_lookup_combo_string("AudioMixdown", ghb_int_value(mix)));
            select_acodec &= aconfig->in.codec | HB_ACODEC_PASS_FLAG;
            ghb_settings_set_string(asettings, "AudioSamplerate",
                ghb_lookup_combo_string("AudioSamplerate", ghb_int_value(0)));
        }
        else
        {
            mix = 0;
            ghb_settings_set_string(asettings, "AudioMixdown",
                ghb_lookup_combo_string("AudioMixdown", ghb_int_value(mix)));
            ghb_settings_set_string(asettings, "AudioSamplerate",
                ghb_lookup_combo_string("AudioSamplerate", ghb_int_value(0)));
            bitrate = 448;
        }
        ghb_settings_set_double(asettings, "AudioTrackDRCSlider", 0.0);
    }
    else
    {
        if (mix == 0)
            mix = ghb_get_best_mix( aconfig, select_acodec, 0);
        bitrate = hb_audio_bitrate_get_best(select_acodec, bitrate, sr, mix);
        ghb_settings_set_string(asettings, "AudioMixdown",
            ghb_lookup_combo_string("AudioMixdown", ghb_int_value(mix)));
    }
    ghb_settings_set_string(asettings, "AudioBitrate",
        ghb_lookup_combo_string("AudioBitrate", ghb_int_value(bitrate)));

    ghb_settings_take_value(asettings, "AudioEncoder",
                            ghb_lookup_audio_encoder_value(select_acodec));
}

void
ghb_adjust_audio_rate_combos(signal_user_data_t *ud)
{
    gint titleindex, track, acodec, select_acodec, mix;
    hb_audio_config_t *aconfig;
    GtkWidget *widget;
    GValue *gval;
    gint mux;
    gint bitrate;
    gint sr = 48000;

    g_debug("ghb_adjust_audio_rate_combos ()");
    mux = ghb_settings_combo_int(ud->settings, "FileFormat");
    titleindex = ghb_settings_combo_int(ud->settings, "title");

    widget = GHB_WIDGET(ud->builder, "AudioTrack");
    gval = ghb_widget_value(widget);
    track = ghb_lookup_combo_int("AudioTrack", gval);
    ghb_value_free(gval);

    widget = GHB_WIDGET(ud->builder, "AudioEncoder");
    gval = ghb_widget_value(widget);
    acodec = ghb_lookup_combo_int("AudioEncoder", gval);
    ghb_value_free(gval);

    widget = GHB_WIDGET(ud->builder, "AudioMixdown");
    gval = ghb_widget_value(widget);
    mix = ghb_lookup_combo_int("AudioMixdown", gval);
    ghb_value_free(gval);

    widget = GHB_WIDGET(ud->builder, "AudioBitrate");
    gval = ghb_widget_value(widget);
    bitrate = ghb_lookup_combo_int("AudioBitrate", gval);
    ghb_value_free(gval);

    widget = GHB_WIDGET(ud->builder, "AudioSamplerate");
    gval = ghb_widget_value(widget);
    sr = ghb_lookup_combo_int("AudioSamplerate", gval);
    ghb_value_free(gval);

    aconfig = ghb_get_scan_audio_info(titleindex, track);
    if (sr == 0)
    {
        sr = aconfig ? aconfig->in.samplerate : 48000;
    }
    gint fallback = ghb_select_fallback(ud->settings, acodec);
    gint copy_mask = ghb_get_copy_mask(ud->settings);
    select_acodec = ghb_select_audio_codec(mux, aconfig, acodec, fallback, copy_mask);
    gboolean codec_defined_bitrate = FALSE;
    if (ghb_audio_is_passthru (select_acodec))
    {
        if (aconfig)
        {
            bitrate = aconfig->in.bitrate / 1000;

            // Set the values for bitrate and samplerate to the input rates
            ghb_set_bitrate_opts (ud->builder, bitrate, bitrate, bitrate);
            mix = 0;
            ghb_ui_update(ud, "AudioMixdown", ghb_int64_value(mix));
            select_acodec &= aconfig->in.codec | HB_ACODEC_PASS_FLAG;
            codec_defined_bitrate = TRUE;
            ghb_ui_update(ud, "AudioSamplerate", ghb_int64_value(0));
        }
        else
        {
            ghb_ui_update(ud, "AudioSamplerate", ghb_int64_value(0));
            mix = 0;
            ghb_ui_update(ud, "AudioMixdown", ghb_int64_value(mix));
            bitrate = 448;
        }
        ghb_ui_update(ud, "AudioTrackDRCSlider", ghb_double_value(0));
    }
    else
    {
        if (mix == 0)
            mix = ghb_get_best_mix( aconfig, select_acodec, 0);
        bitrate = hb_audio_bitrate_get_best(select_acodec, bitrate, sr, mix);
        ghb_ui_update(ud, "AudioMixdown", ghb_int64_value(mix));
    }
    if (!codec_defined_bitrate)
    {
        int low, high;
        mix = ghb_get_best_mix( aconfig, select_acodec, mix);
        hb_audio_bitrate_get_limits(select_acodec, sr, mix, &low, &high);
        ghb_set_bitrate_opts (ud->builder, low, high, -1);
    }
    ghb_ui_update(ud, "AudioBitrate", ghb_int64_value(bitrate));

    ghb_settings_take_value(ud->settings, "AudioEncoder",
                            ghb_lookup_audio_encoder_value(select_acodec));
    GValue *asettings = audio_get_selected_settings(ud, NULL);
    if (asettings)
    {
        ghb_settings_take_value(asettings, "AudioEncoder",
                            ghb_lookup_audio_encoder_value(select_acodec));
    }
    ghb_audio_list_refresh_selected(ud);
}

static void
audio_update_dialog_widgets(signal_user_data_t *ud, GValue *asettings)
{
    if (asettings != NULL)
    {
        block_updates = TRUE;
        ghb_ui_update(ud, "AudioTrack",
                      ghb_settings_get_value(asettings, "AudioTrack"));
        ghb_ui_update(ud, "AudioEncoder",
                      ghb_settings_get_value(asettings, "AudioEncoder"));
        ghb_ui_update(ud, "AudioBitrate",
                      ghb_settings_get_value(asettings, "AudioBitrate"));
        ghb_ui_update(ud, "AudioTrackName",
                      ghb_settings_get_value(asettings, "AudioTrackName"));
        ghb_ui_update(ud, "AudioSamplerate",
                      ghb_settings_get_value(asettings, "AudioSamplerate"));
        ghb_ui_update(ud, "AudioMixdown",
                      ghb_settings_get_value(asettings, "AudioMixdown"));
        ghb_ui_update(ud, "AudioTrackDRCSlider",
                      ghb_settings_get_value(asettings, "AudioTrackDRCSlider"));
        ghb_ui_update(ud, "AudioTrackGainSlider",
                      ghb_settings_get_value(asettings, "AudioTrackGainSlider"));
        ghb_ui_update(ud, "AudioTrackQuality",
                      ghb_settings_get_value(asettings, "AudioTrackQuality"));
        ghb_ui_update(ud, "AudioTrackQualityEnable",
                  ghb_settings_get_value(asettings, "AudioTrackQualityEnable"));
        block_updates = FALSE;
    }
    audio_deps(ud, asettings, NULL);
}

static void
free_audio_hash_key_value(gpointer data)
{
    g_free(data);
}

const gchar*
ghb_get_user_audio_lang(GValue *settings, hb_title_t *title, gint track)
{
    GValue *audio_list, *asettings;
    const gchar *lang;

    audio_list = ghb_settings_get_value(settings, "audio_list");
    if (ghb_array_len(audio_list) <= track)
        return "und";
    asettings = ghb_array_get_nth(audio_list, track);
    track = ghb_settings_get_int(asettings, "AudioTrack");
    lang = ghb_get_source_audio_lang(title, track);
    return lang;
}

static gboolean*
get_track_used(gint settings, GHashTable *track_indices, gint count)
{
    gboolean *used;

    used = g_hash_table_lookup(track_indices, &settings);
    if (used == NULL)
    {
        gint *key;

        used = g_malloc0(count * sizeof(gboolean));
        key = g_malloc(sizeof(gint));
        *key = settings;
        g_hash_table_insert(track_indices, key, used);
    }
    return used;
}

static GValue*
audio_add_track(
    GValue *settings,
    hb_title_t *title,
    int track,
    int encoder,
    gboolean enable_quality,
    gdouble quality,
    int bitrate,
    int samplerate,
    int mix,
    gdouble drc,
    gdouble gain)
{
    GValue *asettings;
    hb_audio_config_t *aconfig = NULL;

    if (title != NULL)
    {
        aconfig = hb_list_audio_config_item(title->list_audio, track);
    }

    asettings = ghb_dict_value_new();

    ghb_settings_set_int(asettings, "AudioTrack", track);

    ghb_settings_set_string(asettings, "AudioEncoder",
        ghb_lookup_combo_string("AudioEncoder", ghb_int_value(encoder)));

    ghb_settings_set_boolean(asettings,
                             "AudioTrackQualityEnable", enable_quality);
    ghb_settings_set_double(asettings, "AudioTrackQuality", quality);

    ghb_settings_set_string(asettings, "AudioBitrate",
        ghb_lookup_combo_string("AudioBitrate", ghb_int_value(bitrate)));

    ghb_settings_set_string(asettings, "AudioSamplerate",
        ghb_lookup_combo_string("AudioSamplerate", ghb_int_value(samplerate)));

    if (aconfig != NULL)
    {
        mix = ghb_get_best_mix(aconfig, encoder, mix);
    }
    ghb_settings_set_string(asettings, "AudioMixdown",
        ghb_lookup_combo_string("AudioMixdown", ghb_int_value(mix)));

    ghb_settings_set_double(asettings, "AudioTrackDRCSlider", drc);

    ghb_settings_set_double(asettings, "AudioTrackGainSlider", gain);

    audio_sanitize_settings(settings, asettings);
    audio_add_to_settings(settings, asettings);

    return asettings;
}

static GValue*
audio_select_and_add_track(
    hb_title_t *title,
    GValue *settings,
    GValue *pref_audio,
    const char *lang,
    int pref_index,
    int start_track)
{
    GValue *audio, *asettings = NULL;
    gdouble drc, gain, quality;
    gboolean enable_quality;
    gint track, mux, acodec, bitrate, samplerate, mix;

    gint select_acodec;
    gint fallback;

    mux = ghb_settings_combo_int(settings, "FileFormat");
    gint copy_mask = ghb_get_copy_mask(settings);

    audio = ghb_array_get_nth(pref_audio, pref_index);

    acodec = ghb_settings_combo_int(audio, "AudioEncoder");
    fallback = ghb_select_fallback(settings, acodec);

    bitrate = ghb_settings_combo_int(audio, "AudioBitrate");
    samplerate = ghb_settings_combo_int(audio, "AudioSamplerate");
    mix = ghb_settings_combo_int(audio, "AudioMixdown");
    drc = ghb_settings_get_double(audio, "AudioTrackDRCSlider");
    gain = ghb_settings_get_double(audio, "AudioTrackGainSlider");
    enable_quality = ghb_settings_get_boolean(audio, "AudioTrackQualityEnable");
    quality = ghb_settings_get_double(audio, "AudioTrackQuality");

    track = ghb_find_audio_track(title, lang, start_track);
    if (track >= 0)
    {
        // Check to see if:
        // 1. pref codec is passthru
        // 2. source codec is not passthru
        // 3. next pref is enabled
        hb_audio_config_t *aconfig;
        aconfig = hb_list_audio_config_item(title->list_audio, track);
        select_acodec = ghb_select_audio_codec(
                            mux, aconfig, acodec, fallback, copy_mask);

        asettings = audio_add_track(settings, title, track, select_acodec,
                                    enable_quality, quality, bitrate,
                                    samplerate, mix, drc, gain);
    }
    return asettings;
}

static void set_pref_audio_with_lang(
    hb_title_t *title,
    GValue *settings,
    const char *lang,
    int behavior,
    gboolean secondary_audio_track_mode,
    GHashTable *track_used)
{
    const GValue *pref_audio, *audio_list;
    int count, ii, track, track_count, audio_count;
    gint mux;

    audio_list = ghb_settings_get_value(settings, "audio_list");

    mux = ghb_settings_combo_int(settings, "FileFormat");
    pref_audio = ghb_settings_get_value(settings, "AudioList");
    audio_count = hb_list_count(title->list_audio);
    count = ghb_array_len(pref_audio);
    int next_track = 0;
    track = ghb_find_audio_track(title, lang, next_track);
    while (track >= 0)
    {
        track_count = ghb_array_len(audio_list);
        count = (track_count && secondary_audio_track_mode) ? 1 : count;
        for (ii = 0; ii < count; ii++)
        {
            const GValue *audio;
            gint acodec, fallback, copy_mask, bitrate, samplerate, mix;
            gint select_acodec;
            gdouble drc, gain, quality;
            gboolean enable_quality;

            audio = ghb_array_get_nth(pref_audio, ii);
            acodec = ghb_settings_combo_int(audio, "AudioEncoder");
            fallback = ghb_select_fallback(settings, acodec);
            copy_mask = ghb_get_copy_mask(settings);
            bitrate = ghb_settings_combo_int(audio, "AudioBitrate");
            samplerate = ghb_settings_combo_int(audio, "AudioSamplerate");
            mix = ghb_settings_combo_int(audio, "AudioMixdown");
            drc = ghb_settings_get_double(audio, "AudioTrackDRCSlider");
            gain = ghb_settings_get_double(audio, "AudioTrackGainSlider");
            enable_quality = ghb_settings_get_boolean(audio,
                                                  "AudioTrackQualityEnable");
            quality = ghb_settings_get_double(audio, "AudioTrackQuality");

            // Check to see if:
            // 1. pref codec is passthru
            // 2. source codec is not passthru
            // 3. next pref is enabled
            hb_audio_config_t *aconfig;
            aconfig = hb_list_audio_config_item(title->list_audio, track);
            select_acodec = ghb_select_audio_codec(
                                mux, aconfig, acodec, fallback, copy_mask);

            // Was the source track already encoded
            // with the selected encode settings.
            gboolean *used = get_track_used(ii, track_used, audio_count);
            if (!used[track])
            {
                used[track] = TRUE;
                audio_add_track(settings, title, track, select_acodec,
                                enable_quality, quality, bitrate,
                                samplerate, mix, drc, gain);
            }
        }

        next_track = track + 1;
        if (behavior == 2)
        {
            track = ghb_find_audio_track(title, lang, next_track);
        }
        else
        {
            break;
        }
    }
}

void ghb_audio_title_change(signal_user_data_t *ud, gboolean title_valid)
{
    GtkWidget *w = GHB_WIDGET(ud->builder, "audio_add");
    gtk_widget_set_sensitive(w, title_valid);
    w = GHB_WIDGET(ud->builder, "audio_add_all");
    gtk_widget_set_sensitive(w, title_valid);
    w = GHB_WIDGET(ud->builder, "audio_reset");
    gtk_widget_set_sensitive(w, title_valid);
}

void
ghb_set_pref_audio_settings(gint titleindex, GValue *settings)
{
    GHashTable *track_used;
    hb_title_t *title;

    const GValue *lang_list;
    gint behavior;
    gint ii, audio_count, lang_count;

    g_debug("set_pref_audio");
    behavior = ghb_settings_combo_int(settings, "AudioTrackSelectionBehavior");

    // Clear the audio list
    ghb_clear_audio_list_settings(settings);

    title = ghb_get_title_info(titleindex);
    if (behavior == 0 || title == NULL)
    {
        // None or no source title
        return;
    }
    audio_count = hb_list_count(title->list_audio);
    if (audio_count == 0)
    {
        // No source audio
        return;
    }

    track_used = g_hash_table_new_full(g_int_hash, g_int_equal,
                        free_audio_hash_key_value, free_audio_hash_key_value);

    // Find "best" audio based on audio preset defaults
    lang_list = ghb_settings_get_value(settings, "AudioLanguageList");

    lang_count = ghb_array_len(lang_list);
    for (ii = 0; ii < lang_count; ii++)
    {
        const gchar *lang;
        gboolean mode;
        GValue *glang = ghb_array_get_nth(lang_list, ii);
        lang = g_value_get_string(glang);
        mode = ghb_settings_get_boolean(settings, "AudioSecondaryEncoderMode");
        set_pref_audio_with_lang(title, settings, lang, behavior, mode, track_used);
    }
    GValue *audio_list = ghb_settings_get_value(settings, "audio_list");
    if (audio_list == NULL || ghb_array_len(audio_list) == 0)
    {
        // No matching audio tracks found.  Add first track matching
        // any language.
        set_pref_audio_with_lang(title, settings, "und", 1, FALSE, track_used);
    }
    g_hash_table_destroy(track_used);
}

static GValue*
audio_get_selected_settings(signal_user_data_t *ud, int *index)
{
    GtkTreeView *tv;
    GtkTreePath *tp;
    GtkTreeSelection *ts;
    GtkTreeModel *tm;
    GtkTreeIter ti;
    gint *indices;
    gint row;
    GValue *asettings = NULL;
    const GValue *audio_list;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    ts = gtk_tree_view_get_selection (tv);
    if (gtk_tree_selection_get_selected(ts, &tm, &ti))
    {
        // Get the row number
        tp = gtk_tree_model_get_path (tm, &ti);
        indices = gtk_tree_path_get_indices (tp);
        row = indices[0];
        gtk_tree_path_free(tp);
        // find audio settings
        if (row < 0) return NULL;

        audio_list = ghb_settings_get_value(ud->settings, "audio_list");
        if (row >= ghb_array_len(audio_list))
            return NULL;

        asettings = ghb_array_get_nth(audio_list, row);
        if (index != NULL)
            *index = row;
    }
    return asettings;
}

static void
audio_refresh_list_row_ui(
    GtkTreeModel *tm,
    GtkTreeIter *ti,
    signal_user_data_t *ud,
    const GValue *settings)
{
    GtkTreeIter cti;
    char *info_src, *info_src_2;
    char *info_dst, *info_dst_2;


    info_src_2 = NULL;
    info_dst_2 = NULL;

    const gchar *s_track, *s_codec, *s_mix;
    gchar *s_drc, *s_gain, *s_br_quality, *s_sr, *s_track_name;
    gdouble drc, gain;
    hb_audio_config_t *aconfig;
    int titleindex, track, sr, codec;

    titleindex = ghb_settings_combo_int(ud->settings, "title");
    track = ghb_settings_get_int(settings, "AudioTrack");
    aconfig = ghb_get_scan_audio_info(titleindex, track);
    if (aconfig == NULL)
    {
        return;
    }


    s_track = ghb_settings_combo_option(settings, "AudioTrack");
    codec = ghb_settings_combo_int(settings, "AudioEncoder");
    s_codec = ghb_settings_combo_option(settings, "AudioEncoder");

    double quality = ghb_settings_get_double(settings, "AudioTrackQuality");
    if (ghb_settings_get_boolean(settings, "AudioTrackQualityEnable") &&
        quality != HB_INVALID_AUDIO_QUALITY)
    {
        s_br_quality = ghb_format_quality("Quality: ", codec, quality);
    }
    else
    {
        s_br_quality = g_strdup_printf("Bitrate: %skbps",
            ghb_settings_combo_option(settings, "AudioBitrate"));
    }

    sr = ghb_settings_combo_int(settings, "AudioSamplerate");
    if (sr == 0)
    {
        sr = aconfig->in.samplerate;
    }
    s_sr = g_strdup_printf("%.4gkHz", (double)sr/1000);

    s_mix = ghb_settings_combo_option(settings, "AudioMixdown");
    gain = ghb_settings_get_double(settings, "AudioTrackGainSlider");
    s_gain = g_strdup_printf("%ddB", (int)gain);

    drc = ghb_settings_get_double(settings, "AudioTrackDRCSlider");
    if (drc < 1.0)
        s_drc = g_strdup(_("Off"));
    else
        s_drc = g_strdup_printf("%.1f", drc);

    s_track_name = ghb_settings_get_string(settings, "AudioTrackName");

    info_src = g_strdup_printf("%s (%.4gkHz)",
        s_track,
        (double)aconfig->in.samplerate / 1000);
    if (aconfig->in.bitrate > 0)
    {
        info_src_2 = g_strdup_printf(
            "Bitrate: %.4gkbps",
            (double)aconfig->in.bitrate / 1000);
    }

    if (ghb_audio_is_passthru(codec))
    {
        info_dst = g_strdup_printf("Passthrough");
    }
    else
    {
        info_dst = g_strdup_printf("%s (%s) (%s)", s_codec, s_mix, s_sr);
        if (s_track_name && s_track_name[0])
        {
            info_dst_2 = g_strdup_printf(
                "%s\nGain: %s\nDRC: %s\nTrack Name: %s",
                s_br_quality, s_gain, s_drc, s_track_name);
        }
        else
        {
            info_dst_2 = g_strdup_printf("%s\nGain: %s\nDRC: %s",
                                            s_br_quality, s_gain, s_drc);
        }
    }
    gtk_tree_store_set(GTK_TREE_STORE(tm), ti,
        // These are displayed in list
        0, info_src,
        1, "-->",
        2, info_dst,
        3, "hb-edit",
        4, "hb-queue-delete",
        5, 0.5,
        -1);

    if (info_src_2 != NULL || info_dst_2 != NULL)
    {
        if (info_src_2 == NULL)
            info_src_2 = g_strdup("");
        if (info_dst_2 == NULL)
            info_dst_2 = g_strdup("");

        // Get the child of the selection
        if (!gtk_tree_model_iter_children(tm, &cti, ti))
        {
            gtk_tree_store_append(GTK_TREE_STORE(tm), &cti, ti);
        }
        gtk_tree_store_set(GTK_TREE_STORE(tm), &cti,
            // These are displayed in list
            0, info_src_2,
            2, info_dst_2,
            5, 0.0,
            -1);
    }
    else
    {
        if(gtk_tree_model_iter_children(tm, &cti, ti))
        {
            gtk_tree_store_remove(GTK_TREE_STORE(tm), &cti);
        }
    }

    g_free(info_src);
    g_free(info_src_2);
    g_free(info_dst);
    g_free(info_dst_2);

    g_free(s_sr);
    g_free(s_drc);
    g_free(s_gain);
    g_free(s_br_quality);
    g_free(s_track_name);
}

void
ghb_audio_list_refresh_selected(signal_user_data_t *ud)
{
    GtkTreeView *tv;
    GtkTreePath *tp;
    GtkTreeSelection *ts;
    GtkTreeModel *tm;
    GtkTreeIter ti;
    gint *indices;
    gint row;
    GValue *asettings = NULL;
    const GValue *audio_list;

    g_debug("ghb_audio_list_refresh_selected ()");
    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    ts = gtk_tree_view_get_selection (tv);
    if (gtk_tree_selection_get_selected(ts, &tm, &ti))
    {
        // Get the row number
        tp = gtk_tree_model_get_path (tm, &ti);
        indices = gtk_tree_path_get_indices (tp);
        row = indices[0];
        gtk_tree_path_free(tp);
        if (row < 0) return;

        audio_list = ghb_settings_get_value(ud->settings, "audio_list");
        if (row >= ghb_array_len(audio_list))
            return;

        asettings = ghb_array_get_nth(audio_list, row);
        audio_refresh_list_row_ui(tm, &ti, ud, asettings);
    }
}

static void
audio_refresh_list_ui(signal_user_data_t *ud)
{
    GValue *audio_list;
    GValue *asettings;
    gint ii, count, tm_count;
    GtkTreeView *tv;
    GtkTreeModel *tm;
    GtkTreeIter ti;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    tm = gtk_tree_view_get_model(tv);

    tm_count = gtk_tree_model_iter_n_children(tm, NULL);

    audio_list = ghb_settings_get_value(ud->settings, "audio_list");
    count = ghb_array_len(audio_list);
    if (count != tm_count)
    {
        ghb_clear_audio_list_ui(ud->builder);
        for (ii = 0; ii < count; ii++)
        {
            gtk_tree_store_append(GTK_TREE_STORE(tm), &ti, NULL);
        }
    }
    for (ii = 0; ii < count; ii++)
    {
        gtk_tree_model_iter_nth_child(tm, &ti, NULL, ii);
        asettings = ghb_array_get_nth(audio_list, ii);
        audio_refresh_list_row_ui(tm, &ti, ud, asettings);
    }
}

void
ghb_audio_list_refresh_all(signal_user_data_t *ud)
{
    audio_refresh_list_ui(ud);
}

G_MODULE_EXPORT void
audio_codec_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    static gint prev_acodec = 0;
    gint acodec_code;
    GValue *asettings;

    ghb_widget_to_setting(ud->settings, widget);
    acodec_code = ghb_settings_combo_int(ud->settings, "AudioEncoder");

    if (block_updates)
    {
        prev_acodec = acodec_code;
        return;
    }

    asettings = audio_get_selected_settings(ud, NULL);
    if (ghb_audio_is_passthru (prev_acodec) &&
        !ghb_audio_is_passthru (acodec_code))
    {
        // Transition from passthru to not, put some audio settings back to
        // pref settings
        gint titleindex;
        gint track;
        gint br, sr, mix_code;

        if (asettings != NULL)
        {
            br = ghb_settings_get_int(asettings, "AudioBitrate");
            sr = ghb_settings_combo_int(asettings, "AudioSamplerate");
            mix_code = ghb_settings_combo_int(asettings, "AudioMixdown");
        }
        else
        {
            br = 160;
            sr = 0;
            mix_code = 0;
        }

        titleindex = ghb_settings_combo_int(ud->settings, "title");
        track = ghb_settings_get_int(ud->settings, "AudioTrack");
        if (sr)
        {
            sr = ghb_find_closest_audio_samplerate(sr);
        }
        ghb_ui_update(ud, "AudioSamplerate", ghb_int64_value(sr));

        hb_audio_config_t *aconfig;
        aconfig = ghb_get_scan_audio_info(titleindex, track);
        if (sr == 0)
        {
            sr = aconfig ? aconfig->in.samplerate : 48000;
        }
        mix_code = ghb_get_best_mix( aconfig, acodec_code, mix_code);
        br = hb_audio_bitrate_get_best(acodec_code, br, sr, mix_code);
        ghb_ui_update(ud, "AudioBitrate", ghb_int64_value(br));

        ghb_ui_update(ud, "AudioMixdown", ghb_int64_value(mix_code));
    }
    prev_acodec = acodec_code;
    if (asettings != NULL)
    {
        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        ghb_live_reset(ud);
    }

    float low, high, gran, defval;
    int dir;
    hb_audio_quality_get_limits(acodec_code, &low, &high, &gran, &dir);
    defval = hb_audio_quality_get_default(acodec_code);
    GtkScaleButton *sb;
    GtkAdjustment *adj;
    sb = GTK_SCALE_BUTTON(GHB_WIDGET(ud->builder, "AudioTrackQuality"));
    adj = gtk_scale_button_get_adjustment(sb);
    if (dir)
    {
        // Quality values are inverted
        defval = high - defval + low;
    }
    gtk_adjustment_configure (adj, defval, low, high, gran, gran * 10, 0);
}

G_MODULE_EXPORT void
audio_track_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GValue *asettings;

    g_debug("audio_track_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    if (block_updates)
    {
        return;
    }

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        const gchar *track;

        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        track = ghb_settings_combo_option(asettings, "AudioTrack");
        ghb_settings_set_string(asettings, "AudioTrackDescription", track);
        ghb_live_reset(ud);
    }
}

G_MODULE_EXPORT void
audio_mix_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GValue *asettings;

    ghb_widget_to_setting(ud->settings, widget);
    if (block_updates)
    {
        return;
    }

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        ghb_live_reset(ud);
    }
}

G_MODULE_EXPORT void
audio_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GValue *asettings;

    ghb_widget_to_setting(ud->settings, widget);
    if (block_updates)
    {
        return;
    }

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        ghb_live_reset(ud);
    }
}

G_MODULE_EXPORT void
audio_quality_radio_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_check_dependency(ud, widget, NULL);
    audio_widget_changed_cb(widget, ud);
}

G_MODULE_EXPORT void
audio_passthru_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT gchar*
format_drc_cb(GtkScale *scale, gdouble val, signal_user_data_t *ud)
{
    if (val < 1.0)
        return g_strdup_printf("%-7s", _("Off"));
    else
        return g_strdup_printf("%-7.1f", val);
}

static inline int is_close(float a, float b, float metric)
{
    float diff = a - b;
    diff = (diff > 0) ? diff : -diff;
    return diff < metric;
}

char * ghb_format_quality( const char *prefix, int codec, double quality )
{
    float low, high, gran;
    int dir;
    hb_audio_quality_get_limits(codec, &low, &high, &gran, &dir);

    int digits = 0;
    float tmp = gran;
    while (1)
    {
        if (is_close(tmp, (int)tmp, gran / 10))
            break;
        digits++;
        tmp *= 10;
    }
    return g_strdup_printf("%s%.*f", prefix, digits, quality);
}

G_MODULE_EXPORT void
quality_widget_changed_cb(GtkWidget *widget, gdouble quality, signal_user_data_t *ud)
{
    GValue *asettings;

    g_debug("quality_widget_changed_cb ()");

    ghb_check_dependency(ud, widget, NULL);
    float low, high, gran;
    int dir;
    int codec = ghb_settings_combo_int(ud->settings, "AudioEncoder");
    hb_audio_quality_get_limits(codec, &low, &high, &gran, &dir);
    if (dir)
    {
        // Quality values are inverted
        quality = high - quality + low;
    }
    char *s_quality = ghb_format_quality("", codec, quality);
    ghb_ui_update( ud, "AudioTrackQualityValue", ghb_string_value(s_quality));
    g_free(s_quality);

    if (block_updates) return;

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        ghb_settings_set_double(asettings, "AudioTrackQuality", quality);
        ghb_audio_list_refresh_selected(ud);
    }
    ghb_live_reset(ud);
}

G_MODULE_EXPORT void
drc_widget_changed_cb(GtkWidget *widget, gdouble drc, signal_user_data_t *ud)
{
    GValue *asettings;

    g_debug("drc_widget_changed_cb ()");

    ghb_widget_to_setting(ud->settings, widget);
    if (block_updates)
    {
        return;
    }

    char *s_drc;
    if (drc < 0.99)
        s_drc = g_strdup(_("Off"));
    else
        s_drc = g_strdup_printf("%.1f", drc);
    ghb_ui_update( ud, "AudioTrackDRCValue", ghb_string_value(s_drc));
    g_free(s_drc);

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        ghb_live_reset(ud);
    }
}

G_MODULE_EXPORT gchar*
format_gain_cb(GtkScale *scale, gdouble val, signal_user_data_t *ud)
{
    if ( val >= 21.0 )
        return g_strdup_printf("*11*");
    return g_strdup_printf("%ddB", (int)val);
}

G_MODULE_EXPORT void
gain_widget_changed_cb(GtkWidget *widget, gdouble gain, signal_user_data_t *ud)
{
    GValue *asettings;

    g_debug("gain_widget_changed_cb ()");

    ghb_widget_to_setting(ud->settings, widget);
    if (block_updates)
    {
        return;
    }

    char *s_gain;
    if ( gain >= 21.0 )
        s_gain = g_strdup_printf("*11*");
    else
        s_gain = g_strdup_printf("%ddB", (int)gain);
    ghb_ui_update( ud, "AudioTrackGainValue", ghb_string_value(s_gain));
    g_free(s_gain);

    asettings = audio_get_selected_settings(ud, NULL);
    if (asettings != NULL)
    {
        ghb_widget_to_setting(asettings, widget);
        audio_deps(ud, asettings, widget);
        ghb_audio_list_refresh_selected(ud);
        ghb_live_reset(ud);
    }
}

void
ghb_clear_audio_list_settings(GValue *settings)
{
    GValue *audio_list;

    g_debug("clear_audio_list_settings ()");
    audio_list = ghb_settings_get_value(settings, "audio_list");
    if (audio_list == NULL)
    {
        audio_list = ghb_array_value_new(8);
        ghb_settings_set_value(settings, "audio_list", audio_list);
    }
    else
        ghb_array_value_reset(audio_list, 8);
}

void
ghb_clear_audio_list_ui(GtkBuilder *builder)
{
    GtkTreeView *tv;
    GtkTreeStore *ts;

    g_debug("clear_audio_list_ui ()");
    tv = GTK_TREE_VIEW(GHB_WIDGET(builder, "audio_list"));
    ts = GTK_TREE_STORE(gtk_tree_view_get_model(tv));
    gtk_tree_store_clear(ts);
}

static void
ghb_add_audio_to_ui(signal_user_data_t *ud, const GValue *asettings)
{
    GtkTreeView *tv;
    GtkTreeIter ti;
    GtkTreeModel *tm;
    GtkTreeSelection *ts;

    if (asettings == NULL)
        return;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    ts = gtk_tree_view_get_selection (tv);
    tm = gtk_tree_view_get_model(tv);

    gtk_tree_store_append(GTK_TREE_STORE(tm), &ti, NULL);
    gtk_tree_selection_select_iter(ts, &ti);

    audio_refresh_list_row_ui(tm, &ti, ud, asettings);
}

G_MODULE_EXPORT void
audio_list_selection_changed_cb(GtkTreeSelection *ts, signal_user_data_t *ud)
{
    GtkTreeModel *tm;
    GtkTreeIter ti;
    GValue *asettings = NULL;
    gint row;

    g_debug("audio_list_selection_changed_cb ()");
    if (gtk_tree_selection_get_selected(ts, &tm, &ti))
    {
        GtkTreeIter pti;

        if (gtk_tree_model_iter_parent(tm, &pti, &ti))
        {
            GtkTreePath *path;
            GtkTreeView *tv;

            gtk_tree_selection_select_iter(ts, &pti);
            path = gtk_tree_model_get_path(tm, &pti);
            tv = gtk_tree_selection_get_tree_view(ts);
            // Make the parent visible in scroll window if it is not.
            gtk_tree_view_scroll_to_cell(tv, path, NULL, FALSE, 0, 0);
            gtk_tree_path_free(path);
            return;
        }

        GtkTreePath *tp;
        gint *indices;
        GValue *audio_list;

        // Get the row number
        tp = gtk_tree_model_get_path (tm, &ti);
        indices = gtk_tree_path_get_indices (tp);
        row = indices[0];
        gtk_tree_path_free(tp);

        if (row < 0) return;
        audio_list = ghb_settings_get_value(ud->settings, "audio_list");
        if (row >= 0 && row < ghb_array_len(audio_list))
            asettings = ghb_array_get_nth(audio_list, row);
    }
    audio_update_dialog_widgets(ud, asettings);
}

static void
audio_add_to_settings(GValue *settings, GValue *asettings)
{
    GValue *audio_list;
    const gchar * track;

    audio_list = ghb_settings_get_value(settings, "audio_list");
    if (audio_list == NULL)
    {
        audio_list = ghb_array_value_new(8);
        ghb_settings_set_value(settings, "audio_list", audio_list);
    }

    int title_no = ghb_settings_get_int(settings, "title_no");
    int track_no = ghb_settings_get_int(asettings, "AudioTrack");
    track = ghb_audio_track_description(track_no, title_no);
    ghb_settings_set_string(asettings, "AudioTrackDescription", track);

    GValue *aname;
    aname = ghb_dict_lookup(asettings, "AudioTrackName");
    if (aname == NULL)
    {
        ghb_settings_set_string(asettings, "AudioTrackName", "");
    }
    ghb_array_append(audio_list, asettings);
}

G_MODULE_EXPORT void
audio_add_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    // Add the current audio settings to the list.
    GValue *asettings, *backup;

    // Back up settings in case we need to revert.
    backup = ghb_value_dup(
                ghb_settings_get_value(ud->settings, "audio_list"));

    int titleindex = ghb_settings_combo_int(ud->settings, "title");
    hb_title_t *title = ghb_get_title_info(titleindex);
    GValue *pref_audio = ghb_settings_get_value(ud->settings, "AudioList");

    asettings = audio_select_and_add_track(title, ud->settings, pref_audio,
                                           "und", 0, 0);
    ghb_add_audio_to_ui(ud, asettings);

    if (asettings != NULL)
    {
        // Pop up the edit dialog
        GtkResponseType response;
        GtkWidget *dialog = GHB_WIDGET(ud->builder, "audio_dialog");
        response = gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_hide(dialog);
        if (response != GTK_RESPONSE_OK)
        {
            ghb_settings_take_value(ud->settings, "audio_list", backup);
            asettings = audio_get_selected_settings(ud, NULL);
            if (asettings != NULL)
            {
                audio_update_dialog_widgets(ud, asettings);
            }
            audio_refresh_list_ui(ud);
        }
        else
        {
            ghb_value_free(backup);
        }
    }
}

G_MODULE_EXPORT void
audio_add_all_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    // Add the current audio settings to the list.

    ghb_clear_audio_list_settings(ud->settings);
    ghb_clear_audio_list_ui(ud->builder);

    int titleindex = ghb_settings_combo_int(ud->settings, "title");
    hb_title_t *title = ghb_get_title_info(titleindex);
    GValue *pref_audio = ghb_settings_get_value(ud->settings, "AudioList");

    int pref_count = ghb_array_len(pref_audio);

    int ii;
    for (ii = 0; ii < pref_count; ii++)
    {
        GValue *asettings;
        int track = 0;

        do
        {

            asettings = audio_select_and_add_track(title, ud->settings,
                                                   pref_audio, "und", ii,
                                                   track);
            if (asettings != NULL)
            {
                track = ghb_settings_get_int(asettings, "AudioTrack") + 1;
            }
        } while (asettings != NULL);
    }
    audio_refresh_list_ui(ud);
}

G_MODULE_EXPORT void
audio_edit_clicked_cb(GtkWidget *widget, gchar *path, signal_user_data_t *ud)
{
    GtkTreeView *tv;
    GtkTreePath *tp;
    GtkTreeModel *tm;
    GtkTreeSelection *ts;
    GtkTreeIter ti;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    ts = gtk_tree_view_get_selection(tv);
    tm = gtk_tree_view_get_model(tv);
    tp = gtk_tree_path_new_from_string (path);
    if (gtk_tree_path_get_depth(tp) > 1) return;
    if (gtk_tree_model_get_iter(tm, &ti, tp))
    {
        GValue *asettings, *backup;

        gtk_tree_selection_select_iter(ts, &ti);

        // Back up settings in case we need to revert.
        backup = ghb_value_dup(
                    ghb_settings_get_value(ud->settings, "audio_list"));

        // Pop up the edit dialog
        GtkResponseType response;
        GtkWidget *dialog = GHB_WIDGET(ud->builder, "audio_dialog");
        response = gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_hide(dialog);
        if (response != GTK_RESPONSE_OK)
        {
            ghb_settings_take_value(ud->settings, "audio_list", backup);
            asettings = audio_get_selected_settings(ud, NULL);
            if (asettings != NULL)
            {
                audio_update_dialog_widgets(ud, asettings);
            }
            audio_refresh_list_ui(ud);
        }
        else
        {
            ghb_value_free(backup);
        }
    }
}

G_MODULE_EXPORT void
audio_remove_clicked_cb(GtkWidget *widget, gchar *path, signal_user_data_t *ud)
{
    GtkTreeView *tv;
    GtkTreePath *tp;
    GtkTreeModel *tm;
    GtkTreeSelection *ts;
    GtkTreeIter ti, nextIter;
    gint row;
    gint *indices;
    GValue *audio_list;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "audio_list"));
    ts = gtk_tree_view_get_selection(tv);
    tm = gtk_tree_view_get_model(tv);
    tp = gtk_tree_path_new_from_string (path);
    if (gtk_tree_path_get_depth(tp) > 1) return;
    if (gtk_tree_model_get_iter(tm, &ti, tp))
    {
        nextIter = ti;
        if (!gtk_tree_model_iter_next(tm, &nextIter))
        {
            nextIter = ti;
            if (gtk_tree_model_get_iter_first(tm, &nextIter))
            {
                gtk_tree_selection_select_iter(ts, &nextIter);
            }
        }
        else
        {
            gtk_tree_selection_select_iter(ts, &nextIter);
        }

        audio_list = ghb_settings_get_value(ud->settings, "audio_list");

        // Get the row number
        indices = gtk_tree_path_get_indices (tp);
        row = indices[0];
        if (row < 0 || row >= ghb_array_len(audio_list))
        {
            gtk_tree_path_free(tp);
            return;
        }

        // Update our settings list before removing the row from the
        // treeview.  Removing from the treeview sometimes provokes an
        // immediate selection change, so the list needs to be up to date
        // when this happens.
        GValue *old = ghb_array_get_nth(audio_list, row);
        ghb_value_free(old);
        ghb_array_remove(audio_list, row);

        // Remove the selected item
        gtk_tree_store_remove(GTK_TREE_STORE(tm), &ti);

        ghb_live_reset(ud);
    }
    gtk_tree_path_free(tp);
}

G_MODULE_EXPORT void
audio_reset_clicked_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    int titleindex = ghb_settings_combo_int(ud->settings, "title");
    ghb_set_pref_audio_settings(titleindex, ud->settings);
    audio_refresh_list_ui(ud);
}

static GtkWidget *find_widget(GtkWidget *widget, gchar *name)
{
    const char *wname;
    GtkWidget *result = NULL;

    if (widget == NULL || name == NULL)
        return NULL;

    wname = gtk_widget_get_name(widget);
    if (wname != NULL && !strncmp(wname, name, 80))
    {
        return widget;
    }
    if (GTK_IS_CONTAINER(widget))
    {
        GList *list, *link;
        link = list = gtk_container_get_children(GTK_CONTAINER(widget));
        while (link)
        {
            result = find_widget(GTK_WIDGET(link->data), name);
            if (result != NULL)
                break;
            link = link->next;
        }
        g_list_free(list);
    }
    return result;
}

static void audio_def_update_widgets(GtkWidget *row, GValue *adict)
{
    GHashTableIter ti;
    gchar *key;
    GValue *gval;
    GtkWidget *widget;

    ghb_dict_iter_init(&ti, adict);

    block_updates = TRUE;
    while (g_hash_table_iter_next(&ti, (gpointer*)&key, (gpointer*)&gval))
    {
        widget = find_widget(row, key);
        if (widget != NULL)
        {
            ghb_update_widget(widget, gval);
        }
    }
    block_updates = FALSE;
}

static GtkListBoxRow*
audio_settings_get_row(GtkWidget *widget)
{
    while (widget != NULL && G_OBJECT_TYPE(widget) != GTK_TYPE_LIST_BOX_ROW)
    {
        widget = gtk_widget_get_parent(widget);
    }
    return GTK_LIST_BOX_ROW(widget);
}

static void audio_def_settings_bitrate_show(GtkWidget *widget, gboolean show)
{
    GtkWidget *bitrate_widget;
    GtkWidget *quality_widget;

    bitrate_widget = find_widget(widget, "AudioBitrate");
    quality_widget = find_widget(widget, "AudioTrackQualityBox");

    if (show)
    {
        gtk_widget_hide(quality_widget);
        gtk_widget_show(bitrate_widget);
    }
    else
    {
        gtk_widget_hide(bitrate_widget);
        gtk_widget_show(quality_widget);
    }
}

static void audio_def_settings_quality_set_sensitive(GtkWidget *w, gboolean s)
{
    GtkWidget *bitrate_widget;
    GtkWidget *quality_widget;

    bitrate_widget = find_widget(w, "AudioTrackBitrateEnable");
    quality_widget = find_widget(w, "AudioTrackQualityEnable");
    if (!s)
    {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(bitrate_widget), TRUE);
    }
    gtk_widget_set_sensitive(GTK_WIDGET(quality_widget), s);
}

static void audio_def_settings_show(GtkWidget *widget, gboolean show)
{
    GtkWidget *settings_box;
    GtkWidget *add_button;

    settings_box = find_widget(widget, "settings_box");
    add_button = find_widget(widget, "add_button");

    if (show)
    {
        gtk_widget_hide(add_button);
        gtk_widget_show(settings_box);
    }
    else
    {
        gtk_widget_hide(settings_box);
        gtk_widget_show(add_button);
    }
}

static void
audio_def_settings_init_row(GValue *adict, GtkWidget *row)
{
    GtkWidget *widget;

    widget = find_widget(row, "AudioEncoder");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioBitrate");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioMixdown");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioSamplerate");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioTrackGainSlider");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioTrackDRCSlider");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioTrackQuality");
    ghb_widget_to_setting(adict, widget);
    widget = find_widget(row, "AudioTrackQualityEnable");
    ghb_widget_to_setting(adict, widget);
}

G_MODULE_EXPORT void
audio_def_setting_add_cb(GtkWidget *w, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_setting_remove_cb(GtkWidget *w, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_encode_setting_changed_cb(GtkWidget *w, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_drc_changed_cb(GtkWidget *w, gdouble drc, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_gain_changed_cb(GtkWidget *w, gdouble gain, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_quality_changed_cb(GtkWidget *w, gdouble quality, signal_user_data_t *ud);
G_MODULE_EXPORT void
audio_def_quality_enable_changed_cb(GtkWidget *w, signal_user_data_t *ud);

GtkWidget * ghb_create_audio_settings_row(signal_user_data_t *ud)
{
    GtkBox *box, *box2, *box3;
    GtkComboBox *combo;
    GtkScaleButton *scale;
    GtkLabel *label;
    GtkRadioButton *radio;
    GtkButton *button;
    GtkImage *image;

    box = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));

    // Add Button
    button = GTK_BUTTON(gtk_button_new_with_label("Add"));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(button),
      "Add an audio encoder.\n"
      "Each selected source track will be encoded with all selected encoders.");
    gtk_widget_set_valign(GTK_WIDGET(button), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(button), "add_button");
    gtk_widget_hide(GTK_WIDGET(button));
    g_signal_connect(button, "clicked", (GCallback)audio_def_setting_add_cb, ud);
    gtk_box_pack_start(box, GTK_WIDGET(button), FALSE, FALSE, 0);

    // Hidden widgets box
    box2 = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));
    gtk_widget_set_name(GTK_WIDGET(box2), "settings_box");

    // Audio Encoder ComboBox
    combo = GTK_COMBO_BOX(gtk_combo_box_new());
    ghb_init_combo_box(combo);
    ghb_audio_encoder_opts_set(combo);
    // Init to first audio encoder
    const hb_encoder_t *aud_enc = hb_audio_encoder_get_next(NULL);
    ghb_update_widget(GTK_WIDGET(combo), ghb_int64_value(aud_enc->codec));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(combo),
        "Set the audio codec to encode this track with.");
    gtk_widget_set_valign(GTK_WIDGET(combo), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(combo), "AudioEncoder");
    gtk_widget_show(GTK_WIDGET(combo));
    g_signal_connect(combo, "changed", (GCallback)audio_def_encode_setting_changed_cb, ud);
    gtk_box_pack_start(box2, GTK_WIDGET(combo), FALSE, FALSE, 0);

    box3 = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));
    gtk_widget_set_name(GTK_WIDGET(box3), "br_q_box");
    gtk_widget_show(GTK_WIDGET(box3));

    // Bitrate vs Quality RadioButton
    GtkBox *vbox;
    vbox = GTK_BOX(gtk_box_new(GTK_ORIENTATION_VERTICAL, 0));
    radio = GTK_RADIO_BUTTON(gtk_radio_button_new_with_label(NULL, "Bitrate"));
    gtk_widget_set_name(GTK_WIDGET(radio), "AudioTrackBitrateEnable");
    gtk_widget_show(GTK_WIDGET(radio));
    gtk_box_pack_start(vbox, GTK_WIDGET(radio), FALSE, FALSE, 0);
    radio = GTK_RADIO_BUTTON(
                gtk_radio_button_new_with_label_from_widget(radio, "Quality"));
    gtk_widget_set_name(GTK_WIDGET(radio), "AudioTrackQualityEnable");
    g_signal_connect(radio, "toggled", (GCallback)audio_def_quality_enable_changed_cb, ud);
    gtk_widget_show(GTK_WIDGET(radio));
    gtk_box_pack_start(vbox, GTK_WIDGET(radio), FALSE, FALSE, 0);
    gtk_widget_show(GTK_WIDGET(vbox));
    gtk_box_pack_start(box3, GTK_WIDGET(vbox), FALSE, FALSE, 0);

    // Audio Bitrate ComboBox
    combo = GTK_COMBO_BOX(gtk_combo_box_new());
    ghb_init_combo_box(combo);
    ghb_audio_bitrate_opts_set(combo, FALSE);
    ghb_update_widget(GTK_WIDGET(combo), ghb_int64_value(192));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(combo),
        "Set the bitrate to encode this track with.");
    gtk_widget_set_valign(GTK_WIDGET(combo), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(combo), "AudioBitrate");
    gtk_widget_show(GTK_WIDGET(combo));
    g_signal_connect(combo, "changed", (GCallback)audio_def_encode_setting_changed_cb, ud);
    gtk_box_pack_start(box3, GTK_WIDGET(combo), FALSE, FALSE, 0);

    GtkBox *qbox;
    qbox = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));
    gtk_widget_set_name(GTK_WIDGET(qbox), "AudioTrackQualityBox");

    // Audio Quality ScaleButton
    const gchar *quality_icons[] = {
        "weather-storm",
        "weather-clear",
        "weather-storm",
        "weather-showers-scattered",
        "weather-showers",
        "weather-overcast",
        "weather-few-clouds",
        "weather-clear",
        NULL
    };
    scale = GTK_SCALE_BUTTON(gtk_scale_button_new(GTK_ICON_SIZE_BUTTON,
                                                  0, 10, 0.1, quality_icons));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(scale),
        "<b>Audio Quality:</b>\n"
        "For encoders that support it, adjust the quality of the output.");

    gtk_widget_set_valign(GTK_WIDGET(scale), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(scale), "AudioTrackQuality");
    gtk_widget_show(GTK_WIDGET(scale));
    g_signal_connect(scale, "value-changed", (GCallback)audio_def_quality_changed_cb, ud);
    gtk_box_pack_start(qbox, GTK_WIDGET(scale), FALSE, FALSE, 0);

    // Audio Quality Label
    label = GTK_LABEL(gtk_label_new("0.00"));
    gtk_label_set_width_chars(label, 4);
    gtk_misc_set_alignment(GTK_MISC(label), 0, 0.5);
    gtk_widget_set_name(GTK_WIDGET(label), "AudioTrackQualityValue");
    gtk_widget_show(GTK_WIDGET(label));
    gtk_box_pack_start(qbox, GTK_WIDGET(label), FALSE, FALSE, 0);
    gtk_widget_hide(GTK_WIDGET(qbox));
    gtk_box_pack_start(box3, GTK_WIDGET(qbox), FALSE, FALSE, 0);
    gtk_box_pack_start(box2, GTK_WIDGET(box3), FALSE, FALSE, 0);

    // Audio Mix ComboBox
    combo = GTK_COMBO_BOX(gtk_combo_box_new());
    ghb_init_combo_box(combo);
    ghb_mix_opts_set(combo);
    ghb_update_widget(GTK_WIDGET(combo), ghb_int64_value(HB_AMIXDOWN_5POINT1));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(combo),
        "Set the mixdown of the output audio track.");
    gtk_widget_set_valign(GTK_WIDGET(combo), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(combo), "AudioMixdown");
    gtk_widget_show(GTK_WIDGET(combo));
    g_signal_connect(combo, "changed", (GCallback)audio_def_encode_setting_changed_cb, ud);
    gtk_box_pack_start(box2, GTK_WIDGET(combo), FALSE, FALSE, 0);

    // Audio Sample Rate ComboBox
    combo = GTK_COMBO_BOX(gtk_combo_box_new());
    ghb_init_combo_box(combo);
    ghb_audio_samplerate_opts_set(combo);
    ghb_update_widget(GTK_WIDGET(combo), ghb_int64_value(0));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(combo),
        "Set the sample rate of the output audio track.");
    gtk_widget_set_valign(GTK_WIDGET(combo), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(combo), "AudioSamplerate");
    gtk_widget_show(GTK_WIDGET(combo));
    g_signal_connect(combo, "changed", (GCallback)audio_def_encode_setting_changed_cb, ud);
    gtk_box_pack_start(box2, GTK_WIDGET(combo), FALSE, FALSE, 0);

    box3 = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));
    gtk_widget_set_name(GTK_WIDGET(box3), "gain_box");
    gtk_widget_show(GTK_WIDGET(box3));

    // Audio Gain ScaleButton
    const gchar *gain_icons[] = {
        "audio-volume-muted",
        "audio-volume-high",
        "audio-volume-low",
        "audio-volume-medium",
        NULL
    };
    scale = GTK_SCALE_BUTTON(gtk_scale_button_new(GTK_ICON_SIZE_BUTTON,
                                                  -20, 21, 1, gain_icons));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(scale),
        "<b>Audio Gain:</b>\n"
        "Adjust the amplification or attenuation of the output audio track.");

    gtk_widget_set_valign(GTK_WIDGET(scale), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(scale), "AudioTrackGainSlider");
    gtk_widget_show(GTK_WIDGET(scale));
    g_signal_connect(scale, "value-changed", (GCallback)audio_def_gain_changed_cb, ud);
    gtk_box_pack_start(box3, GTK_WIDGET(scale), FALSE, FALSE, 0);

    // Audio Gain Label
    label = GTK_LABEL(gtk_label_new("0dB"));
    gtk_label_set_width_chars(label, 6);
    gtk_misc_set_alignment(GTK_MISC(label), 0, 0.5);
    gtk_widget_set_name(GTK_WIDGET(label), "AudioTrackGainValue");
    gtk_widget_show(GTK_WIDGET(label));
    gtk_box_pack_start(box3, GTK_WIDGET(label), FALSE, FALSE, 0);
    gtk_box_pack_start(box2, GTK_WIDGET(box3), FALSE, FALSE, 0);

    box3 = GTK_BOX(gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0));
    gtk_widget_set_name(GTK_WIDGET(box3), "drc_box");
    gtk_widget_show(GTK_WIDGET(box3));

    // Audio DRC ComboBox
    const gchar *drc_icons[] = {
        "audio-input-microphone",
        NULL
    };
    scale = GTK_SCALE_BUTTON(gtk_scale_button_new(GTK_ICON_SIZE_BUTTON,
                                                  0.9, 4, 0.1, drc_icons));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(scale),
        "<b>Dynamic Range Compression:</b>\n"
        "Adjust the dynamic range of the output audio track.\n"
        "For source audio that has a wide dynamic range,\n"
        "very loud and very soft sequences, DRC allows you\n"
        "to 'compress' the range by making loud sounds\n"
        "softer and soft sounds louder.\n");

    gtk_widget_set_valign(GTK_WIDGET(scale), GTK_ALIGN_CENTER);
    gtk_widget_set_name(GTK_WIDGET(scale), "AudioTrackDRCSlider");
    gtk_widget_show(GTK_WIDGET(scale));
    g_signal_connect(scale, "value-changed", (GCallback)audio_def_drc_changed_cb, ud);
    gtk_box_pack_start(box3, GTK_WIDGET(scale), FALSE, FALSE, 0);

    // Audio DRC Label
    label = GTK_LABEL(gtk_label_new("Off"));
    gtk_label_set_width_chars(label, 4);
    gtk_misc_set_alignment(GTK_MISC(label), 0, 0.5);
    gtk_widget_set_name(GTK_WIDGET(label), "AudioTrackDRCValue");
    gtk_widget_show(GTK_WIDGET(label));
    gtk_box_pack_start(box3, GTK_WIDGET(label), FALSE, FALSE, 0);
    gtk_box_pack_start(box2, GTK_WIDGET(box3), FALSE, FALSE, 0);

    // Remove button
    image = GTK_IMAGE(gtk_image_new_from_icon_name("hb-remove",
                                                   GTK_ICON_SIZE_BUTTON));
    button = GTK_BUTTON(gtk_button_new());
    gtk_button_set_image(button, GTK_WIDGET(image));
    gtk_widget_set_tooltip_markup(GTK_WIDGET(button),
      "Remove this audio encoder");
    gtk_button_set_relief(button, GTK_RELIEF_NONE);
    gtk_widget_set_valign(GTK_WIDGET(button), GTK_ALIGN_CENTER);
    gtk_widget_set_halign(GTK_WIDGET(button), GTK_ALIGN_END);
    gtk_widget_set_name(GTK_WIDGET(button), "remove_button");
    gtk_widget_show(GTK_WIDGET(button));
    g_signal_connect(button, "clicked", (GCallback)audio_def_setting_remove_cb, ud);
    gtk_box_pack_start(box2, GTK_WIDGET(button), TRUE, TRUE, 0);

    gtk_widget_show(GTK_WIDGET(box2));
    gtk_box_pack_start(box, GTK_WIDGET(box2), TRUE, TRUE, 0);

    gtk_widget_show(GTK_WIDGET(box));

    GtkWidget *widget;

    int width;
    widget = find_widget(GTK_WIDGET(box), "AudioEncoder");
    gtk_widget_get_preferred_width(widget, NULL, &width);

    widget = GHB_WIDGET(ud->builder, "audio_defaults_encoder_label");
    gtk_widget_set_size_request(widget, width, -1);
    widget = find_widget(GTK_WIDGET(box), "br_q_box");
    gtk_widget_get_preferred_width(widget, NULL, &width);
    widget = GHB_WIDGET(ud->builder, "audio_defaults_bitrate_label");
    gtk_widget_set_size_request(widget, width, -1);
    widget = find_widget(GTK_WIDGET(box), "AudioMixdown");
    gtk_widget_get_preferred_width(widget, NULL, &width);
    widget = GHB_WIDGET(ud->builder, "audio_defaults_mixdown_label");
    gtk_widget_set_size_request(widget, width, -1);
    widget = find_widget(GTK_WIDGET(box), "AudioSamplerate");
    gtk_widget_get_preferred_width(widget, NULL, &width);
    widget = GHB_WIDGET(ud->builder, "audio_defaults_samplerate_label");
    gtk_widget_set_size_request(widget, width, -1);
    widget = find_widget(GTK_WIDGET(box), "gain_box");
    gtk_widget_get_preferred_width(widget, NULL, &width);
    widget = GHB_WIDGET(ud->builder, "audio_defaults_gain_label");
    gtk_widget_set_size_request(widget, width, -1);
    widget = find_widget(GTK_WIDGET(box), "drc_box");
    gtk_widget_get_preferred_width(widget, NULL, &width);
    widget = GHB_WIDGET(ud->builder, "audio_defaults_drc_label");
    gtk_widget_set_size_request(widget, width, -1);

    return GTK_WIDGET(box);
}

static void
audio_def_setting_update(signal_user_data_t *ud, GtkWidget *widget)
{
    GtkListBoxRow *row = audio_settings_get_row(widget);
    gint index = gtk_list_box_row_get_index(row);

    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    int count = ghb_array_len(alist);
    if (!block_updates && index >= 0 && index < count)
    {
        GValue *adict = ghb_array_get_nth(alist, index);
        ghb_widget_to_setting(adict, widget);
    }
}

G_MODULE_EXPORT void
audio_add_lang_clicked_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GtkListBox *avail = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_avail_lang"));
    GtkListBox *selected = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_selected_lang"));
    GtkListBoxRow *row;
    GtkWidget *label;

    row = gtk_list_box_get_selected_row(avail);
    if (row != NULL)
    {
        int idx;
        const iso639_lang_t *lang;
        GValue *glang, *alang_list;

        // Remove from UI available language list box
        label = gtk_bin_get_child(GTK_BIN(row));
        g_object_ref(G_OBJECT(label));
        gtk_widget_destroy(GTK_WIDGET(row));
        gtk_widget_show(label);
        // Add to UI selected language list box
        gtk_list_box_insert(selected, label, -1);

        // Add to preset language list
        idx = (intptr_t)g_object_get_data(G_OBJECT(label), "lang_idx");
        lang = ghb_iso639_lookup_by_int(idx);
        glang = ghb_string_value_new(lang->iso639_2);
        alang_list = ghb_settings_get_value(ud->settings, "AudioLanguageList");
        ghb_array_append(alang_list, glang);
        ghb_clear_presets_selection(ud);
    }
}

G_MODULE_EXPORT void
audio_remove_lang_clicked_cb(GtkWidget *widget, signal_user_data_t *ud)
{

    GtkListBox *avail, *selected;
    GtkListBoxRow *row;
    GtkWidget *label;

    avail = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_avail_lang"));
    selected = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_selected_lang"));
    row = gtk_list_box_get_selected_row(selected);
    if (row != NULL)
    {
        gint index;
        GValue *alang_list;

        index = gtk_list_box_row_get_index(row);

        // Remove from UI selected language list box
        label = gtk_bin_get_child(GTK_BIN(row));
        g_object_ref(G_OBJECT(label));
        int idx = (intptr_t)g_object_get_data(G_OBJECT(label), "lang_idx");
        gtk_widget_destroy(GTK_WIDGET(row));
        gtk_widget_show(label);
        // Add to UI available language list box
        gtk_list_box_insert(avail, label, idx);

        // Remove from preset language list
        alang_list = ghb_settings_get_value(ud->settings, "AudioLanguageList");
        GValue *glang = ghb_array_get_nth(alang_list, index);
        ghb_array_remove(alang_list, index);
        ghb_value_free(glang);
        ghb_clear_presets_selection(ud);
    }
}

static void audio_quality_update_limits(GtkWidget *widget, int encoder)
{
    float low, high, gran, defval;
    int dir;

    hb_audio_quality_get_limits(encoder, &low, &high, &gran, &dir);
    defval = hb_audio_quality_get_default(encoder);
    GtkScaleButton *sb;
    GtkAdjustment *adj;
    sb = GTK_SCALE_BUTTON(widget);
    adj = gtk_scale_button_get_adjustment(sb);
    if (dir)
    {
        // Quality values are inverted
        defval = high - defval + low;
    }
    gtk_adjustment_configure (adj, defval, low, high, gran, gran * 10, 0);
}

void audio_def_set_limits(signal_user_data_t *ud, GtkWidget *widget)
{
    GtkListBoxRow *row = audio_settings_get_row(widget);
    gint index = gtk_list_box_row_get_index(row);

    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    int count = ghb_array_len(alist);
    if (index < 0 || index >= count)
        return;

    GValue *adict = ghb_array_get_nth(alist, index);

    int encoder = ghb_settings_combo_int(adict, "AudioEncoder");
    int fallback = ghb_settings_combo_int(ud->settings, "AudioEncoderFallback");
    // Allow quality settings if the current encoder supports quality
    // or if the encoder is auto-passthru and the fallback encoder
    // supports quality.
    gboolean sensitive =
        hb_audio_quality_get_default(encoder) != HB_INVALID_AUDIO_QUALITY ||
        (encoder == HB_ACODEC_AUTO_PASS &&
         hb_audio_quality_get_default(fallback) != HB_INVALID_AUDIO_QUALITY);
    audio_def_settings_quality_set_sensitive(GTK_WIDGET(row), sensitive);

    int enc;
    if (sensitive)
    {
        enc = encoder;
        if (hb_audio_quality_get_default(encoder) == HB_INVALID_AUDIO_QUALITY)
        {
            enc = fallback;
        }
        audio_quality_update_limits(find_widget(GTK_WIDGET(row),
                                                "AudioTrackQuality"), enc);
    }

    enc = encoder;
    if (enc & HB_ACODEC_PASS_FLAG)
    {
        enc = ghb_select_fallback(ud->settings, enc);
    }
    int sr = ghb_settings_combo_int(adict, "AudioSamplerate");
    if (sr == 0)
    {
        sr = 48000;
    }
    int mix = ghb_settings_combo_int(adict, "AudioMixdown");
    int low, high;
    hb_audio_bitrate_get_limits(enc, sr, mix, &low, &high);
    GtkWidget *w = find_widget(GTK_WIDGET(row), "AudioBitrate");
    ghb_audio_bitrate_opts_filter(GTK_COMBO_BOX(w), low, high);
    w = find_widget(GTK_WIDGET(row), "AudioMixdown");
    ghb_mix_opts_filter(GTK_COMBO_BOX(w), enc);
}

void audio_def_set_all_limits_cb(GtkWidget *widget, gpointer data)
{
    signal_user_data_t *ud = (signal_user_data_t*)data;
    audio_def_set_limits(ud, widget);
}

void audio_def_set_all_limits(signal_user_data_t *ud)
{
    GtkListBox *list_box;

    list_box = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_list_default"));
    gtk_container_foreach(GTK_CONTAINER(list_box),
                          audio_def_set_all_limits_cb, (gpointer)ud);
}

G_MODULE_EXPORT void
audio_def_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_fallback_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    audio_def_set_all_limits(ud);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_quality_enable_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    audio_def_setting_update(ud, widget);

    GtkListBoxRow *row = audio_settings_get_row(widget);
    gint index = gtk_list_box_row_get_index(row);

    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    GValue *adict = ghb_array_get_nth(alist, index);

    audio_def_settings_bitrate_show(GTK_WIDGET(row),
                !ghb_settings_get_boolean(adict, "AudioTrackQualityEnable"));
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_quality_changed_cb(GtkWidget *widget, gdouble quality, signal_user_data_t *ud)
{
    audio_def_setting_update(ud, widget);

    GtkListBoxRow *row = audio_settings_get_row(widget);
    GtkWidget *quality_label = find_widget(GTK_WIDGET(row),
                                           "AudioTrackQualityValue");
    gint index = gtk_list_box_row_get_index(row);

    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    GValue *adict = ghb_array_get_nth(alist, index);
    int codec = ghb_settings_combo_int(adict, "AudioEncoder");

    float low, high, gran;
    int dir;
    hb_audio_quality_get_limits(codec, &low, &high, &gran, &dir);
    if (dir)
    {
        // Quality values are inverted
        quality = high - quality + low;
    }
    char *s_quality = ghb_format_quality("", codec, quality);
    ghb_update_widget(quality_label, ghb_string_value(s_quality));
    g_free(s_quality);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_gain_changed_cb(GtkWidget *widget, gdouble gain, signal_user_data_t *ud)
{
    audio_def_setting_update(ud, widget);

    GtkListBoxRow *row = audio_settings_get_row(widget);
    GtkWidget *gain_label = find_widget(GTK_WIDGET(row), "AudioTrackGainValue");
    char *s_gain;
    if ( gain >= 21.0 )
        s_gain = g_strdup_printf("*11*");
    else
        s_gain = g_strdup_printf("%ddB", (int)gain);
    ghb_update_widget(gain_label, ghb_string_value(s_gain));
    g_free(s_gain);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_drc_changed_cb(GtkWidget *widget, gdouble drc, signal_user_data_t *ud)
{
    audio_def_setting_update(ud, widget);

    GtkListBoxRow *row = audio_settings_get_row(widget);
    GtkWidget *drc_label = find_widget(GTK_WIDGET(row), "AudioTrackDRCValue");

    char *s_drc;
    if (drc < 0.99)
        s_drc = g_strdup(_("Off"));
    else
        s_drc = g_strdup_printf("%.1f", drc);
    ghb_update_widget(drc_label, ghb_string_value(s_drc));
    g_free(s_drc);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_setting_add_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GtkListBoxRow *row = audio_settings_get_row(widget);

    GValue *adict;
    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    int count = ghb_array_len(alist);
    if (count > 0)
    {
        // Use first item in list as defaults for new entries.
        adict = ghb_value_dup(ghb_array_get_nth(alist, 0));
        audio_def_update_widgets(GTK_WIDGET(row), adict);
    }
    else
    {
        // Use hard coded defaults
        adict = ghb_dict_value_new();
        audio_def_settings_init_row(adict, GTK_WIDGET(row));
    }
    ghb_array_append(alist, adict);
    audio_def_settings_show(GTK_WIDGET(row), TRUE);

    // Add new "Add" button
    widget = ghb_create_audio_settings_row(ud);
    audio_def_settings_show(widget, FALSE);
    GtkListBox *list_box;
    list_box = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_list_default"));
    gtk_list_box_insert(list_box, widget, -1);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_setting_remove_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GtkListBoxRow *row = audio_settings_get_row(widget);
    gint index = gtk_list_box_row_get_index(row);

    GValue *alist = ghb_settings_get_value(ud->settings, "AudioList");
    int count = ghb_array_len(alist);
    if (index < 0 || index >= count)
    {
        return;
    }
    gtk_widget_destroy(GTK_WIDGET(row));
    GValue *asettings = ghb_array_get_nth(alist, index);
    ghb_array_remove(alist, index);
    ghb_value_free(asettings);
    ghb_clear_presets_selection(ud);
}

G_MODULE_EXPORT void
audio_def_encode_setting_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    audio_def_setting_update(ud, widget);
    audio_def_set_limits(ud, widget);
    ghb_clear_presets_selection(ud);
}

GtkListBoxRow* ghb_find_lang_row(GtkListBox *list_box, int lang_idx)
{
    GList *list, *link;
    GtkListBoxRow *result = NULL;

    list = link = gtk_container_get_children(GTK_CONTAINER(list_box));
    while (link != NULL)
    {
        GtkListBoxRow *row = (GtkListBoxRow*)link->data;
        GtkWidget *label = gtk_bin_get_child(GTK_BIN(row));
        int idx = (intptr_t)g_object_get_data(G_OBJECT(label), "lang_idx");
        if (idx == lang_idx)
        {
            result = row;
            break;
        }
        link = link->next;
    }
    g_list_free(list);

    return result;
}

static void audio_def_lang_list_clear_cb(GtkWidget *row, gpointer data)
{
    GtkListBox *avail = (GtkListBox*)data;
    GtkWidget *label = gtk_bin_get_child(GTK_BIN(row));
    g_object_ref(G_OBJECT(label));
    gtk_widget_destroy(GTK_WIDGET(row));
    gtk_widget_show(label);
    int idx = (intptr_t)g_object_get_data(G_OBJECT(label), "lang_idx");
    gtk_list_box_insert(avail, label, idx);
}

static void
audio_def_selected_lang_list_clear(signal_user_data_t *ud)
{
    GtkListBox *avail, *selected;
    avail = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_avail_lang"));
    selected = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_selected_lang"));
    gtk_container_foreach(GTK_CONTAINER(selected),
                          audio_def_lang_list_clear_cb, (gpointer)avail);
}

static void
audio_def_lang_list_init(signal_user_data_t *ud)
{
    GValue *lang_list;

    // Clear selected languages.
    audio_def_selected_lang_list_clear(ud);

    lang_list = ghb_settings_get_value(ud->settings, "AudioLanguageList");
    if (lang_list == NULL)
    {
        lang_list = ghb_array_value_new(8);
        ghb_settings_set_value(ud->settings, "AudioLanguageList", lang_list);
    }

    int ii, count;
    count = ghb_array_len(lang_list);
    for (ii = 0; ii < count; )
    {
        GValue *lang_val = ghb_array_get_nth(lang_list, ii);
        int idx = ghb_lookup_audio_lang(lang_val);

        GtkListBox *avail, *selected;
        GtkListBoxRow *row;
        avail = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_avail_lang"));
        selected = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_selected_lang"));
        row = ghb_find_lang_row(avail, idx);
        if (row)
        {
            GtkWidget *label = gtk_bin_get_child(GTK_BIN(row));
            g_object_ref(G_OBJECT(label));
            gtk_widget_destroy(GTK_WIDGET(row));
            gtk_widget_show(label);
            gtk_list_box_insert(selected, label, -1);
            ii++;
        }
        else
        {
            // Error in list.  Probably duplicate languages.  Remove
            // this item from the list.
            GValue *glang = ghb_array_get_nth(lang_list, ii);
            ghb_array_remove(lang_list, ii);
            ghb_value_free(glang);
            count--;
        }
    }
}

void ghb_audio_defaults_to_ui(signal_user_data_t *ud)
{
    GtkListBox *list_box;
    GValue *alist;
    int count, ii;

    audio_def_lang_list_init(ud);

    // Init the AudioList settings if necessary
    alist = ghb_settings_get_value(ud->settings, "AudioList");
    if (alist == NULL)
    {
        alist = ghb_array_value_new(8);
        ghb_settings_set_value(ud->settings, "AudioList", alist);
    }

    // Empty the current list
    list_box = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_list_default"));
    ghb_container_empty(GTK_CONTAINER(list_box));

    GtkWidget *widget;
    // Populate with new values
    count = ghb_array_len(alist);
    for (ii = 0; ii < count; ii++)
    {
        GValue *adict;

        adict = ghb_array_get_nth(alist, ii);
        widget = ghb_create_audio_settings_row(ud);
        gtk_list_box_insert(list_box, widget, -1);
        audio_def_update_widgets(widget, adict);
    }
    // Add row with "Add" button
    widget = ghb_create_audio_settings_row(ud);
    audio_def_settings_show(widget, FALSE);
    gtk_list_box_insert(list_box, widget, -1);
}

void ghb_init_audio_defaults_ui(signal_user_data_t *ud)
{
    GtkListBox *list_box;

    list_box = GTK_LIST_BOX(GHB_WIDGET(ud->builder, "audio_avail_lang"));
    ghb_init_lang_list_box(list_box);
}
