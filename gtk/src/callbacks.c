/* -*- Mode: C; indent-tabs-mode: t; c-basic-offset: 4; tab-width: 4 -*- */
/*
 * callbacks.c
 * Copyright (C) John Stebbins 2008-2013 <stebbins@stebbins>
 *
 * callbacks.c is free software.
 *
 * You may redistribute it and/or modify it under the terms of the
 * GNU General Public License, as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#ifdef HAVE_CONFIG_H
#  include <config.h>
#endif

#include <string.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <time.h>

#include <glib/gstdio.h>
#include <glib/gi18n.h>
#include <gio/gio.h>

#include "ghbcompat.h"

#if !defined(_WIN32)
#include <poll.h>
#define G_UDEV_API_IS_SUBJECT_TO_CHANGE 1
#if defined(__linux__)
#include <gudev/gudev.h>
#endif
#include <dbus/dbus-glib.h>
#include <dbus/dbus-glib-lowlevel.h>

#include <netinet/in.h>
#include <netdb.h>

#if !defined(_NO_UPDATE_CHECK)
#if defined(_OLD_WEBKIT)
#include <webkit.h>
#else
#include <webkit/webkit.h>
#endif
#endif

#include <libnotify/notify.h>
#ifndef NOTIFY_CHECK_VERSION
#define NOTIFY_CHECK_VERSION(x,y,z) 0
#endif

#include <gdk/gdkx.h>
#ifndef NOTIFY_CHECK_VERSION
#define NOTIFY_CHECK_VERSION(x,y,z) 0
#endif
#else
#include <winsock2.h>
#include <dbt.h>
#endif

#if defined(_USE_APP_IND)
#include <libappindicator/app-indicator.h>
#endif

#include "hb.h"
#include "callbacks.h"
#include "queuehandler.h"
#include "audiohandler.h"
#include "subtitlehandler.h"
#include "resources.h"
#include "settings.h"
#include "presets.h"
#include "preview.h"
#include "values.h"
#include "plist.h"
#include "appcast.h"
#include "hb-backend.h"
#include "ghb-dvd.h"
#include "ghbcellrenderertext.h"
#include "x264handler.h"

static void load_all_titles(signal_user_data_t *ud, int titleindex);
static void update_chapter_list_settings(GValue *settings);
static GList* dvd_device_list();
static void prune_logs(signal_user_data_t *ud);
void ghb_notify_done(signal_user_data_t *ud);
gpointer ghb_check_update(signal_user_data_t *ud);
static gboolean ghb_can_shutdown_gsm();
static void ghb_shutdown_gsm();
static gboolean ghb_can_suspend_gpm();
static void ghb_suspend_gpm();
static gboolean appcast_busy = FALSE;

// This is a dependency map used for greying widgets
// that are dependent on the state of another widget.
// The enable_value comes from the values that are
// obtained from ghb_widget_value().  For combo boxes
// you will have to look further to combo box options
// maps in hb-backend.c

GValue *dep_map;
GValue *rev_map;

void
ghb_init_dep_map()
{
    dep_map = ghb_resource_get("widget-deps");
    rev_map = ghb_resource_get("widget-reverse-deps");
}

static gboolean
dep_check(signal_user_data_t *ud, const gchar *name, gboolean *out_hide)
{
    GtkWidget *widget;
    GObject *dep_object;
    gint ii;
    gint count;
    gboolean result = TRUE;
    GValue *array, *data;
    gchar *widget_name;

    g_debug("dep_check () %s", name);

    if (rev_map == NULL) return TRUE;
    array = ghb_dict_lookup(rev_map, name);
    count = ghb_array_len(array);
    *out_hide = FALSE;
    for (ii = 0; ii < count; ii++)
    {
        data = ghb_array_get_nth(array, ii);
        widget_name = ghb_value_string(ghb_array_get_nth(data, 0));
        widget = GHB_WIDGET(ud->builder, widget_name);
        dep_object = gtk_builder_get_object(ud->builder, name);
        if (widget != NULL && !gtk_widget_is_sensitive(widget))
        {
            g_free(widget_name);
            continue;
        }
        if (dep_object == NULL)
        {
            g_message("Failed to find widget");
        }
        else
        {
            gchar *value;
            gint jj = 0;
            gchar **values;
            gboolean sensitive = FALSE;
            gboolean die, hide;

            die = ghb_value_boolean(ghb_array_get_nth(data, 2));
            hide = ghb_value_boolean(ghb_array_get_nth(data, 3));
            value = ghb_value_string(ghb_array_get_nth(data, 1));
            values = g_strsplit(value, "|", 10);
            g_free(value);

            if (widget)
                value = ghb_widget_string(widget);
            else
                value = ghb_settings_get_string(ud->settings, widget_name);
            while (values && values[jj])
            {
                if (values[jj][0] == '>')
                {
                    gdouble dbl = g_strtod (&values[jj][1], NULL);
                    gdouble dvalue = ghb_widget_double(widget);
                    if (dvalue > dbl)
                    {
                        sensitive = TRUE;
                        break;
                    }
                }
                else if (values[jj][0] == '<')
                {
                    gdouble dbl = g_strtod (&values[jj][1], NULL);
                    gdouble dvalue = ghb_widget_double(widget);
                    if (dvalue < dbl)
                    {
                        sensitive = TRUE;
                        break;
                    }
                }
                if (strcmp(values[jj], value) == 0)
                {
                    sensitive = TRUE;
                    break;
                }
                jj++;
            }
            sensitive = die ^ sensitive;
            if (!sensitive)
            {
                result = FALSE;
                *out_hide |= hide;
            }
            g_strfreev (values);
            g_free(value);
        }
        g_free(widget_name);
    }
    return result;
}

void
ghb_check_dependency(
    signal_user_data_t *ud,
    GtkWidget *widget,
    const char *alt_name)
{
    GObject *dep_object;
    const gchar *name;
    GValue *array, *data;
    gint count, ii;
    gchar *dep_name;
    GType type;

    if (widget != NULL)
    {
        type = G_OBJECT_TYPE(widget);
        if (type == GTK_TYPE_COMBO_BOX)
            if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) < 0) return;
        name = ghb_get_setting_key(widget);
    }
    else
        name = alt_name;

    g_debug("ghb_check_dependency () %s", name);

    if (dep_map == NULL) return;
    array = ghb_dict_lookup(dep_map, name);
    count = ghb_array_len(array);
    for (ii = 0; ii < count; ii++)
    {
        gboolean sensitive;
        gboolean hide;

        data = ghb_array_get_nth(array, ii);
        dep_name = ghb_value_string(data);
        dep_object = gtk_builder_get_object(ud->builder, dep_name);
        if (dep_object == NULL)
        {
            g_message("Failed to find dependent widget %s", dep_name);
            g_free(dep_name);
            continue;
        }
        sensitive = dep_check(ud, dep_name, &hide);
        gtk_widget_set_sensitive(GTK_WIDGET(dep_object), sensitive);
        if (!sensitive && hide)
        {
            if (gtk_widget_get_visible(GTK_WIDGET(dep_object)))
            {
                gtk_widget_hide(GTK_WIDGET(dep_object));
            }
        }
        else
        {
            if (!gtk_widget_get_visible(GTK_WIDGET(dep_object)))
            {
                gtk_widget_show_now(GTK_WIDGET(dep_object));
            }
        }
        g_free(dep_name);
    }
}

void
ghb_check_all_depencencies(signal_user_data_t *ud)
{
    GHashTableIter iter;
    gchar *dep_name;
    GValue *value;
    GObject *dep_object;

    g_debug("ghb_check_all_depencencies ()");
    if (rev_map == NULL) return;
    ghb_dict_iter_init(&iter, rev_map);
    // middle (void*) cast prevents gcc warning "defreferencing type-punned
    // pointer will break strict-aliasing rules"
    while (g_hash_table_iter_next(
            &iter, (gpointer*)(void*)&dep_name, (gpointer*)(void*)&value))
    {
        gboolean sensitive;
        gboolean hide;

        dep_object = gtk_builder_get_object (ud->builder, dep_name);
        if (dep_object == NULL)
        {
            g_message("Failed to find dependent widget %s", dep_name);
            continue;
        }
        sensitive = dep_check(ud, dep_name, &hide);
        gtk_widget_set_sensitive(GTK_WIDGET(dep_object), sensitive);
        if (!sensitive && hide)
        {
            gtk_widget_hide(GTK_WIDGET(dep_object));
        }
        else
        {
            gtk_widget_show_now(GTK_WIDGET(dep_object));
        }
    }
}

G_MODULE_EXPORT void
on_quit1_activate(GtkMenuItem *quit, signal_user_data_t *ud)
{
    gint state = ghb_get_queue_state();
    g_debug("on_quit1_activate ()");
    if (state & (GHB_STATE_WORKING|GHB_STATE_SEARCHING))
    {
        if (ghb_cancel_encode2(ud, _("Closing HandBrake will terminate encoding.\n")))
        {
            ghb_hb_cleanup(FALSE);
            prune_logs(ud);
            gtk_main_quit();
            return;
        }
        return;
    }
    ghb_hb_cleanup(FALSE);
    prune_logs(ud);
    gtk_main_quit();
}

gboolean
uppers_and_unders(gchar *str)
{
    if (str == NULL) return FALSE;
    str = g_strchomp(g_strchug(str));
    while (*str)
    {
        if (*str == ' ')
        {
            return FALSE;
        }
        if (*str >= 'a' && *str <= 'z')
        {
            return FALSE;
        }
        str++;
    }
    return TRUE;
}

enum
{
    CAMEL_FIRST_UPPER,
    CAMEL_OTHER
};

void
camel_convert(gchar *str)
{
    gint state = CAMEL_OTHER;

    if (str == NULL) return;
    while (*str)
    {
        if (*str == '_') *str = ' ';
        switch (state)
        {
            case CAMEL_OTHER:
            {
                if (*str >= 'A' && *str <= 'Z')
                    state = CAMEL_FIRST_UPPER;
                else
                    state = CAMEL_OTHER;

            } break;
            case CAMEL_FIRST_UPPER:
            {
                if (*str >= 'A' && *str <= 'Z')
                    *str = *str - 'A' + 'a';
                else
                    state = CAMEL_OTHER;
            } break;
        }
        str++;
    }
}

#if defined(_WIN32)
static gchar*
get_dvd_device_name(gchar *device)
{
    return g_strdup(device);
}
#else
static gchar*
get_dvd_device_name(GDrive *gd)
{
    return g_drive_get_identifier(gd, G_VOLUME_IDENTIFIER_KIND_UNIX_DEVICE);
}
#endif

static GHashTable *volname_hash = NULL;
#if GLIB_CHECK_VERSION(2, 32, 0)
static GMutex     volname_mutex_static;
#endif
static GMutex     *volname_mutex;

static void
free_volname_key(gpointer data)
{
    if (data != NULL)
        g_free(data);
}

static void
free_volname_value(gpointer data)
{
    if (data != NULL)
        g_free(data);
}

#if defined(_WIN32)
static gchar*
get_direct_dvd_volume_name(const gchar *drive)
{
    gchar *result = NULL;
    gchar vname[51], fsname[51];

    if (GetVolumeInformation(drive, vname, 50, NULL, NULL, NULL, fsname, 50))
    {
        result = g_strdup_printf("%s", vname);
    }
    return result;
}
#else
static gchar*
get_direct_dvd_volume_name(const gchar *drive)
{
    gchar *result;

    result = ghb_dvd_volname (drive);
    return result;
}
#endif

static gchar*
get_dvd_volume_name(gpointer gd)
{
    gchar *label = NULL;
    gchar *result;
    gchar *drive;

    drive = get_dvd_device_name(gd);
    g_mutex_lock(volname_mutex);
    label = g_strdup(g_hash_table_lookup(volname_hash, drive));
    g_mutex_unlock(volname_mutex);
    if (label != NULL)
    {
        if (uppers_and_unders(label))
        {
            camel_convert(label);
        }
#if defined(_WIN32)
        result = g_strdup_printf("%s (%s)", label, drive);
#else
        result = g_strdup_printf("%s - %s", drive, label);
#endif
        g_free(label);
    }
    else
    {
        result = g_strdup_printf("%s", drive);
    }
    g_free(drive);
    return result;
}

void
ghb_volname_cache_init(void)
{
#if GLIB_CHECK_VERSION(2, 32, 0)
    g_mutex_init(&volname_mutex_static);
    volname_mutex = &volname_mutex_static;
#else
    volname_mutex = g_mutex_new();
#endif
    volname_hash = g_hash_table_new_full(g_str_hash, g_str_equal,
                                        free_volname_key, free_volname_value);
}

static void
free_drive(gpointer drive)
{
#if defined(_WIN32)
        g_free(drive);
#else
        g_object_unref(drive);
#endif
}

gpointer
ghb_cache_volnames(signal_user_data_t *ud)
{
    GList *link, *drives;

    g_debug("ghb_cache_volnames()");
    link = drives = dvd_device_list();
    if (drives == NULL)
        return NULL;

    g_mutex_lock(volname_mutex);
    g_hash_table_remove_all(volname_hash);
    while (link != NULL)
    {
        gchar *name, *drive;

#if !defined(_WIN32)
        if (!g_drive_has_media (link->data))
        {
            g_object_unref(link->data);
            link = link->next;
            continue;
        }
#endif
        drive = get_dvd_device_name(link->data);
        name = get_direct_dvd_volume_name(drive);

        if (drive != NULL && name != NULL)
        {
            g_hash_table_insert(volname_hash, drive, name);
        }
        else
        {
            if (drive != NULL)
                g_free(drive);
            if (name != NULL)
                g_free(name);
        }

        free_drive(link->data);
        link = link->next;
    }
    g_mutex_unlock(volname_mutex);

    g_list_free(drives);

    g_idle_add((GSourceFunc)ghb_file_menu_add_dvd, ud);

    return NULL;
}

static const gchar*
get_extension(signal_user_data_t *ud, GValue *settings)
{
    int container;
    const gchar *extension;

    container = ghb_settings_combo_int(settings, "FileFormat");
    if ((container & HB_MUX_MASK_MP4) &&
        ghb_settings_get_boolean(ud->prefs, "UseM4v"))
    {
        return "m4v";
    }
    extension = hb_container_get_default_extension(container);
    if (extension == NULL)
        extension = "error";
    return extension;
}

static void
set_destination_settings(signal_user_data_t *ud, GValue *settings)
{
    const gchar *extension;
    gchar *filename;

    extension = get_extension(ud, settings);

    g_debug("set_destination_settings");
    if (ghb_settings_get_boolean(ud->prefs, "use_source_name"))
    {
        GString *str = g_string_new("");
        gchar *vol_name;
        gint title;

        vol_name = ghb_settings_get_string(settings, "volume_label");
        g_string_append_printf(str, "%s", vol_name);
        title = ghb_settings_get_int(settings, "title_no");
        if (title >= 0)
        {
            if (ghb_settings_get_boolean(ud->prefs, "title_no_in_destination"))
            {

                g_string_append_printf(str, " - %d", title+1);
            }
            if (ghb_settings_combo_int(settings, "PtoPType") == 0 &&
                ghb_settings_get_boolean(ud->prefs, "chapters_in_destination"))
            {
                gint start, end;

                if (!ghb_settings_get_boolean(
                        ud->prefs, "title_no_in_destination"))
                {
                    g_string_append_printf(str, " -");
                }
                start = ghb_settings_get_int(settings, "start_point");
                end = ghb_settings_get_int(settings, "end_point");
                if (start == end)
                    g_string_append_printf(str, " Ch %d", start);
                else
                    g_string_append_printf(str, " Ch %d-%d", start, end);
            }
        }
        g_string_append_printf(str, ".%s", extension);
        filename = g_string_free(str, FALSE);
        ghb_settings_set_string(settings, "dest_file", filename);
        g_free(vol_name);
        g_free(filename);
    }
    else
    {
        filename = g_strdup_printf("new_video.%s", extension);
        ghb_settings_set_string(settings, "dest_file", filename);
        g_free(filename);
    }
}

static void
set_destination(signal_user_data_t *ud)
{
    set_destination_settings(ud, ud->settings);
    ghb_ui_update(ud, "dest_file",
        ghb_settings_get_value(ud->settings, "dest_file"));
}

static gchar*
get_file_label(const gchar *filename)
{
    gchar *base, *pos, *end;

    base = g_path_get_basename(filename);
    pos = strrchr(base, '.');
    if (pos != NULL)
    {
        // If the last '.' is within 4 chars of end of name, assume
        // there is an extension we want to strip.
        end = &base[strlen(base) - 1];
        if (end - pos <= 4)
            *pos = 0;
    }
    return base;
}

static gchar*
resolve_drive_name(gchar *filename)
{
#if defined(_WIN32)
    if (filename[1] == ':')
    {
        gchar drive[4];
        gchar *name;
        gint dtype;

        g_strlcpy(drive, filename, 4);
        dtype = GetDriveType(drive);
        if (dtype == DRIVE_CDROM)
        {
            gchar vname[51], fsname[51];
            GetVolumeInformation(drive, vname, 50, NULL,
                                NULL, NULL, fsname, 50);
            name = g_strdup(vname);
            return name;
        }
    }
    return NULL;
#else
    return NULL;
#endif
}

static gboolean
update_source_label(signal_user_data_t *ud, const gchar *source)
{
    gchar *label = NULL;
    gint len;
    gchar **path;
    gchar *start;
    gchar *filename = g_strdup(source);

    g_debug("update_source_label()");
    len = strlen(filename);
    if (g_file_test(filename, G_FILE_TEST_IS_DIR))
    {
        // Skip dos drive letters
#if defined(_WIN32)
        start = strchr(filename, ':');
#else
        start = filename;
#endif
        label = resolve_drive_name(filename);
        if (label != NULL)
        {
            if (uppers_and_unders(label))
            {
                camel_convert(label);
            }
        }
        else
        {
            if (filename[len-1] == G_DIR_SEPARATOR) filename[len-1] = 0;
            if (start != NULL)
                start++;
            else
                start = filename;

            path = g_strsplit(start, G_DIR_SEPARATOR_S, -1);
            len = g_strv_length (path);
            if ((len > 1) && (strcmp("VIDEO_TS", path[len-1]) == 0))
            {
                label = g_strdup(path[len-2]);
                if (uppers_and_unders(label))
                {
                    camel_convert(label);
                }
            }
            else if (len > 0)
            {
                if (path[len-1][0] != 0)
                {
                    label = g_strdup(path[len-1]);
                    if (uppers_and_unders(label))
                    {
                        camel_convert(label);
                    }
                }
                else
                    label = g_strdup("new_video");
            }
            else
                label = g_strdup("new_video");
            g_strfreev (path);
        }
    }
    else
    {
        // Is regular file or block dev.
        // Check to see if it is a dvd image
        label = ghb_dvd_volname (filename);
        if (label == NULL)
        {
            label = get_file_label(filename);
        }
        else
        {
            if (uppers_and_unders(label))
            {
                camel_convert(label);
            }
        }
    }
    g_free(filename);
    GtkWidget *widget = GHB_WIDGET (ud->builder, "volume_label");
    if (label != NULL)
    {
        gtk_label_set_text (GTK_LABEL(widget), label);
        ghb_settings_set_string(ud->settings, "volume_label", label);
        g_free(label);
    }
    else
    {
        label = _("No Title Found");
        gtk_label_set_text (GTK_LABEL(widget), label);
        ghb_settings_set_string(ud->settings, "volume_label", label);
        return FALSE;
    }
    return TRUE;
}

G_MODULE_EXPORT void
chooser_file_selected_cb(GtkFileChooser *dialog, signal_user_data_t *ud)
{
    gchar *name = gtk_file_chooser_get_filename (dialog);
    GtkTreeModel *store;
    GtkTreeIter iter;
    const gchar *device;
    gboolean foundit = FALSE;
    GtkComboBox *combo;

    g_debug("chooser_file_selected_cb ()");

    if (name == NULL) return;
    combo = GTK_COMBO_BOX(GHB_WIDGET(ud->builder, "source_device"));
    store = gtk_combo_box_get_model(combo);
    if (gtk_tree_model_get_iter_first(store, &iter))
    {
        do
        {
            gtk_tree_model_get(store, &iter, 0, &device, -1);
            if (strcmp(name, device) == 0)
            {
                foundit = TRUE;
                break;
            }
        } while (gtk_tree_model_iter_next(GTK_TREE_MODEL(store), &iter));
    }
    if (foundit)
        gtk_combo_box_set_active_iter (combo, &iter);
    else
        gtk_combo_box_set_active (combo, 0);

    g_free(name);
}

G_MODULE_EXPORT void
dvd_device_changed_cb(GtkComboBoxText *combo, signal_user_data_t *ud)
{
    GtkWidget *dialog;
    gint ii;

    g_debug("dvd_device_changed_cb ()");
    ii = gtk_combo_box_get_active (GTK_COMBO_BOX(combo));
    if (ii > 0)
    {
        const gchar *device;
        gchar *name;

        dialog = GHB_WIDGET(ud->builder, "source_dialog");
        device = gtk_combo_box_text_get_active_text(combo);
        // Protext against unexpected NULL return value
        if (device != NULL)
        {
            name = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER(dialog));
            if (name == NULL || strcmp(name, device) != 0)
                gtk_file_chooser_select_filename (GTK_FILE_CHOOSER(dialog), device);
            if (name != NULL)
                g_free(name);
        }
    }
}

static void
source_dialog_extra_widgets(
    signal_user_data_t *ud,
    GtkWidget *dialog)
{
    GtkComboBoxText *combo;
    GList *drives, *link;

    g_debug("source_dialog_extra_widgets ()");
    combo = GTK_COMBO_BOX_TEXT(GHB_WIDGET(ud->builder, "source_device"));
    gtk_list_store_clear(GTK_LIST_STORE(
                gtk_combo_box_get_model(GTK_COMBO_BOX(combo))));

    link = drives = dvd_device_list();
    gtk_combo_box_text_append_text (combo, _("Not Selected"));
    while (link != NULL)
    {
        gchar *name = get_dvd_device_name(link->data);
        gtk_combo_box_text_append_text(combo, name);
        g_free(name);
        free_drive(link->data);
        link = link->next;
    }
    g_list_free(drives);
    gtk_combo_box_set_active(GTK_COMBO_BOX(combo), 0);
}

static void break_duration(gint64 duration, gint *hh, gint *mm, gint *ss)
{
    *hh = duration / (60*60);
    *mm = (duration / 60) % 60;
    *ss = duration % 60;
}

static void
update_title_duration(signal_user_data_t *ud)
{
    gint ti;
    gint hh, mm, ss, start, end;
    gchar *text;
    GtkWidget *widget;

    ti = ghb_settings_combo_int(ud->settings, "title");
    widget = GHB_WIDGET (ud->builder, "title_duration");

    if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        ghb_part_duration(ti, start, end, &hh, &mm, &ss);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 1)
    {
        gint duration;

        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        duration = end - start;
        break_duration(duration, &hh, &mm, &ss);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 2)
    {
        hb_title_t * title = ghb_get_title_info (ti);
        if (title != NULL)
        {
            gint64 frames;
            gint duration;

            start = ghb_settings_get_int(ud->settings, "start_point");
            end = ghb_settings_get_int(ud->settings, "end_point");
            frames = end - start + 1;
            duration = frames * title->rate_base / title->rate;
            break_duration(duration, &hh, &mm, &ss);
        }
        else
        {
            hh = mm = ss = 0;
        }
    }
    text = g_strdup_printf("%02d:%02d:%02d", hh, mm, ss);
    gtk_label_set_text(GTK_LABEL(widget), text);
    g_free(text);
}

static void show_container_options(signal_user_data_t *ud)
{
    GtkWidget *w1, *w2, *w3;
    w1 = GHB_WIDGET(ud->builder, "Mp4LargeFile");
    w2 = GHB_WIDGET(ud->builder, "Mp4HttpOptimize");
    w3 = GHB_WIDGET(ud->builder, "Mp4iPodCompatible");

    gint mux = ghb_settings_combo_int(ud->settings, "FileFormat");
    gint enc = ghb_settings_combo_int(ud->settings, "VideoEncoder");

    gtk_widget_set_visible(w1, (mux == HB_MUX_MP4V2));
    gtk_widget_set_visible(w2, (mux & HB_MUX_MASK_MP4));
    gtk_widget_set_visible(w3, (mux & HB_MUX_MASK_MP4) && (enc == HB_VCODEC_X264));
}

static void
adjustment_configure(
    GtkAdjustment *adj,
    double val,
    double min, double max,
    double step, double page, double page_sz)
{
    gtk_adjustment_configure(adj, val, min, max, step, page, page_sz);
}

static void
spin_configure(signal_user_data_t *ud, char *name, double val, double min, double max)
{
    GtkSpinButton *spin;
    GtkAdjustment *adj;
    double step, page, page_sz;

    spin = GTK_SPIN_BUTTON(GHB_WIDGET(ud->builder, name));

    adj = gtk_spin_button_get_adjustment(spin);
    step = gtk_adjustment_get_step_increment(adj);
    page = gtk_adjustment_get_page_increment(adj);
    page_sz = gtk_adjustment_get_page_size(adj);

    adjustment_configure(adj, val, min, max, step, page, page_sz);
}

static void
scale_configure(
    signal_user_data_t *ud,
    char *name,
    double val, double min, double max,
    double step, double page,
    int digits, gboolean inverted)
{
    GtkScale *scale;
    GtkAdjustment *adj;
    double page_sz;

    scale = GTK_SCALE(GHB_WIDGET(ud->builder, name));

    adj = gtk_range_get_adjustment(GTK_RANGE(scale));
    page_sz = gtk_adjustment_get_page_size(adj);

    adjustment_configure(adj, val, min, max, step, page, page_sz);

    gtk_scale_set_digits(scale, digits);
    gtk_range_set_inverted(GTK_RANGE(scale), inverted);
}

void
ghb_set_widget_ranges(signal_user_data_t *ud, GValue *settings)
{
    int titleindex = ghb_settings_combo_int(settings, "title");
    hb_title_t * title = ghb_get_title_info(titleindex);
    double val;

    // Reconfigure the UI combo boxes
    ghb_update_ui_combo_box(ud, "AudioTrack", titleindex, FALSE);
    ghb_update_ui_combo_box(ud, "SubtitleTrack", titleindex, FALSE);

    if (title != NULL)
    {

        // Set the limits of cropping.  hb_set_anamorphic_size crashes if
        // you pass it a cropped width or height == 0.
        gint vbound, hbound;
        vbound = title->height / 2 - 8;
        hbound = title->width / 2 - 8;

        val = ghb_settings_get_int(ud->settings, "PictureTopCrop");
        spin_configure(ud, "PictureTopCrop", val, 0, vbound);
        val = ghb_settings_get_int(ud->settings, "PictureBottomCrop");
        spin_configure(ud, "PictureBottomCrop", val, 0, vbound);
        val = ghb_settings_get_int(ud->settings, "PictureLeftCrop");
        spin_configure(ud, "PictureLeftCrop", val, 0, hbound);
        val = ghb_settings_get_int(ud->settings, "PictureRightCrop");
        spin_configure(ud, "PictureRightCrop", val, 0, hbound);

        gint duration = title->duration / 90000;

        if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
        {
            gint num_chapters = hb_list_count(title->list_chapter);

            val = ghb_settings_get_int(ud->settings, "start_point");
            spin_configure(ud, "start_point", val, 1, num_chapters);
            val = ghb_settings_get_int(ud->settings, "end_point");
            spin_configure(ud, "end_point", val, 1, num_chapters);
        }
        else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 1)
        {
            val = ghb_settings_get_int(ud->settings, "start_point");
            spin_configure(ud, "start_point", val, 0, duration-1);
            val = ghb_settings_get_int(ud->settings, "end_point");
            spin_configure(ud, "end_point", val, 0, duration);
        }
        else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 2)
        {
            gdouble max_frames;
            max_frames = (gdouble)duration * title->rate / title->rate_base;

            val = ghb_settings_get_int(ud->settings, "start_point");
            spin_configure(ud, "start_point", val, 1, max_frames);
            val = ghb_settings_get_int(ud->settings, "end_point");
            spin_configure(ud, "end_point", val, 1, max_frames);
        }

        val = ghb_settings_get_int(ud->settings, "angle");
        spin_configure(ud, "angle", val, 1, title->angle_count);
    }

    float vqmin, vqmax, step, page;
    int inverted, digits;

    ghb_vquality_range(ud, &vqmin, &vqmax, &step, &page, &digits, &inverted);
    val = ghb_settings_get_double(ud->settings, "VideoQualitySlider");
    scale_configure(ud, "VideoQualitySlider", val, vqmin, vqmax,
                    step, page, digits, inverted);
}

static void
check_chapter_markers(signal_user_data_t *ud)
{
    GtkWidget *widget;
    gint start, end;

    if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        widget = GHB_WIDGET (ud->builder, "ChapterMarkers");
        gtk_widget_set_sensitive(widget, end > start);
    }
}

#if 0
void
show_settings(GValue *settings)
{
    GHashTableIter iter;
    gchar *key;
    GValue *gval;

    ghb_dict_iter_init(&iter, settings);
    // middle (void*) cast prevents gcc warning "defreferencing type-punned
    // pointer will break strict-aliasing rules"
    while (g_hash_table_iter_next(
            &iter, (gpointer*)(void*)&key, (gpointer*)(void*)&gval))
    {
        char *str = ghb_value_string(gval);
        printf("show key %s val %s\n", key, str);
        g_free(str);
    }
}
#endif

void
ghb_load_settings(signal_user_data_t * ud)
{
    GValue *preset;
    gboolean preset_modified;
    static gboolean busy = FALSE;

    if (busy)
        return;
    busy = TRUE;

    preset = ghb_settings_get_value(ud->settings, "preset");
    preset_modified = ghb_settings_get_boolean(ud->settings, "preset_modified");
    if (preset_modified)
    {
        ghb_clear_presets_selection(ud);
    }
    else
    {
        ghb_settings_set_boolean(ud->settings, "preset_reload", TRUE);
        ghb_select_preset(ud->builder, preset);
        ghb_settings_set_boolean(ud->settings, "preset_reload", FALSE);
    }

    ud->dont_clear_presets = TRUE;
    ud->scale_busy = TRUE;

    ghb_set_widget_ranges(ud, ud->settings);
    ghb_check_all_depencencies(ud);
    show_container_options(ud);
    check_chapter_markers(ud);

    ghb_settings_to_ui(ud, ud->settings);
    ghb_audio_defaults_to_ui(ud);
    ghb_subtitle_defaults_to_ui(ud);
    ghb_audio_list_refresh_all(ud);
    ghb_subtitle_list_refresh_all(ud);
    ghb_chapter_list_refresh_all(ud);
    update_title_duration(ud);
    ghb_update_title_info(ud);

    ud->dont_clear_presets = FALSE;
    ud->scale_busy = FALSE;
    busy = FALSE;

    ghb_picture_settings_deps(ud);
}

static void
show_scan_progress(signal_user_data_t *ud)
{
    GtkProgressBar *progress;
    GtkLabel *label;

    progress = GTK_PROGRESS_BAR(GHB_WIDGET(ud->builder, "scan_prog"));
    gtk_progress_bar_set_fraction (progress, 0);
    gtk_widget_show(GTK_WIDGET(progress));

    label = GTK_LABEL(GHB_WIDGET(ud->builder, "volume_label"));
    gtk_label_set_text( label, _("Scanning ...") );
}

static void
start_scan(
    signal_user_data_t *ud,
    const gchar *path,
    gint titlenum,
    gint preview_count)
{
    GtkWidget *widget;
    ghb_status_t status;

    ghb_get_status(&status);
    if (status.scan.state != GHB_STATE_IDLE)
        return;

    widget = GHB_WIDGET(ud->builder, "sourcetoolbutton");
    gtk_tool_button_set_icon_name(GTK_TOOL_BUTTON(widget), "hb-stop");
    gtk_tool_button_set_label(GTK_TOOL_BUTTON(widget), _("Stop Scan"));
    gtk_tool_item_set_tooltip_text(GTK_TOOL_ITEM(widget), _("Stop Scan"));

    widget = GHB_WIDGET(ud->builder, "source_open");
    gtk_widget_set_sensitive(widget, FALSE);
    widget = GHB_WIDGET(ud->builder, "source_title_open");
    gtk_widget_set_sensitive(widget, FALSE);
    ghb_backend_scan(path, titlenum, preview_count,
            90000L * ghb_settings_get_int64(ud->prefs, "MinTitleDuration"));
}

gboolean
ghb_idle_scan(signal_user_data_t *ud)
{
    gchar *path;
    path = ghb_settings_get_string(ud->globals, "scan_source");
    ghb_do_scan(ud, path, 0, TRUE);
    g_free(path);
    return FALSE;
}

extern GValue *ghb_queue_edit_settings;
static gchar *last_scan_file = NULL;

void
ghb_do_scan(
    signal_user_data_t *ud,
    const gchar *filename,
    gint titlenum,
    gboolean force)
{
    g_debug("ghb_do_scan()");
    if (!force && last_scan_file != NULL &&
        strcmp(last_scan_file, filename) == 0)
    {
        if (ghb_queue_edit_settings != NULL)
        {
            int titleindex = ghb_settings_get_int(ghb_queue_edit_settings, "title_no");
            ghb_array_replace(ud->settings_array, titleindex, ghb_queue_edit_settings);
            ud->settings = ghb_queue_edit_settings;
            ghb_load_settings(ud);
            ghb_queue_edit_settings = NULL;
        }
        else
        {
            int titleindex = ghb_settings_get_int(ud->settings, "title_no");
            load_all_titles(ud, titleindex);
        }
        return;
    }
    if (last_scan_file != NULL)
        g_free(last_scan_file);
    last_scan_file = NULL;
    if (filename != NULL)
    {
        last_scan_file = g_strdup(filename);
        ghb_settings_set_string(ud->globals, "scan_source", filename);
        if (update_source_label(ud, filename))
        {
            gchar *path;
            gint preview_count;

            show_scan_progress(ud);
            path = ghb_settings_get_string(ud->globals, "scan_source");
            prune_logs(ud);

            preview_count = ghb_settings_get_int(ud->prefs, "preview_count");
            start_scan(ud, path, titlenum, preview_count);
            g_free(path);
        }
        else
        {
            // TODO: error dialog
        }
    }
}

static void
do_source_dialog(GtkButton *button, gboolean single, signal_user_data_t *ud)
{
    GtkWidget *dialog;
    gchar *sourcename;
    gint    response;

    g_debug("source_browse_clicked_cb ()");
    sourcename = ghb_settings_get_string(ud->globals, "scan_source");
    GtkWidget *widget;
    widget = GHB_WIDGET(ud->builder, "single_title_box");
    if (single)
        gtk_widget_show(widget);
    else
        gtk_widget_hide(widget);
    dialog = GHB_WIDGET(ud->builder, "source_dialog");
    source_dialog_extra_widgets(ud, dialog);

#if GTK_CHECK_VERSION(3, 0, 0)
    // gtk3 has a STUPID BUG!  If you select the "same_path" it ends
    // up selecting the "Recent Files" shortcut instead of taking you
    // to the directory and file THAT YOU ASKED FOR!!!  FUUUUK!!!
    //
    // So instead, I am just setting the current folder.  It's not
    // optimal because if you are processing several files in the same
    // folder, you want the selection to return to where you were last
    // in the list, but there's not much else I can do at this point.
    gchar *dirname = g_path_get_dirname(sourcename);
    gtk_file_chooser_set_current_folder(GTK_FILE_CHOOSER(dialog), dirname);
    g_free(dirname);
#else
    gtk_file_chooser_set_filename(GTK_FILE_CHOOSER(dialog), sourcename);
#endif
    response = gtk_dialog_run(GTK_DIALOG (dialog));
    gtk_widget_hide(dialog);
    if (response == GTK_RESPONSE_NO)
    {
        gchar *filename;

        filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
        if (filename != NULL)
        {
            gint titlenum;

            if (single)
                titlenum = ghb_settings_get_int(ud->settings, "single_title");
            else
                titlenum = 0;
            ghb_do_scan(ud, filename, titlenum, TRUE);
            if (strcmp(sourcename, filename) != 0)
            {
                ghb_settings_set_string(ud->prefs, "default_source", filename);
                ghb_pref_save(ud->prefs, "default_source");
                ghb_dvd_set_current(filename, ud);
            }
            g_free(filename);
        }
    }
    g_free(sourcename);
}

G_MODULE_EXPORT void
source_button_clicked_cb(GtkButton *button, signal_user_data_t *ud)
{
    ghb_status_t status;
    ghb_get_status(&status);
    if (status.scan.state & GHB_STATE_SCANNING)
    {
        ghb_backend_scan_stop();
    }
    else
    {
        do_source_dialog(button, FALSE, ud);
    }
}

G_MODULE_EXPORT void
single_title_source_cb(GtkButton *button, signal_user_data_t *ud)
{
    do_source_dialog(button, TRUE, ud);
}

G_MODULE_EXPORT void
dvd_source_activate_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    const gchar *filename;
    gchar *sourcename;

    sourcename = ghb_settings_get_string(ud->globals, "scan_source");
    filename = gtk_buildable_get_name(GTK_BUILDABLE(widget));
    ghb_do_scan(ud, filename, 0, TRUE);
    if (strcmp(sourcename, filename) != 0)
    {
        ghb_settings_set_string(ud->prefs, "default_source", filename);
        ghb_pref_save(ud->prefs, "default_source");
        ghb_dvd_set_current(filename, ud);
    }
    g_free(sourcename);
}

void
ghb_update_destination_extension(signal_user_data_t *ud)
{
    static gchar *containers[] = {".mkv", ".mp4", ".m4v", ".error", NULL};
    gchar *filename;
    const gchar *extension;
    gint ii;
    GtkEntry *entry;
    static gboolean busy = FALSE;

    g_debug("ghb_update_destination_extension ()");
    // Since this function modifies the thing that triggers it's
    // invocation, check to see if busy to prevent accidental infinite
    // recursion.
    if (busy)
        return;
    busy = TRUE;
    extension = get_extension(ud, ud->settings);
    entry = GTK_ENTRY(GHB_WIDGET(ud->builder, "dest_file"));
    filename = g_strdup(gtk_entry_get_text(entry));
    for (ii = 0; containers[ii] != NULL; ii++)
    {
        if (g_str_has_suffix(filename, containers[ii]))
        {
            gchar *pos;
            gchar *new_name;

            pos = g_strrstr( filename, "." );
            if (pos == NULL)
            {
                // No period? shouldn't happen
                break;
            }
            *pos = 0;
            if (strcmp(extension, &pos[1]) == 0)
            {
                // Extension is already correct
                break;
            }
            new_name = g_strjoin(".", filename, extension, NULL);
            ghb_ui_update(ud, "dest_file", ghb_string_value(new_name));
            g_free(new_name);
            break;
        }
    }
    g_free(filename);
    busy = FALSE;
}

static void
destination_select_title(GtkEntry *entry)
{
    const gchar *dest;
    gint start, end;

    dest = gtk_entry_get_text(entry);
    for (end = strlen(dest)-1; end > 0; end--)
    {
        if (dest[end] == '.')
        {
            break;
        }
    }
    for (start = end; start >= 0; start--)
    {
        if (dest[start] == G_DIR_SEPARATOR)
        {
            start++;
            break;
        }
    }
    if (start < 0) start = 0;
    if (start < end)
    {
        gtk_editable_select_region(GTK_EDITABLE(entry), start, end);
    }
}

G_MODULE_EXPORT gboolean
destination_grab_cb(
    GtkEntry *entry,
    signal_user_data_t *ud)
{
    destination_select_title(entry);
    return FALSE;
}

static gboolean update_default_destination = FALSE;

G_MODULE_EXPORT void
dest_dir_set_cb(GtkFileChooserButton *dest_chooser, signal_user_data_t *ud)
{
    gchar *dest_file, *dest_dir, *dest;

    g_debug("dest_dir_set_cb ()");
    ghb_widget_to_setting(ud->settings, (GtkWidget*)dest_chooser);
    dest_file = ghb_settings_get_string(ud->settings, "dest_file");
    dest_dir = ghb_settings_get_string(ud->settings, "dest_dir");
    dest = g_strdup_printf("%s" G_DIR_SEPARATOR_S "%s", dest_dir, dest_file);
    ghb_settings_set_string(ud->settings, "destination", dest);
    g_free(dest_file);
    g_free(dest_dir);
    g_free(dest);
    update_default_destination = TRUE;
}

G_MODULE_EXPORT void
dest_file_changed_cb(GtkEntry *entry, signal_user_data_t *ud)
{
    gchar *dest_file, *dest_dir, *dest;

    g_debug("dest_file_changed_cb ()");
    ghb_update_destination_extension(ud);
    ghb_widget_to_setting(ud->settings, (GtkWidget*)entry);
    // This signal goes off with ever keystroke, so I'm putting this
    // update on the timer.
    dest_file = ghb_settings_get_string(ud->settings, "dest_file");
    dest_dir = ghb_settings_get_string(ud->settings, "dest_dir");
    dest = g_strdup_printf("%s" G_DIR_SEPARATOR_S "%s", dest_dir, dest_file);
    ghb_settings_set_string(ud->settings, "destination", dest);
    g_free(dest_file);
    g_free(dest_dir);
    g_free(dest);
    update_default_destination = TRUE;
}

G_MODULE_EXPORT void
destination_browse_clicked_cb(GtkButton *button, signal_user_data_t *ud)
{
    GtkWidget *dialog;
    GtkEntry *entry;
    gchar *destname;
    gchar *basename;

    g_debug("destination_browse_clicked_cb ()");
    destname = ghb_settings_get_string(ud->settings, "destination");
    dialog = gtk_file_chooser_dialog_new ("Choose Destination",
                      NULL,
                      GTK_FILE_CHOOSER_ACTION_SAVE,
                      GHB_STOCK_CANCEL, GTK_RESPONSE_CANCEL,
                      GHB_STOCK_SAVE, GTK_RESPONSE_ACCEPT,
                      NULL);
    gtk_file_chooser_set_filename(GTK_FILE_CHOOSER(dialog), destname);
    basename = g_path_get_basename(destname);
    g_free(destname);
    gtk_file_chooser_set_current_name(GTK_FILE_CHOOSER(dialog), basename);
    g_free(basename);
    if (gtk_dialog_run(GTK_DIALOG (dialog)) == GTK_RESPONSE_ACCEPT)
    {
        char *filename, *dirname;
        GtkFileChooser *dest_chooser;

        filename = gtk_file_chooser_get_filename (GTK_FILE_CHOOSER (dialog));
        basename = g_path_get_basename(filename);
        dirname = g_path_get_dirname(filename);
        entry = (GtkEntry*)GHB_WIDGET(ud->builder, "dest_file");
        gtk_entry_set_text(entry, basename);
        dest_chooser = GTK_FILE_CHOOSER(GHB_WIDGET(ud->builder, "dest_dir"));
        gtk_file_chooser_set_filename(dest_chooser, dirname);
        g_free (dirname);
        g_free (basename);
        g_free (filename);
    }
    gtk_widget_destroy(dialog);
}

G_MODULE_EXPORT gboolean
window_destroy_event_cb(GtkWidget *widget, GdkEvent *event, signal_user_data_t *ud)
{
    g_debug("window_destroy_event_cb ()");
    ghb_hb_cleanup(FALSE);
    prune_logs(ud);
    gtk_main_quit();
    return FALSE;
}

G_MODULE_EXPORT gboolean
window_delete_event_cb(GtkWidget *widget, GdkEvent *event, signal_user_data_t *ud)
{
    gint state = ghb_get_queue_state();
    g_debug("window_delete_event_cb ()");
    if (state & (GHB_STATE_WORKING|GHB_STATE_SEARCHING))
    {
        if (ghb_cancel_encode2(ud, _("Closing HandBrake will terminate encoding.\n")))
        {
            ghb_hb_cleanup(FALSE);
            prune_logs(ud);
            gtk_main_quit();
            return FALSE;
        }
        return TRUE;
    }
    ghb_hb_cleanup(FALSE);
    prune_logs(ud);
    gtk_main_quit();
    return FALSE;
}

static void
update_acodec(signal_user_data_t *ud)
{
    ghb_adjust_audio_rate_combos(ud);
    ghb_grey_combo_options (ud);
}

G_MODULE_EXPORT void
container_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("container_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    show_container_options(ud);
    update_acodec(ud);
    ghb_update_destination_extension(ud);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    ghb_subtitle_prune(ud);
    ghb_subtitle_list_refresh_all(ud);
    ghb_audio_list_refresh_selected(ud);
}

static gchar*
get_aspect_string(gint aspect_n, gint aspect_d)
{
    gchar *aspect;

    if (aspect_d < 10)
    {
        aspect = g_strdup_printf("%d:%d", aspect_n, aspect_d);
    }
    else
    {
        gdouble aspect_nf = (gdouble)aspect_n / aspect_d;
        aspect = g_strdup_printf("%.2f:1", aspect_nf);
    }
    return aspect;
}

static gchar*
get_rate_string(gint rate_base, gint rate)
{
    gdouble rate_f = (gdouble)rate / rate_base;
    gchar *rate_s;

    rate_s = g_strdup_printf("%.6g", rate_f);
    return rate_s;
}

static void
update_aspect_info(signal_user_data_t *ud)
{
    GtkWidget *widget;
    gchar *text;

    text = ghb_settings_get_boolean(ud->settings, "PictureAutoCrop") ? _("On") : _("Off");
    widget = GHB_WIDGET(ud->builder, "crop_auto");
    gtk_label_set_text(GTK_LABEL(widget), text);
    text = ghb_settings_get_boolean(ud->settings, "autoscale") ? _("On") : _("Off");
    widget = GHB_WIDGET(ud->builder, "scale_auto");
    gtk_label_set_text(GTK_LABEL(widget), text);
    switch (ghb_settings_combo_int(ud->settings, "PicturePAR"))
    {
        case 0:
            text = _("Off");
            break;
        case 1:
            text = _("Strict");
            break;
        case 2:
            text = _("Loose");
            break;
        case 3:
            text = _("Custom");
            break;
        default:
            text = _("Unknown");
            break;
    }
    widget = GHB_WIDGET(ud->builder, "scale_anamorphic");
    gtk_label_set_text(GTK_LABEL(widget), text);
}

static void
update_crop_info(signal_user_data_t *ud)
{
    GtkWidget *widget;
    gchar *text;
    gint width, height, crop[4] = {0,};
    gint titleindex;
    hb_title_t *title;

    titleindex = ghb_settings_combo_int(ud->settings, "title");
    title = ghb_get_title_info(titleindex);
    if (title != NULL)
    {
        crop[0] = ghb_settings_get_int(ud->settings, "PictureTopCrop");
        crop[1] = ghb_settings_get_int(ud->settings, "PictureBottomCrop");
        crop[2] = ghb_settings_get_int(ud->settings, "PictureLeftCrop");
        crop[3] = ghb_settings_get_int(ud->settings, "PictureRightCrop");
        width = title->width - crop[2] - crop[3];
        height = title->height - crop[0] - crop[1];
        widget = GHB_WIDGET(ud->builder, "crop_dimensions");
        text = g_strdup_printf ("%d x %d", width, height);
        gtk_label_set_text(GTK_LABEL(widget), text);
        widget = GHB_WIDGET(ud->builder, "crop_dimensions2");
        gtk_label_set_text(GTK_LABEL(widget), text);
        g_free(text);
    }
    widget = GHB_WIDGET (ud->builder, "crop_values");
    text = g_strdup_printf ("%d:%d:%d:%d", crop[0], crop[1], crop[2], crop[3]);
    gtk_label_set_text (GTK_LABEL(widget), text);
    g_free(text);
}

static void
update_scale_info(signal_user_data_t *ud)
{
    GtkWidget *widget;
    gchar *text;

    gint width = ghb_settings_get_int(ud->settings, "scale_width");
    gint height = ghb_settings_get_int(ud->settings, "scale_height");
    widget = GHB_WIDGET(ud->builder, "scale_dimensions");
    text = g_strdup_printf("%d x %d", width, height);
    gtk_label_set_text(GTK_LABEL(widget), text);
    g_free(text);
}

void
ghb_update_title_info(signal_user_data_t *ud)
{
    GtkWidget *widget;
    gchar *text;

    int titleindex = ghb_settings_get_int(ud->settings, "title_no");
    hb_title_t * title = ghb_get_title_info(titleindex);
    if (title == NULL)
        return;

    update_title_duration(ud);

    widget = GHB_WIDGET (ud->builder, "source_video_codec");
    if ( title->video_codec_name )
        gtk_label_set_text (GTK_LABEL(widget), title->video_codec_name);
    else
        gtk_label_set_text (GTK_LABEL(widget), "Unknown");

    widget = GHB_WIDGET (ud->builder, "source_dimensions");
    text = g_strdup_printf ("%d x %d", title->width, title->height);
    gtk_label_set_text (GTK_LABEL(widget), text);
    g_free(text);

    widget = GHB_WIDGET (ud->builder, "source_aspect");
    gint aspect_n, aspect_d;
    hb_reduce(&aspect_n, &aspect_d,
                title->width * title->pixel_aspect_width,
                title->height * title->pixel_aspect_height);
    text = get_aspect_string(aspect_n, aspect_d);
    gtk_label_set_text (GTK_LABEL(widget), text);
    g_free(text);

    widget = GHB_WIDGET (ud->builder, "source_frame_rate");
    text = (gchar*)get_rate_string(title->rate_base, title->rate);
    gtk_label_set_text (GTK_LABEL(widget), text);
    g_free(text);

    //widget = GHB_WIDGET (ud->builder, "source_interlaced");
    //gtk_label_set_text (GTK_LABEL(widget), title->interlaced ? "Yes" : "No");

    ghb_update_display_aspect_label(ud);

    update_crop_info(ud);
    update_aspect_info(ud);
    update_scale_info(ud);
}

void
set_title_settings(signal_user_data_t *ud, GValue *settings, gint titleindex)
{
    ghb_settings_set_int(settings, "title", titleindex);
    ghb_settings_set_int(settings, "title_no", titleindex);

    hb_title_t * title = ghb_get_title_info(titleindex);
    if (title != NULL)
    {
        gint num_chapters = hb_list_count(title->list_chapter);

        ghb_settings_set_int(settings, "angle", 1);
        ghb_settings_set_int(settings, "PtoPType", 0);
        ghb_settings_set_int(settings, "start_point", 1);
        ghb_settings_set_int(settings, "end_point", num_chapters);
        ghb_settings_set_int(settings, "source_width", title->width);
        ghb_settings_set_int(settings, "source_height", title->height);
        ghb_settings_set_string(settings, "source", title->path);
        if (title->type == HB_STREAM_TYPE || title->type == HB_FF_STREAM_TYPE)
        {
            if (title->name != NULL && title->name[0] != 0)
            {
                ghb_settings_set_string(settings, "volume_label", title->name);
            }
            else
            {
                gchar *label = _("No Title Found");
                ghb_settings_set_string(settings, "volume_label", label);
            }
        }
        else
        {
            ghb_settings_set_value(settings, "volume_label",
                ghb_settings_get_value(ud->settings, "volume_label"));
        }
        ghb_settings_set_int(settings, "scale_width",
                             title->width - title->crop[2] - title->crop[3]);

        // If anamorphic or keep_aspect, the hight will
        // be automatically calculated
        gboolean keep_aspect;
        gint pic_par;
        keep_aspect = ghb_settings_get_boolean(settings, "PictureKeepRatio");
        pic_par = ghb_settings_combo_int(settings, "PicturePAR");
        if (!(keep_aspect || pic_par) || pic_par == 3)
        {
            ghb_settings_set_int(settings, "scale_height",
                             title->width - title->crop[0] - title->crop[1]);
        }

        ghb_set_scale_settings(settings, GHB_PIC_KEEP_PAR|GHB_PIC_USE_MAX);
        ghb_settings_set_int(settings, "angle_count", title->angle_count);

        ghb_settings_set_string(settings, "MetaName", title->name);
        if (title->metadata)
        {
            if (title->metadata->name)
            {
                ghb_settings_set_string(settings, "MetaName",
                    title->metadata->name);
            }
            ghb_settings_set_string(settings, "MetaArtist",
                    title->metadata->artist);
            ghb_settings_set_string(settings, "MetaReleaseDate",
                    title->metadata->release_date);
            ghb_settings_set_string(settings, "MetaComment",
                    title->metadata->comment);
            if (!title->metadata->name && title->metadata->album)
            {
                ghb_settings_set_string(settings, "MetaName",
                    title->metadata->album);
            }
            ghb_settings_set_string(settings, "MetaAlbumArtist",
                    title->metadata->album_artist);
            ghb_settings_set_string(settings, "MetaGenre",
                    title->metadata->genre);
            ghb_settings_set_string(settings, "MetaDescription",
                    title->metadata->description);
            ghb_settings_set_string(settings, "MetaLongDescription",
                    title->metadata->long_description);
        }
        ghb_set_pref_subtitle_settings(ud, title, settings);
    }
    update_chapter_list_settings(settings);
    ghb_set_pref_audio_settings(titleindex, settings);

    set_destination_settings(ud, settings);
    ghb_settings_set_value(settings, "dest_dir",
                       ghb_settings_get_value(ud->prefs, "destination_dir"));

    char *dest_file, *dest_dir, *dest;
    dest_file = ghb_settings_get_string(settings, "dest_file");
    dest_dir = ghb_settings_get_string(settings, "dest_dir");
    dest = g_strdup_printf("%s" G_DIR_SEPARATOR_S "%s", dest_dir, dest_file);
    ghb_settings_set_string(settings, "destination", dest);
    g_free(dest_file);
    g_free(dest_dir);
    g_free(dest);

    ghb_settings_set_int(settings, "preview_frame", 2);
}

void
ghb_set_current_title_settings(signal_user_data_t *ud)
{
    int titleindex = ghb_settings_get_int(ud->settings, "title_no");
    set_title_settings(ud, ud->settings, titleindex);
}

static void
load_all_titles(signal_user_data_t *ud, int titleindex)
{
    gint ii, count;
    GValue *preset, *preset_path;
    GValue *settings_array;

    hb_list_t *list = ghb_get_title_list();
    count = hb_list_count(list);

    if (count == 0)
        count = 1;

    settings_array = ghb_array_value_new(count);

    preset = ghb_get_current_preset(ud);
    preset_path = ghb_get_current_preset_path(ud);
    for (ii = 0; ii < count; ii++)
    {
        GValue *settings = ghb_settings_new();

        ghb_settings_init(settings, "Initialization");
        ghb_preset_to_settings(settings, preset);
        ghb_settings_set_value(settings, "preset", preset_path);
        set_title_settings(ud, settings, ii);
        ghb_array_append(settings_array, settings);
    }
    ghb_value_free(preset_path);
    if (titleindex < 0 || titleindex >= count)
    {
        titleindex = 0;
    }
    ghb_value_free(ud->settings_array);
    ud->settings_array = settings_array;
    ud->settings = ghb_array_get_nth(ud->settings_array, titleindex);
}

static gboolean update_preview = FALSE;

G_MODULE_EXPORT void
title_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    gint titleindex, count;
    GValue *wval;

    g_debug("title_changed_cb ()");

    wval = ghb_widget_value(widget);
    titleindex = ghb_lookup_combo_int("title", wval);
    ghb_value_free(wval);

    count = ghb_array_len(ud->settings_array);
    int idx = (titleindex >= 0 && titleindex < count) ? titleindex : 0;
    ud->settings = ghb_array_get_nth(ud->settings_array, idx);
    ghb_load_settings(ud);

    hb_title_t * title = ghb_get_title_info(titleindex);
    ghb_audio_title_change(ud, title != NULL);
    ghb_subtitle_title_change(ud, title != NULL);
    ghb_grey_combo_options(ud);

    if (title != NULL)
    {
        ghb_set_preview_image(ud);
        ghb_preview_set_visible(ud);
    }
}

G_MODULE_EXPORT void
ptop_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    gint ti;
    hb_title_t * title;

    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_live_reset(ud);

    ti = ghb_settings_combo_int(ud->settings, "title");
    title = ghb_get_title_info(ti);
    if (title == NULL)
        return;

    gint duration = title->duration / 90000;
    if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
    {
        gint num_chapters = hb_list_count(title->list_chapter);
        spin_configure(ud, "start_point", 1, 1, num_chapters);
        spin_configure(ud, "end_point", num_chapters, 1, num_chapters);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 1)
    {
        spin_configure(ud, "start_point", 0, 0, duration-1);
        spin_configure(ud, "end_point", duration, 0, duration);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 2)
    {
        gdouble max_frames = (gdouble)duration * title->rate / title->rate_base;
        spin_configure(ud, "start_point", 1, 1, max_frames);
        spin_configure(ud, "end_point", max_frames, 1, max_frames);
    }
}

G_MODULE_EXPORT void
framerate_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);

    if (ghb_settings_combo_int(ud->settings, "VideoFramerate") != 0)
    {
        if (!ghb_settings_get_boolean(ud->settings, "VideoFrameratePFR"))
        {
            ghb_ui_update(ud, "VideoFramerateCFR", ghb_boolean_value(TRUE));
        }
    }
    if (ghb_settings_combo_int(ud->settings, "VideoFramerate") == 0 &&
        ghb_settings_get_boolean(ud->settings, "VideoFrameratePFR"))
    {
        ghb_ui_update(ud, "VideoFramerateVFR", ghb_boolean_value(TRUE));
    }
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
}

G_MODULE_EXPORT void
setting_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
}

G_MODULE_EXPORT gboolean
meta_focus_out_cb(GtkWidget *widget, GdkEventFocus *event,
    signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    return FALSE;
}

G_MODULE_EXPORT void
meta_setting_widget_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
}

G_MODULE_EXPORT void
chapter_markers_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
}

G_MODULE_EXPORT void
vquality_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);

    double vquality = ghb_settings_get_double(ud->settings, "VideoQualitySlider");
    if (vquality < 1.0)
    {
        ghb_ui_update(ud, "h264Profile", ghb_string_value("auto"));
    }

    gint vcodec = ghb_settings_combo_int(ud->settings, "VideoEncoder");
    gdouble step;
    if (vcodec == HB_VCODEC_X264)
    {
        step = ghb_settings_combo_double(ud->prefs, "VideoQualityGranularity");
    }
    else
    {
        step = 1;
    }
    gdouble val = gtk_range_get_value(GTK_RANGE(widget));
    val = ((int)((val + step / 2) / step)) * step;
    gtk_range_set_value(GTK_RANGE(widget), val);
}

G_MODULE_EXPORT void
http_opt_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    // AC3 is not allowed when Web optimized
    ghb_grey_combo_options (ud);
}

G_MODULE_EXPORT void
vcodec_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    float val, vqmin, vqmax, step, page;
    int inverted, digits;

    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    show_container_options(ud);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);

    val = ghb_vquality_default(ud);
    ghb_vquality_range(ud, &vqmin, &vqmax, &step, &page, &digits, &inverted);
    scale_configure(ud, "VideoQualitySlider", val, vqmin, vqmax,
                    step, page, digits, inverted);
}

G_MODULE_EXPORT void
start_point_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    gint start, end;

    ghb_widget_to_setting(ud->settings, widget);
    if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start > end)
            ghb_ui_update(ud, "end_point", ghb_int_value(start));
        ghb_check_dependency(ud, widget, NULL);
        if (ghb_settings_get_boolean(ud->prefs, "chapters_in_destination"))
        {
            set_destination(ud);
        }
        widget = GHB_WIDGET (ud->builder, "ChapterMarkers");
        // End may have been changed above, get it again
        end = ghb_settings_get_int(ud->settings, "end_point");
        gtk_widget_set_sensitive(widget, end > start);
        update_title_duration(ud);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 1)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start >= end)
            ghb_ui_update(ud, "end_point", ghb_int_value(start+1));
        ghb_check_dependency(ud, widget, NULL);
        update_title_duration(ud);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 2)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start > end)
            ghb_ui_update(ud, "end_point", ghb_int_value(start));
        ghb_check_dependency(ud, widget, NULL);
        update_title_duration(ud);
    }
}

G_MODULE_EXPORT void
end_point_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    gint start, end;

    ghb_widget_to_setting(ud->settings, widget);
    if (ghb_settings_combo_int(ud->settings, "PtoPType") == 0)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start > end)
            ghb_ui_update(ud, "start_point", ghb_int_value(end));
        ghb_check_dependency(ud, widget, NULL);
        if (ghb_settings_get_boolean(ud->prefs, "chapters_in_destination"))
        {
            set_destination(ud);
        }
        widget = GHB_WIDGET (ud->builder, "ChapterMarkers");
        // Start may have been changed above, get it again
        start = ghb_settings_get_int(ud->settings, "start_point");
        gtk_widget_set_sensitive(widget, end > start);
        update_title_duration(ud);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 1)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start >= end)
            ghb_ui_update(ud, "start_point", ghb_int_value(end-1));
        ghb_check_dependency(ud, widget, NULL);
        update_title_duration(ud);
    }
    else if (ghb_settings_combo_int(ud->settings, "PtoPType") == 2)
    {
        start = ghb_settings_get_int(ud->settings, "start_point");
        end = ghb_settings_get_int(ud->settings, "end_point");
        if (start > end)
            ghb_ui_update(ud, "start_point", ghb_int_value(end));
        ghb_check_dependency(ud, widget, NULL);
        update_title_duration(ud);
    }
}

G_MODULE_EXPORT void
scale_width_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("scale_width_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, GHB_PIC_KEEP_WIDTH);
    update_preview = TRUE;
    ghb_live_reset(ud);

    update_scale_info(ud);
}

G_MODULE_EXPORT void
scale_height_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("scale_height_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, GHB_PIC_KEEP_HEIGHT);

    update_preview = TRUE;
    ghb_live_reset(ud);

    update_scale_info(ud);
}

G_MODULE_EXPORT void
crop_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("crop_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, 0);
    update_preview = TRUE;
    ghb_live_reset(ud);

    update_crop_info(ud);
}

G_MODULE_EXPORT void
display_width_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("display_width_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, GHB_PIC_KEEP_DISPLAY_WIDTH);

    update_preview = TRUE;
}

G_MODULE_EXPORT void
display_height_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("display_height_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, GHB_PIC_KEEP_DISPLAY_HEIGHT);

    update_preview = TRUE;
}

G_MODULE_EXPORT void
par_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("par_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, GHB_PIC_KEEP_PAR);

    update_preview = TRUE;
}

G_MODULE_EXPORT void
scale_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("scale_changed_cb ()");
    ghb_widget_to_setting(ud->settings, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_clear_presets_selection(ud);
    ghb_live_reset(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, 0);
    update_preview = TRUE;

    update_aspect_info(ud);
}

G_MODULE_EXPORT void
show_crop_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("show_crop_changed_cb ()");
    ghb_widget_to_setting(ud->prefs, widget);
    ghb_check_dependency(ud, widget, NULL);
    ghb_live_reset(ud);
    if (gtk_widget_is_sensitive(widget))
        ghb_set_scale(ud, 0);
    ghb_pref_save(ud->prefs, "preview_show_crop");
    update_preview = TRUE;
}

G_MODULE_EXPORT void
generic_entry_changed_cb(GtkEntry *entry, signal_user_data_t *ud)
{
    // Normally (due to user input) I only want to process the entry
    // when editing is done and the focus-out signal is sent.
    // But... there's always a but.
    // If the entry is changed by software, the focus-out signal is not sent.
    // The changed signal is sent ... so here we are.
    // I don't want to process upon every keystroke, so I prevent processing
    // while the widget has focus.
    g_debug("generic_entry_changed_cb ()");
    if (!gtk_widget_has_focus((GtkWidget*)entry))
    {
        ghb_widget_to_setting(ud->settings, (GtkWidget*)entry);
    }
}

G_MODULE_EXPORT void
prefs_dialog_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *dialog;

    g_debug("prefs_dialog_cb ()");
    dialog = GHB_WIDGET(ud->builder, "prefs_dialog");
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_hide(dialog);
    ghb_prefs_store();
}

typedef struct
{
    GtkMessageDialog *dlg;
    const gchar *msg;
    const gchar *action;
    gint timeout;
    signal_user_data_t *ud;
} countdown_t;

static gboolean
quit_cb(countdown_t *cd)
{
    gchar *str;

    cd->timeout--;
    if (cd->timeout == 0)
    {
        ghb_hb_cleanup(FALSE);
        prune_logs(cd->ud);

        gtk_widget_destroy (GTK_WIDGET(cd->dlg));
        gtk_main_quit();
        return FALSE;
    }
    str = g_strdup_printf(_("%s\n\n%s in %d seconds ..."),
                            cd->msg, cd->action, cd->timeout);
    gtk_message_dialog_set_markup(cd->dlg, str);
    g_free(str);
    return TRUE;
}

static gboolean
shutdown_cb(countdown_t *cd)
{
    gchar *str;

    cd->timeout--;
    if (cd->timeout == 0)
    {
        ghb_hb_cleanup(FALSE);
        prune_logs(cd->ud);

        ghb_shutdown_gsm();
        gtk_main_quit();
        return FALSE;
    }
    str = g_strdup_printf(_("%s\n\n%s in %d seconds ..."),
                            cd->msg, cd->action, cd->timeout);
    gtk_message_dialog_set_markup(cd->dlg, str);
    g_free(str);
    return TRUE;
}

static gboolean
suspend_cb(countdown_t *cd)
{
    gchar *str;

    cd->timeout--;
    if (cd->timeout == 0)
    {
        gtk_widget_destroy (GTK_WIDGET(cd->dlg));
        ghb_suspend_gpm();
        return FALSE;
    }
    str = g_strdup_printf(_("%s\n\n%s in %d seconds ..."),
                            cd->msg, cd->action, cd->timeout);
    gtk_message_dialog_set_markup(cd->dlg, str);
    g_free(str);
    return TRUE;
}

void
ghb_countdown_dialog(
    GtkMessageType type,
    const gchar *message,
    const gchar *action,
    const gchar *cancel,
    GSourceFunc action_func,
    signal_user_data_t *ud,
    gint timeout)
{
    GtkWidget *dialog;
    GtkResponseType response;
    guint timeout_id;
    countdown_t cd;

    cd.msg = message;
    cd.action = action;
    cd.timeout = timeout;
    cd.ud = ud;

    // Toss up a warning dialog
    dialog = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                            type, GTK_BUTTONS_NONE,
                            _("%s\n\n%s in %d seconds ..."),
                            message, action, timeout);
    gtk_dialog_add_buttons( GTK_DIALOG(dialog),
                           cancel, GTK_RESPONSE_CANCEL,
                           NULL);

    cd.dlg = GTK_MESSAGE_DIALOG(dialog);
    timeout_id = g_timeout_add(1000, action_func, &cd);
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (dialog);
    if (response == GTK_RESPONSE_CANCEL)
    {
        GMainContext *mc;
        GSource *source;

        mc = g_main_context_default();
        source = g_main_context_find_source_by_id(mc, timeout_id);
        if (source != NULL)
            g_source_destroy(source);
    }
}

gboolean
ghb_message_dialog(GtkMessageType type, const gchar *message, const gchar *no, const gchar *yes)
{
    GtkWidget *dialog;
    GtkResponseType response;

    // Toss up a warning dialog
    dialog = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                            type, GTK_BUTTONS_NONE,
                            "%s", message);
    gtk_dialog_add_buttons( GTK_DIALOG(dialog),
                           no, GTK_RESPONSE_NO,
                           yes, GTK_RESPONSE_YES, NULL);
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (dialog);
    if (response == GTK_RESPONSE_NO)
    {
        return FALSE;
    }
    return TRUE;
}

void
ghb_error_dialog(GtkMessageType type, const gchar *message, const gchar *cancel)
{
    GtkWidget *dialog;

    // Toss up a warning dialog
    dialog = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                            type, GTK_BUTTONS_NONE,
                            "%s", message);
    gtk_dialog_add_buttons( GTK_DIALOG(dialog),
                           cancel, GTK_RESPONSE_CANCEL, NULL);
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (dialog);
}

void
ghb_cancel_encode(signal_user_data_t *ud, const gchar *extra_msg)
{
    GtkWidget *dialog;
    GtkResponseType response;

    if (extra_msg == NULL) extra_msg = "";
    // Toss up a warning dialog
    dialog = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                GTK_MESSAGE_WARNING, GTK_BUTTONS_NONE,
                _("%sYour movie will be lost if you don't continue encoding."),
                extra_msg);
    gtk_dialog_add_buttons( GTK_DIALOG(dialog),
                           _("Cancel Current and Stop"), 1,
                           _("Cancel Current, Start Next"), 2,
                           _("Finish Current, then Stop"), 3,
                           _("Continue Encoding"), 4,
                           NULL);
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (dialog);
    switch ((int)response)
    {
        case 1:
            ghb_stop_queue();
            ud->cancel_encode = GHB_CANCEL_ALL;
            break;
        case 2:
            ghb_stop_queue();
            ud->cancel_encode = GHB_CANCEL_CURRENT;
            break;
        case 3:
            ud->cancel_encode = GHB_CANCEL_FINISH;
            break;
        case 4:
        default:
            ud->cancel_encode = GHB_CANCEL_NONE;
            break;
    }
}

gboolean
ghb_cancel_encode2(signal_user_data_t *ud, const gchar *extra_msg)
{
    GtkWidget *dialog;
    GtkResponseType response;

    if (extra_msg == NULL) extra_msg = "";
    // Toss up a warning dialog
    dialog = gtk_message_dialog_new(NULL, GTK_DIALOG_MODAL,
                GTK_MESSAGE_WARNING, GTK_BUTTONS_NONE,
                _("%sYour movie will be lost if you don't continue encoding."),
                extra_msg);
    gtk_dialog_add_buttons( GTK_DIALOG(dialog),
                           _("Cancel Current and Stop"), 1,
                           _("Continue Encoding"), 4,
                           NULL);
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy (dialog);
    switch ((int)response)
    {
        case 1:
            ghb_stop_queue();
            ud->cancel_encode = GHB_CANCEL_ALL;
            return TRUE;
        case 4:
        default:
            break;
    }
    return FALSE;
}

static void
submit_job(GValue *settings)
{
    static gint unique_id = 1;
    gchar *type, *modified, *preset;
    const GValue *path;
    gboolean preset_modified;

    g_debug("submit_job");
    if (settings == NULL) return;
    preset_modified = ghb_settings_get_boolean(settings, "preset_modified");
    path = ghb_settings_get_value(settings, "preset");
    preset = ghb_preset_path_string(path);
    type = ghb_preset_is_custom() ? "Custom " : "";
    modified = preset_modified ? "Modified " : "";
    ghb_log("%s%sPreset: %s", modified, type, preset);
    g_free(preset);

    ghb_settings_set_int(settings, "job_unique_id", unique_id);
    ghb_settings_set_int(settings, "job_status", GHB_QUEUE_RUNNING);
    ghb_add_job (settings, unique_id);
    ghb_start_queue();
    unique_id++;
}

static void
prune_logs(signal_user_data_t *ud)
{
    gchar *dest_dir;
    gint days;

    // Only prune logs stored in the default config dir location
    days = ghb_settings_combo_int(ud->prefs, "LogLongevity");
    if (days > 365)
        return;

    dest_dir = ghb_get_user_config_dir("EncodeLogs");
    if (g_file_test(dest_dir, G_FILE_TEST_IS_DIR))
    {
        const gchar *file;
        gint duration = days * 24 * 60 * 60;

        GDir *gdir = g_dir_open(dest_dir, 0, NULL);
        time_t now;

        now = time(NULL);
        file = g_dir_read_name(gdir);
        while (file)
        {
            gchar *path;
            struct stat stbuf;

            path = g_strdup_printf("%s/%s", dest_dir, file);
            g_stat(path, &stbuf);
            if (now - stbuf.st_mtime > duration)
            {
                g_unlink(path);
            }
            g_free(path);
            file = g_dir_read_name(gdir);
        }
        g_dir_close(gdir);
    }
    g_free(dest_dir);
    ghb_preview_cleanup(ud);
}

static void
queue_scan(signal_user_data_t *ud, GValue *js)
{
    gchar *path;
    gint titlenum;
    time_t  _now;
    struct tm *now;
    gchar *log_path, *pos, *destname, *basename, *dest_dir;

    _now = time(NULL);
    now = localtime(&_now);
    destname = ghb_settings_get_string(js, "destination");
    basename = g_path_get_basename(destname);
    if (ghb_settings_get_boolean(ud->prefs, "EncodeLogLocation"))
    {
        dest_dir = g_path_get_dirname (destname);
    }
    else
    {
        dest_dir = ghb_get_user_config_dir("EncodeLogs");
    }
    g_free(destname);
    pos = g_strrstr( basename, "." );
    if (pos != NULL)
    {
        *pos = 0;
    }
    log_path = g_strdup_printf("%s/%s %d-%02d-%02d %02d-%02d-%02d.log",
        dest_dir,
        basename,
        now->tm_year + 1900, now->tm_mon + 1, now->tm_mday,
        now->tm_hour, now->tm_min, now->tm_sec);
    g_free(basename);
    g_free(dest_dir);
    if (ud->job_activity_log)
        g_io_channel_unref(ud->job_activity_log);
    ud->job_activity_log = g_io_channel_new_file (log_path, "w", NULL);
    if (ud->job_activity_log)
    {
        gchar *ver_str;

        ver_str = g_strdup_printf("Handbrake Version: %s (%d)\n",
                                    hb_get_version(NULL), hb_get_build(NULL));
        g_io_channel_write_chars (ud->job_activity_log, ver_str,
                                    -1, NULL, NULL);
        g_free(ver_str);
    }
    g_free(log_path);

    path = ghb_settings_get_string( js, "source");
    titlenum = ghb_settings_get_int(js, "titlenum");
    ghb_backend_queue_scan(path, titlenum);
    g_free(path);
}

static gint
queue_pending_count(GValue *queue)
{
    gint nn, ii, count;
    GValue *js;
    gint status;

    nn = 0;
    count = ghb_array_len(queue);
    for (ii = 0; ii < count; ii++)
    {

        js = ghb_array_get_nth(queue, ii);
        status = ghb_settings_get_int(js, "job_status");
        if (status == GHB_QUEUE_PENDING)
        {
            nn++;
        }
    }
    return nn;
}

void
ghb_update_pending(signal_user_data_t *ud)
{
    GtkLabel *label;
    gint pending;
    gchar *str;

    label = GTK_LABEL(GHB_WIDGET(ud->builder, "pending_status"));
    pending = queue_pending_count(ud->queue);
    str = g_strdup_printf("%d encode(s) pending", pending);
    gtk_label_set_text(label, str);
    g_free(str);
}

GValue*
ghb_start_next_job(signal_user_data_t *ud, gboolean find_first)
{
    static gint current = 0;
    gint count, ii, jj;
    GValue *js;
    gint status;
    GtkWidget *prog;

    g_debug("start_next_job");
    prog = GHB_WIDGET(ud->builder, "progressbar");
    gtk_widget_show(prog);

    count = ghb_array_len(ud->queue);
    if (find_first)
    {   // Start the first pending item in the queue
        current = 0;
        for (ii = 0; ii < count; ii++)
        {

            js = ghb_array_get_nth(ud->queue, ii);
            status = ghb_settings_get_int(js, "job_status");
            if (status == GHB_QUEUE_PENDING)
            {
                current = ii;
                ghb_inhibit_gsm(ud);
                queue_scan(ud, js);
                ghb_update_pending(ud);
                return js;
            }
        }
        // Nothing pending
        ghb_uninhibit_gsm();
        ghb_notify_done(ud);
        return NULL;
    }
    // Find the next pending item after the current running item
    for (ii = 0; ii < count-1; ii++)
    {
        js = ghb_array_get_nth(ud->queue, ii);
        status = ghb_settings_get_int(js, "job_status");
        if (status == GHB_QUEUE_RUNNING)
        {
            for (jj = ii+1; jj < count; jj++)
            {
                js = ghb_array_get_nth(ud->queue, jj);
                status = ghb_settings_get_int(js, "job_status");
                if (status == GHB_QUEUE_PENDING)
                {
                    current = jj;
                    ghb_inhibit_gsm(ud);
                    queue_scan(ud, js);
                    ghb_update_pending(ud);
                    return js;
                }
            }
        }
    }
    // No running item found? Maybe it was deleted
    // Look for a pending item starting from the last index we started
    for (ii = current; ii < count; ii++)
    {
        js = ghb_array_get_nth(ud->queue, ii);
        status = ghb_settings_get_int(js, "job_status");
        if (status == GHB_QUEUE_PENDING)
        {
            current = ii;
            ghb_inhibit_gsm(ud);
            queue_scan(ud, js);
            ghb_update_pending(ud);
            return js;
        }
    }
    // Nothing found
    ghb_uninhibit_gsm();
    ghb_notify_done(ud);
    ghb_update_pending(ud);
    gtk_widget_hide(prog);
    return NULL;
}

static gint
find_queue_job(GValue *queue, gint unique_id, GValue **job)
{
    GValue *js;
    gint ii, count;
    gint job_unique_id;

    *job = NULL;
    g_debug("find_queue_job");
    if (unique_id == 0)  // Invalid Id
        return -1;

    count = ghb_array_len(queue);
    for (ii = 0; ii < count; ii++)
    {
        js = ghb_array_get_nth(queue, ii);
        job_unique_id = ghb_settings_get_int(js, "job_unique_id");
        if (job_unique_id == unique_id)
        {
            *job = js;
            return ii;
        }
    }
    return -1;
}

gchar*
working_status_string(signal_user_data_t *ud, ghb_instance_status_t *status)
{
    gchar *task_str, *job_str, *status_str;
    gint qcount;
    gint index;
    GValue *js;
    gboolean subtitle_scan = FALSE;

    qcount = ghb_array_len(ud->queue);
    index = find_queue_job(ud->queue, status->unique_id, &js);
    if (js != NULL)
    {
        subtitle_scan = ghb_settings_get_boolean(js, "subtitle_scan");
    }
    if (qcount > 1)
    {
        job_str = g_strdup_printf(_("job %d of %d, "), index+1, qcount);
    }
    else
    {
        job_str = g_strdup("");
    }
    if (status->job_count > 1)
    {
        if (status->job_cur == 1 && subtitle_scan)
        {
            task_str = g_strdup_printf(_("pass %d (subtitle scan) of %d, "),
                status->job_cur, status->job_count);
        }
        else
        {
            task_str = g_strdup_printf(_("pass %d of %d, "),
                status->job_cur, status->job_count);
        }
    }
    else
    {
        task_str = g_strdup("");
    }
    if(status->seconds > -1)
    {
        if (status->rate_cur > 0.0)
        {
            status_str= g_strdup_printf(
                _("Encoding: %s%s%.2f %%"
                " (%.2f fps, avg %.2f fps, ETA %02dh%02dm%02ds)"),
                job_str, task_str,
                100.0 * status->progress,
                status->rate_cur, status->rate_avg, status->hours,
                status->minutes, status->seconds );
        }
        else
        {
            status_str= g_strdup_printf(
                _("Encoding: %s%s%.2f %%"
                " (ETA %02dh%02dm%02ds)"),
                job_str, task_str,
                100.0 * status->progress,
                status->hours, status->minutes, status->seconds );
        }
    }
    else
    {
        status_str= g_strdup_printf(
            _("Encoding: %s%s%.2f %%"),
            job_str, task_str,
            100.0 * status->progress );
    }
    g_free(task_str);
    g_free(job_str);
    return status_str;
}

gchar*
searching_status_string(signal_user_data_t *ud, ghb_instance_status_t *status)
{
    gchar *task_str, *job_str, *status_str;
    gint qcount;
    gint index;
    GValue *js;

    qcount = ghb_array_len(ud->queue);
    index = find_queue_job(ud->queue, status->unique_id, &js);
    if (qcount > 1)
    {
        job_str = g_strdup_printf(_("job %d of %d, "), index+1, qcount);
    }
    else
    {
        job_str = g_strdup("");
    }
    task_str = g_strdup_printf(_("Searching for start time, "));
    if(status->seconds > -1)
    {
        status_str= g_strdup_printf(
            _("Encoding: %s%s%.2f %%"
            " (ETA %02dh%02dm%02ds)"),
            job_str, task_str,
            100.0 * status->progress,
            status->hours, status->minutes, status->seconds );
    }
    else
    {
        status_str= g_strdup_printf(
            _("Encoding: %s%s%.2f %%"),
            job_str, task_str,
            100.0 * status->progress );
    }
    g_free(task_str);
    g_free(job_str);
    return status_str;
}

static void
ghb_backend_events(signal_user_data_t *ud)
{
    ghb_status_t status;
    gchar *status_str;
    GtkProgressBar *progress;
    GtkLabel       *work_status;
    gint titleindex;
    GValue *js;
    gint index;
    GtkTreeView *treeview;
    GtkTreeStore *store;
    GtkTreeIter iter;
    static gint prev_scan_state = -1;
    static gint prev_queue_state = -1;

    ghb_track_status();
    ghb_get_status(&status);
    if (prev_scan_state != status.scan.state ||
        prev_queue_state != status.queue.state)
    {
        ghb_queue_buttons_grey(ud);
        prev_scan_state = status.scan.state;
        prev_queue_state = status.queue.state;
    }
    progress = GTK_PROGRESS_BAR(GHB_WIDGET (ud->builder, "progressbar"));
    work_status = GTK_LABEL(GHB_WIDGET (ud->builder, "work_status"));
    if (status.scan.state == GHB_STATE_IDLE &&
        status.queue.state == GHB_STATE_IDLE)
    {
        static gboolean prev_dvdnav;
        gboolean dvdnav = ghb_settings_get_boolean(ud->prefs, "use_dvdnav");
        if (dvdnav != prev_dvdnav)
        {
            hb_dvd_set_dvdnav(dvdnav);
            prev_dvdnav = dvdnav;
        }
    }
    // First handle the status of title scans
    // Then handle the status of the queue
    if (status.scan.state & GHB_STATE_SCANNING)
    {
        GtkProgressBar *scan_prog;
        GtkLabel *label;

        scan_prog = GTK_PROGRESS_BAR(GHB_WIDGET (ud->builder, "scan_prog"));
        label = GTK_LABEL(GHB_WIDGET (ud->builder, "volume_label"));

        if (status.scan.title_cur == 0)
        {
            status_str = g_strdup (_("Scanning..."));
        }
        else
        {
            if (status.scan.preview_cur == 0)
                status_str = g_strdup_printf(_("Scanning title %d of %d..."),
                              status.scan.title_cur, status.scan.title_count );
            else
                status_str = g_strdup_printf(
                    _("Scanning title %d of %d preview %d..."),
                    status.scan.title_cur, status.scan.title_count,
                    status.scan.preview_cur);

        }
        gtk_label_set_text (label, status_str);
        g_free(status_str);
        if (status.scan.title_count > 0)
        {
            gtk_progress_bar_set_fraction (scan_prog, status.scan.progress);
        }
    }
    else if (status.scan.state & GHB_STATE_SCANDONE)
    {
        gchar *source;
        GtkProgressBar *scan_prog;
        GtkLabel *label;

        GtkWidget *widget;

        widget = GHB_WIDGET(ud->builder, "sourcetoolbutton");
        gtk_tool_button_set_icon_name(GTK_TOOL_BUTTON(widget), "hb-source");
        gtk_tool_button_set_label(GTK_TOOL_BUTTON(widget), _("Source"));
        gtk_tool_item_set_tooltip_text(GTK_TOOL_ITEM(widget), _("Choose Video Source"));

        widget = GHB_WIDGET(ud->builder, "source_open");
        gtk_widget_set_sensitive(widget, TRUE);
        widget = GHB_WIDGET(ud->builder, "source_title_open");
        gtk_widget_set_sensitive(widget, TRUE);

        source = ghb_settings_get_string(ud->globals, "scan_source");
        update_source_label(ud, source);
        g_free(source);

        scan_prog = GTK_PROGRESS_BAR(GHB_WIDGET (ud->builder, "scan_prog"));
        gtk_progress_bar_set_fraction (scan_prog, 1.0);
        gtk_widget_hide(GTK_WIDGET(scan_prog));

        ghb_update_ui_combo_box(ud, "title", 0, FALSE);

        titleindex = ghb_longest_title();
        load_all_titles(ud, titleindex);

        label = GTK_LABEL(GHB_WIDGET (ud->builder, "volume_label"));
        // Are there really any titles.
        hb_title_t * title = ghb_get_title_info(titleindex);
        if (title == NULL)
        {
            gtk_label_set_text(label, _("No Title Found"));
        }
        ghb_clear_scan_state(GHB_STATE_SCANDONE);

        ghb_ui_update(ud, "title", ghb_int64_value(titleindex));

        if (ghb_queue_edit_settings != NULL)
        {
            // Switch to the correct title in the list
            ghb_ui_update(ud, "title",
                ghb_settings_get_value(ghb_queue_edit_settings, "title"));

            // The above should cause the current title index to update
            int titleindex = ghb_settings_get_int(ud->settings, "title_no");
            ghb_array_replace(ud->settings_array, titleindex,
                              ghb_queue_edit_settings);
            ud->settings = ghb_queue_edit_settings;
            ghb_load_settings(ud);
            ghb_queue_edit_settings = NULL;
        }
    }

    if (status.queue.state & GHB_STATE_SCANNING)
    {
        // This needs to be in scanning and working since scanning
        // happens fast enough that it can be missed
        gtk_label_set_text (work_status, _("Scanning ..."));
        gtk_progress_bar_set_fraction (progress, 0);
    }
    else if (status.queue.state & GHB_STATE_SCANDONE)
    {
        ghb_clear_queue_state(GHB_STATE_SCANDONE);
        usleep(2000000);
        submit_job(ud->current_job);
        ghb_update_pending(ud);
    }
    else if (status.queue.state & GHB_STATE_PAUSED)
    {
        gtk_label_set_text (work_status, _("Paused"));
    }
    else if (status.queue.state & GHB_STATE_SEARCHING)
    {
        static gint working = 0;

        // This needs to be in scanning and working since scanning
        // happens fast enough that it can be missed
        index = find_queue_job(ud->queue, status.queue.unique_id, &js);
        if (status.queue.unique_id != 0 && index >= 0)
        {
            gchar working_icon[] = "hb-working0";
            working_icon[10] = '0' + working;
            working = (working+1) % 6;
            treeview = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "queue_list"));
            store = GTK_TREE_STORE(gtk_tree_view_get_model(treeview));
            gchar *path = g_strdup_printf ("%d", index);
            if (gtk_tree_model_get_iter_from_string(
                    GTK_TREE_MODEL(store), &iter, path))
            {
                gtk_tree_store_set(store, &iter, 0, working_icon, -1);
            }
            g_free(path);
        }
        GtkLabel *label;
        gchar *status_str;

        status_str = searching_status_string(ud, &status.queue);
        label = GTK_LABEL(GHB_WIDGET(ud->builder, "queue_status"));
        gtk_label_set_text (label, status_str);
        if (ghb_settings_get_boolean(ud->prefs, "show_status"))
        {
#if !GTK_CHECK_VERSION(2, 16, 0)
            GtkStatusIcon *si;

            si = GTK_STATUS_ICON(GHB_OBJECT(ud->builder, "hb_status"));
            gtk_status_icon_set_tooltip(si, status_str);
#endif
#if defined(_USE_APP_IND)
            char * ai_status_str= g_strdup_printf(
                "%.2f%%",
                100.0 * status.queue.progress);
            app_indicator_set_label( ud->ai, ai_status_str, "99.99%");
            g_free(ai_status_str);
#endif
        }
        gtk_label_set_text (work_status, status_str);
        gtk_progress_bar_set_fraction (progress, status.queue.progress);
        g_free(status_str);
    }
    else if (status.queue.state & GHB_STATE_WORKING)
    {
        static gint working = 0;

        // This needs to be in scanning and working since scanning
        // happens fast enough that it can be missed
        index = find_queue_job(ud->queue, status.queue.unique_id, &js);
        if (status.queue.unique_id != 0 && index >= 0)
        {
            gchar working_icon[] = "hb-working0";
            working_icon[10] = '0' + working;
            working = (working+1) % 6;
            treeview = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "queue_list"));
            store = GTK_TREE_STORE(gtk_tree_view_get_model(treeview));
            gchar *path = g_strdup_printf ("%d", index);
            if (gtk_tree_model_get_iter_from_string(
                    GTK_TREE_MODEL(store), &iter, path))
            {
                gtk_tree_store_set(store, &iter, 0, working_icon, -1);
            }
            g_free(path);
        }
        GtkLabel *label;
        gchar *status_str;

        status_str = working_status_string(ud, &status.queue);
        label = GTK_LABEL(GHB_WIDGET(ud->builder, "queue_status"));
        gtk_label_set_text (label, status_str);
        if (ghb_settings_get_boolean(ud->prefs, "show_status"))
        {
#if defined(_USE_APP_IND)
            char * ai_status_str= g_strdup_printf(
                "%.2f%%",
                100.0 * status.queue.progress);
            app_indicator_set_label( ud->ai, ai_status_str, "99.99%");
            g_free(ai_status_str);
#else
#if !GTK_CHECK_VERSION(2, 16, 0)
            GtkStatusIcon *si;

            si = GTK_STATUS_ICON(GHB_OBJECT(ud->builder, "hb_status"));
            gtk_status_icon_set_tooltip(si, status_str);
#endif
#endif
        }
        gtk_label_set_text (work_status, status_str);
        gtk_progress_bar_set_fraction (progress, status.queue.progress);
        g_free(status_str);
    }
    else if (status.queue.state & GHB_STATE_WORKDONE)
    {
        gint qstatus;

        index = find_queue_job(ud->queue, status.queue.unique_id, &js);
        treeview = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "queue_list"));
        store = GTK_TREE_STORE(gtk_tree_view_get_model(treeview));
        if (ud->cancel_encode == GHB_CANCEL_ALL ||
            ud->cancel_encode == GHB_CANCEL_CURRENT)
            status.queue.error = GHB_ERROR_CANCELED;
        switch( status.queue.error )
        {
            case GHB_ERROR_NONE:
                gtk_label_set_text (work_status, _("Encode Done!"));
                qstatus = GHB_QUEUE_DONE;
                if (js != NULL)
                {
                    gchar *path = g_strdup_printf ("%d", index);
                    if (gtk_tree_model_get_iter_from_string(
                            GTK_TREE_MODEL(store), &iter, path))
                    {
                        gtk_tree_store_set(store, &iter, 0, "hb-complete", -1);
                    }
                    g_free(path);
                }
                break;
            case GHB_ERROR_CANCELED:
                gtk_label_set_text (work_status, _("Encode Canceled."));
                qstatus = GHB_QUEUE_CANCELED;
                if (js != NULL)
                {
                    gchar *path = g_strdup_printf ("%d", index);
                    if (gtk_tree_model_get_iter_from_string(
                            GTK_TREE_MODEL(store), &iter, path))
                    {
                        gtk_tree_store_set(store, &iter, 0, "hb-canceled", -1);
                    }
                    g_free(path);
                }
                break;
            default:
                gtk_label_set_text (work_status, _("Encode Failed."));
                qstatus = GHB_QUEUE_CANCELED;
                if (js != NULL)
                {
                    gchar *path = g_strdup_printf ("%d", index);
                    if (gtk_tree_model_get_iter_from_string(
                            GTK_TREE_MODEL(store), &iter, path))
                    {
                        gtk_tree_store_set(store, &iter, 0, "hb-canceled", -1);
                    }
                    g_free(path);
                }
        }
        gtk_progress_bar_set_fraction (progress, 1.0);
        ghb_clear_queue_state(GHB_STATE_WORKDONE);
        if (ud->job_activity_log)
            g_io_channel_unref(ud->job_activity_log);
        ud->job_activity_log = NULL;
        if (ud->cancel_encode != GHB_CANCEL_ALL &&
            ud->cancel_encode != GHB_CANCEL_FINISH)
        {
            ud->current_job = ghb_start_next_job(ud, FALSE);
        }
        else
        {
            ghb_uninhibit_gsm();
            ud->current_job = NULL;
            gtk_widget_hide(GTK_WIDGET(progress));
        }
        if (js)
            ghb_settings_set_int(js, "job_status", qstatus);
        ghb_save_queue(ud->queue);
        ud->cancel_encode = GHB_CANCEL_NONE;
#if defined(_USE_APP_IND)
        app_indicator_set_label( ud->ai, "", "99.99%");
#else
#if !GTK_CHECK_VERSION(2, 16, 0)
        GtkStatusIcon *si;

        si = GTK_STATUS_ICON(GHB_OBJECT(ud->builder, "hb_status"));
        gtk_status_icon_set_tooltip(si, "HandBrake");
#endif
#endif
    }
    else if (status.queue.state & GHB_STATE_MUXING)
    {
        gtk_label_set_text (work_status, _("Muxing: This may take a while..."));
    }

    if (status.scan.state & GHB_STATE_WORKING)
    {
        GtkProgressBar *live_progress;
        live_progress = GTK_PROGRESS_BAR(
            GHB_WIDGET(ud->builder, "live_encode_progress"));
        status_str = working_status_string(ud, &status.scan);
        gtk_progress_bar_set_text (live_progress, status_str);
        gtk_progress_bar_set_fraction (live_progress, status.scan.progress);
        g_free(status_str);
    }
    if (status.scan.state & GHB_STATE_WORKDONE)
    {
        switch( status.scan.error )
        {
            case GHB_ERROR_NONE:
            {
                ghb_live_encode_done(ud, TRUE);
            } break;
            default:
            {
                ghb_live_encode_done(ud, FALSE);
            } break;
        }
        ghb_clear_scan_state(GHB_STATE_WORKDONE);
    }
}

#if GTK_CHECK_VERSION(2, 16, 0)
G_MODULE_EXPORT gboolean
status_icon_query_tooltip_cb(
    GtkStatusIcon *si,
    gint           x,
    gint           y,
    gboolean       kbd_mode,
    GtkTooltip    *tt,
    signal_user_data_t *ud)
{
    ghb_status_t status;
    gchar *status_str;

    ghb_get_status(&status);
    if (status.queue.state & GHB_STATE_WORKING)
        status_str = working_status_string(ud, &status.queue);
    else if (status.queue.state & GHB_STATE_SEARCHING)
        status_str = searching_status_string(ud, &status.queue);
    else if (status.queue.state & GHB_STATE_WORKDONE)
        status_str = g_strdup(_("Encode Complete"));
    else
        status_str = g_strdup("HandBrake");

    gtk_tooltip_set_text(tt, status_str);
    gtk_tooltip_set_icon_from_icon_name(tt, "hb-icon", GTK_ICON_SIZE_BUTTON);
    g_free(status_str);
    return TRUE;
}
#endif

G_MODULE_EXPORT gboolean
ghb_timer_cb(gpointer data)
{
    signal_user_data_t *ud = (signal_user_data_t*)data;

    ghb_live_preview_progress(ud);
    ghb_backend_events(ud);
    if (update_default_destination)
    {
        gchar *dest, *dest_dir, *def_dest;
        dest = ghb_settings_get_string(ud->settings, "destination");
        dest_dir = g_path_get_dirname (dest);
        def_dest = ghb_settings_get_string(ud->prefs, "destination_dir");
        if (strcmp(dest_dir, def_dest) != 0)
        {
            ghb_settings_set_string (ud->prefs, "destination_dir", dest_dir);
            ghb_pref_save(ud->prefs, "destination_dir");
        }
        g_free(dest);
        g_free(dest_dir);
        g_free(def_dest);
        update_default_destination = FALSE;
    }
    if (update_preview)
    {
        g_debug("Updating preview\n");
        ghb_set_preview_image (ud);
        update_preview = FALSE;
    }

#if !defined(_NO_UPDATE_CHECK)
    if (!appcast_busy)
    {
        gchar *updates;
        updates = ghb_settings_get_string(ud->prefs, "check_updates");
        gint64 duration = 0;
        if (strcmp(updates, "daily") == 0)
            duration = 60 * 60 * 24;
        else if (strcmp(updates, "weekly") == 0)
            duration = 60 * 60 * 24 * 7;
        else if (strcmp(updates, "monthly") == 0)
            duration = 60 * 60 * 24 * 7;

        g_free(updates);
        if (duration != 0)
        {
            gint64 last;
            time_t tt;

            last = ghb_settings_get_int64(ud->prefs, "last_update_check");
            time(&tt);
            if (last + duration < tt)
            {
                ghb_settings_set_int64(ud->prefs,
                                        "last_update_check", tt);
                ghb_pref_save(ud->prefs, "last_update_check");
                GHB_THREAD_NEW("Update Check", (GThreadFunc)ghb_check_update, ud);
            }
        }
    }
#endif
    return TRUE;
}

G_MODULE_EXPORT gboolean
ghb_log_cb(GIOChannel *source, GIOCondition cond, gpointer data)
{
    gchar *text = NULL;
    gsize length, outlength;
    GtkTextView *textview;
    GtkTextBuffer *buffer;
    GtkTextIter iter;
    GtkTextMark *mark;
    GError *gerror = NULL;
    GIOStatus status;

    signal_user_data_t *ud = (signal_user_data_t*)data;

    status = g_io_channel_read_line (source, &text, &length, NULL, &gerror);
    // Trim nils from end of text, they cause g_io_channel_write_chars to
    // fail with an assertion that aborts
    while (length > 0 && text[length-1] == 0)
        length--;
    if (text != NULL && length > 0)
    {
        GdkWindow *window;
        gint width, height;
        gint x, y;
        gboolean bottom = FALSE;
        gchar *utf8_text;

        textview = GTK_TEXT_VIEW(GHB_WIDGET (ud->builder, "activity_view"));
        buffer = gtk_text_view_get_buffer (textview);
        // I would like to auto-scroll the window when the scrollbar
        // is at the bottom,
        // must determine whether the insert point is at
        // the bottom of the window
        window = gtk_text_view_get_window(textview, GTK_TEXT_WINDOW_TEXT);
        if (window != NULL)
        {
            width = gdk_window_get_width(window);
            height = gdk_window_get_height(window);
            gtk_text_view_window_to_buffer_coords(textview,
                GTK_TEXT_WINDOW_TEXT, width, height, &x, &y);
            gtk_text_view_get_iter_at_location(textview, &iter, x, y);
            if (gtk_text_iter_is_end(&iter))
            {
                bottom = TRUE;
            }
        }
        else
        {
            // If the window isn't available, assume bottom
            bottom = TRUE;
        }
        gtk_text_buffer_get_end_iter(buffer, &iter);
        utf8_text = g_convert_with_fallback(text, -1, "UTF-8", "ISO-8859-1",
                                            "?", NULL, &length, NULL);
        if (utf8_text != NULL)
        {
            gtk_text_buffer_insert(buffer, &iter, utf8_text, -1);
            if (bottom)
            {
                gtk_text_buffer_get_end_iter(buffer, &iter);
                mark = gtk_text_buffer_create_mark(buffer, NULL, &iter, FALSE);
                gtk_text_view_scroll_mark_onscreen(textview, mark);
                gtk_text_buffer_delete_mark(buffer, mark);
            }
#if defined(_WIN32)
            gsize one = 1;
            utf8_text[length-1] = '\r';
#endif
            g_io_channel_write_chars (ud->activity_log, utf8_text,
                                    length, &outlength, NULL);
#if defined(_WIN32)
            g_io_channel_write_chars (ud->activity_log, "\n",
                                    one, &one, NULL);
#endif
            g_io_channel_flush(ud->activity_log, NULL);
            if (ud->job_activity_log)
            {
                g_io_channel_write_chars (ud->job_activity_log, utf8_text,
                                        length, &outlength, NULL);
#if defined(_WIN32)
                g_io_channel_write_chars (ud->activity_log, "\n",
                                        one, &outlength, NULL);
#endif
                g_io_channel_flush(ud->job_activity_log, NULL);
            }
            g_free(utf8_text);
        }
    }
    if (text != NULL)
        g_free(text);

    if (status != G_IO_STATUS_NORMAL)
    {
        // This should never happen, but if it does I would get into an
        // infinite loop.  Returning false removes this callback.
        g_warning("Error while reading activity from pipe");
        if (gerror != NULL)
        {
            g_warning("%s", gerror->message);
            g_error_free (gerror);
        }
        return FALSE;
    }
    if (gerror != NULL)
        g_error_free (gerror);
    return TRUE;
}

G_MODULE_EXPORT void
show_activity_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *widget = GHB_WIDGET (ud->builder, "activity_window");
    gtk_widget_set_visible(widget, gtk_toggle_tool_button_get_active(
                        GTK_TOGGLE_TOOL_BUTTON(xwidget)));
}

G_MODULE_EXPORT void
show_activity_menu_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *widget = GHB_WIDGET (ud->builder, "activity_window");
    gtk_widget_set_visible(widget, TRUE);
    widget = GHB_WIDGET (ud->builder, "show_activity");
    gtk_toggle_tool_button_set_active(GTK_TOGGLE_TOOL_BUTTON(widget), TRUE);
}

G_MODULE_EXPORT gboolean
activity_window_delete_cb(GtkWidget *xwidget, GdkEvent *event, signal_user_data_t *ud)
{
    gtk_widget_set_visible(xwidget, FALSE);
    GtkWidget *widget = GHB_WIDGET (ud->builder, "show_activity");
    gtk_toggle_tool_button_set_active(GTK_TOGGLE_TOOL_BUTTON(widget), FALSE);
    return TRUE;
}

void
ghb_log(gchar *log, ...)
{
    va_list args;
    time_t _now;
    struct tm *now;
    gchar fmt[362];

    _now = time(NULL);
    now = localtime( &_now );
    snprintf(fmt, 362, "[%02d:%02d:%02d] gtkgui: %s\n",
            now->tm_hour, now->tm_min, now->tm_sec, log);
    va_start(args, log);
    vfprintf(stderr, fmt, args);
    va_end(args);
}

static void
browse_url(const gchar *url)
{
#if defined(_WIN32)
    ShellExecute(NULL, "open", url, NULL, NULL, SW_SHOWNORMAL);
#else
    gboolean result;
    char *argv[] =
        {"xdg-open",NULL,NULL,NULL};
    argv[1] = (gchar*)url;
    result = g_spawn_async(NULL, argv, NULL, G_SPAWN_SEARCH_PATH, NULL,
                NULL, NULL, NULL);
    if (result) return;

    argv[0] = "gnome-open";
    result = g_spawn_async(NULL, argv, NULL, G_SPAWN_SEARCH_PATH, NULL,
                NULL, NULL, NULL);
    if (result) return;

    argv[0] = "kfmclient";
    argv[1] = "exec";
    argv[2] = "http://trac.handbrake.fr/wiki/HandBrakeGuide";
    result = g_spawn_async(NULL, argv, NULL, G_SPAWN_SEARCH_PATH, NULL,
                NULL, NULL, NULL);
    if (result) return;

    argv[0] = "firefox";
    argv[1] = "http://trac.handbrake.fr/wiki/HandBrakeGuide";
    argv[2] = NULL;
    result = g_spawn_async(NULL, argv, NULL, G_SPAWN_SEARCH_PATH, NULL,
                NULL, NULL, NULL);
#endif
}

#if 0
    JJJ
void
about_web_hook(GtkAboutDialog *about, const gchar *link, gpointer data)
{
    browse_url(link);
}
#endif

G_MODULE_EXPORT void
about_activate_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *widget = GHB_WIDGET (ud->builder, "hb_about");
    gchar *ver;

    ver = g_strdup_printf("%s (%s)", HB_PROJECT_VERSION, HB_PROJECT_BUILD_ARCH);
#if 0
    JJJ
    gtk_about_dialog_set_url_hook(about_web_hook, NULL, NULL);
#endif
    gtk_about_dialog_set_version(GTK_ABOUT_DIALOG(widget), ver);
    g_free(ver);
    gtk_about_dialog_set_website(GTK_ABOUT_DIALOG(widget),
                                HB_PROJECT_URL_WEBSITE);
    gtk_about_dialog_set_website_label(GTK_ABOUT_DIALOG(widget),
                                        HB_PROJECT_URL_WEBSITE);
    gtk_widget_show (widget);
}

G_MODULE_EXPORT void
guide_activate_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    browse_url("http://trac.handbrake.fr/wiki/HandBrakeGuide");
}

G_MODULE_EXPORT void
hb_about_response_cb(GtkWidget *widget, gint response, signal_user_data_t *ud)
{
    gtk_widget_hide (widget);
}

G_MODULE_EXPORT void
show_queue_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *widget = GHB_WIDGET (ud->builder, "queue_window");
    gtk_widget_set_visible(widget, gtk_toggle_tool_button_get_active(
                        GTK_TOGGLE_TOOL_BUTTON(xwidget)));
}

G_MODULE_EXPORT void
show_queue_menu_clicked_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWidget *widget = GHB_WIDGET (ud->builder, "queue_window");
    gtk_widget_set_visible(widget, TRUE);
    widget = GHB_WIDGET (ud->builder, "show_queue");
    gtk_toggle_tool_button_set_active(GTK_TOGGLE_TOOL_BUTTON(widget), TRUE);
}

G_MODULE_EXPORT gboolean
queue_window_delete_cb(GtkWidget *xwidget, GdkEvent *event, signal_user_data_t *ud)
{
    gtk_widget_set_visible(xwidget, FALSE);
    GtkWidget *widget = GHB_WIDGET (ud->builder, "show_queue");
    gtk_toggle_tool_button_set_active(GTK_TOGGLE_TOOL_BUTTON(widget), FALSE);
    return TRUE;
}

G_MODULE_EXPORT void
show_presets_toggled_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    GtkWidget *frame;
    GtkWindow *hb_window;

    g_debug("show_presets_clicked_cb ()");
    frame = GHB_WIDGET (ud->builder, "presets_frame");
    ghb_widget_to_setting(ud->prefs, widget);
    if (ghb_settings_get_boolean(ud->prefs, "show_presets"))
    {
        gtk_widget_show_now(frame);
    }
    else
    {
        gtk_widget_hide(frame);
        hb_window = GTK_WINDOW(GHB_WIDGET (ud->builder, "hb_window"));
        gtk_window_resize(hb_window, 16, 16);
    }
    ghb_pref_save(ud->prefs, "show_presets");
}

static void
chapter_refresh_list_row_ui(
    GtkTreeModel *tm,
    GtkTreeIter *ti,
    GValue *chapter_list,
    int titleindex,
    int index)
{
    gchar *chapter, *s_duration, *s_start;
    gint hh, mm, ss;
    gint64 duration, start;

    // Update row with settings data
    g_debug("Updating chapter row ui");
    chapter = ghb_value_string(ghb_array_get_nth(chapter_list, index));
    duration = ghb_get_chapter_duration(titleindex, index) / 90000;
    break_duration(duration, &hh, &mm, &ss);
    s_duration = g_strdup_printf("%02d:%02d:%02d", hh, mm, ss);
    start = ghb_get_chapter_start(titleindex, index) / 90000;
    break_duration(start, &hh, &mm, &ss);
    s_start = g_strdup_printf("%02d:%02d:%02d", hh, mm, ss);
    gtk_list_store_set(GTK_LIST_STORE(tm), ti,
        0, index+1,
        1, s_start,
        2, s_duration,
        3, chapter,
        4, TRUE,
        -1);
    g_free(chapter);
    g_free(s_duration);
    g_free(s_start);
}

static void
ghb_clear_chapter_list_ui(GtkBuilder *builder)
{
    GtkTreeView *tv;
    GtkListStore *ts;

    tv = GTK_TREE_VIEW(GHB_WIDGET(builder, "chapters_list"));
    ts = GTK_LIST_STORE(gtk_tree_view_get_model(tv));
    gtk_list_store_clear(ts);
}

static void
chapter_refresh_list_ui(signal_user_data_t *ud)
{
    GValue *chapter_list;
    gint ii, count, tm_count;
    GtkTreeView  *tv;
    GtkTreeModel *tm;
    GtkTreeIter   ti;
    int titleindex;

    tv = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "chapters_list"));
    tm = gtk_tree_view_get_model(tv);

    tm_count = gtk_tree_model_iter_n_children(tm, NULL);

    titleindex = ghb_settings_combo_int(ud->settings, "title");
    chapter_list = ghb_settings_get_value(ud->settings, "chapter_list");
    count = ghb_array_len(chapter_list);
    if (count != tm_count)
    {
        ghb_clear_chapter_list_ui(ud->builder);
        for (ii = 0; ii < count; ii++)
        {
            gtk_list_store_append(GTK_LIST_STORE(tm), &ti);
        }
    }
    for (ii = 0; ii < count; ii++)
    {
        gtk_tree_model_iter_nth_child(tm, &ti, NULL, ii);
        chapter_refresh_list_row_ui(tm, &ti, chapter_list, titleindex, ii);
    }
}

void
ghb_chapter_list_refresh_all(signal_user_data_t *ud)
{
    chapter_refresh_list_ui(ud);
}

static void
update_chapter_list_settings(GValue *settings)
{
    GValue *chapters;
    gint titleindex;

    g_debug("update_chapter_list_settings ()");
    titleindex = ghb_settings_get_int(settings, "title_no");
    chapters = ghb_get_chapters(titleindex);
    if (chapters)
        ghb_settings_take_value(settings, "chapter_list", chapters);
}

static gint chapter_edit_key = 0;

G_MODULE_EXPORT gboolean
chapter_keypress_cb(
    GhbCellRendererText *cell,
    GdkEventKey *event,
    signal_user_data_t *ud)
{
    chapter_edit_key = event->keyval;
    return FALSE;
}

G_MODULE_EXPORT void
chapter_edited_cb(
    GhbCellRendererText *cell,
    gchar *path,
    gchar *text,
    signal_user_data_t *ud)
{
    GtkTreePath *treepath;
    GtkListStore *store;
    GtkTreeView *treeview;
    GtkTreeIter iter;
    gint index;
    gint *pi;
    gint row;

    g_debug("chapter_edited_cb ()");
    g_debug("path (%s)", path);
    g_debug("text (%s)", text);
    treeview = GTK_TREE_VIEW(GHB_WIDGET(ud->builder, "chapters_list"));
    store = GTK_LIST_STORE(gtk_tree_view_get_model(treeview));
    treepath = gtk_tree_path_new_from_string (path);
    pi = gtk_tree_path_get_indices(treepath);
    row = pi[0];
    gtk_tree_model_get_iter(GTK_TREE_MODEL(store), &iter, treepath);
    gtk_list_store_set(store, &iter,
        3, text,
        4, TRUE,
        -1);
    gtk_tree_model_get(GTK_TREE_MODEL(store), &iter, 0, &index, -1);

    const GValue *chapters;
    GValue *chapter;

    chapters = ghb_settings_get_value(ud->settings, "chapter_list");
    chapter = ghb_array_get_nth(chapters, index-1);
    g_value_set_string(chapter, text);
    if ((chapter_edit_key == GDK_KEY_Return || chapter_edit_key == GDK_KEY_Down) &&
        gtk_tree_model_iter_next(GTK_TREE_MODEL(store), &iter))
    {
        GtkTreeViewColumn *column;

        gtk_tree_path_next(treepath);
        // When a cell has been edited, I want to advance to the
        // next cell and start editing it automaitcally.
        // Unfortunately, we may not be in a state here where
        // editing is allowed.  This happens when the user selects
        // a new cell with the mouse instead of just hitting enter.
        // Some kind of Gtk quirk.  widget_editable==NULL assertion.
        // Editing is enabled again once the selection event has been
        // processed.  So I'm queueing up a callback to be called
        // when things go idle.  There, I will advance to the next
        // cell and initiate editing.
        //
        // Now, you might be asking why I don't catch the keypress
        // event and determine what action to take based on that.
        // The Gtk developers in their infinite wisdom have made the
        // actual GtkEdit widget being used a private member of
        // GtkCellRendererText, so it can not be accessed to hang a
        // signal handler off of.  And they also do not propagate the
        // keypress signals in any other way.  So that information is lost.
        //g_idle_add((GSourceFunc)next_cell, ud);
        //
        // Keeping the above comment for posterity.
        // I got industrious and made my own CellTextRendererText that
        // passes on the key-press-event. So now I have much better
        // control of this.
        column = gtk_tree_view_get_column(treeview, 3);
        gtk_tree_view_set_cursor(treeview, treepath, column, TRUE);
    }
    else if (chapter_edit_key == GDK_KEY_Up && row > 0)
    {
        GtkTreeViewColumn *column;
        gtk_tree_path_prev(treepath);
        column = gtk_tree_view_get_column(treeview, 3);
        gtk_tree_view_set_cursor(treeview, treepath, column, TRUE);
    }
    gtk_tree_path_free (treepath);
}

void
debug_log_handler(const gchar *domain, GLogLevelFlags flags, const gchar *msg, gpointer data)
{
    signal_user_data_t *ud = (signal_user_data_t*)data;

    if (ud->debug)
    {
        printf("%s: %s\n", domain, msg);
    }
}

void
warn_log_handler(const gchar *domain, GLogLevelFlags flags, const gchar *msg, gpointer data)
{
    printf("%s: %s\n", domain, msg);
}

void
ghb_hbfd(signal_user_data_t *ud, gboolean hbfd)
{
    GtkWidget *widget;
    g_debug("ghb_hbfd");
    widget = GHB_WIDGET(ud->builder, "queue_pause1");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "queue_add");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "show_queue");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "show_activity");
    gtk_widget_set_visible(widget, !hbfd);

    widget = GHB_WIDGET(ud->builder, "chapter_box");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "container_box");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "SettingsNotebook");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "presets_save");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET(ud->builder, "presets_remove");
    gtk_widget_set_visible(widget, !hbfd);
    widget = GHB_WIDGET (ud->builder, "hb_window");
    gtk_window_resize(GTK_WINDOW(widget), 16, 16);

}

G_MODULE_EXPORT void
hbfd_toggled_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("hbfd_toggled_cb");
    ghb_widget_to_setting(ud->prefs, widget);
    gboolean hbfd = ghb_settings_get_boolean(ud->prefs, "hbfd");
    ghb_hbfd(ud, hbfd);
    ghb_pref_save(ud->prefs, "hbfd");
}

G_MODULE_EXPORT void
advanced_video_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("advanced_video_changed_cb");
    ghb_widget_to_setting(ud->prefs, widget);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);
    ghb_show_hide_advanced_video(ud);
}

G_MODULE_EXPORT void
pref_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    ghb_widget_to_setting (ud->prefs, widget);
// FIXME?
    ghb_check_dependency(ud, widget, NULL);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);

    gint preview_count;
    preview_count = ghb_settings_get_int(ud->prefs, "preview_count");
    widget = GHB_WIDGET(ud->builder, "preview_frame");
    gtk_range_set_range(GTK_RANGE(widget), 1, preview_count);
}

G_MODULE_EXPORT void
use_m4v_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("use_m4v_changed_cb");
    ghb_widget_to_setting (ud->prefs, widget);
    ghb_check_dependency(ud, widget, NULL);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);
    ghb_update_destination_extension(ud);
}

G_MODULE_EXPORT void
show_status_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("show_status_cb");
    ghb_widget_to_setting (ud->prefs, widget);
    ghb_check_dependency(ud, widget, NULL);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);

#if defined(_USE_APP_IND)
    if (ud->ai)
    {
        if (ghb_settings_get_boolean(ud->prefs, "show_status"))
        {
            app_indicator_set_status(ud->ai, APP_INDICATOR_STATUS_ACTIVE);
        }
        else
        {
            app_indicator_set_status(ud->ai, APP_INDICATOR_STATUS_PASSIVE);
        }
    }
#else
    GtkStatusIcon *si;

    si = GTK_STATUS_ICON(GHB_OBJECT (ud->builder, "hb_status"));
    gtk_status_icon_set_visible(si,
            ghb_settings_get_boolean(ud->prefs, "show_status"));
#endif
}

G_MODULE_EXPORT void
vqual_granularity_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("vqual_granularity_changed_cb");
    ghb_widget_to_setting (ud->prefs, widget);
    ghb_check_dependency(ud, widget, NULL);

    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);

    float val, vqmin, vqmax, step, page;
    int inverted, digits;

    ghb_vquality_range(ud, &vqmin, &vqmax, &step, &page, &digits, &inverted);
    val = ghb_settings_get_double(ud->settings, "VideoQualitySlider");
    scale_configure(ud, "VideoQualitySlider", val, vqmin, vqmax,
                    step, page, digits, inverted);
}

G_MODULE_EXPORT void
tweaks_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("tweaks_changed_cb");
    ghb_widget_to_setting (ud->prefs, widget);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);
}

G_MODULE_EXPORT void
hbfd_feature_changed_cb(GtkWidget *widget, signal_user_data_t *ud)
{
    g_debug("hbfd_feature_changed_cb");
    ghb_widget_to_setting (ud->prefs, widget);
    const gchar *name = ghb_get_setting_key(widget);
    ghb_pref_set(ud->prefs, name);

    gboolean hbfd = ghb_settings_get_boolean(ud->prefs, "hbfd_feature");
    if (hbfd)
    {
        const GValue *val;
        val = ghb_settings_get_value(ud->prefs, "hbfd");
        ghb_ui_settings_update(ud, ud->prefs, "hbfd", val);
    }
    widget = GHB_WIDGET(ud->builder, "hbfd");
    gtk_widget_set_visible(widget, hbfd);
}

gboolean
ghb_file_menu_add_dvd(signal_user_data_t *ud)
{
    GList *link, *drives;
    static GList *dvd_items = NULL;

    g_debug("ghb_file_menu_add_dvd()");
    GtkMenu *menu = GTK_MENU(GHB_WIDGET(ud->builder, "file_submenu"));

    // Clear previous dvd items from list
    link = dvd_items;
    while (link != NULL)
    {
        GtkWidget * widget = GTK_WIDGET(link->data);
        // widget_destroy automatically removes widget from container.
        gtk_widget_destroy(widget);
        link = link->next;
    }
    g_list_free(dvd_items);
    dvd_items = NULL;

    int pos = 5;
    link = drives = dvd_device_list();
    if (drives != NULL)
    {
        GtkWidget *widget = gtk_separator_menu_item_new();
        dvd_items = g_list_append(dvd_items, (gpointer)widget);

        gtk_menu_shell_insert(GTK_MENU_SHELL(menu), widget, pos++);
        gtk_widget_set_visible(widget, TRUE);

        while (link != NULL)
        {
            GtkWidget *widget;
            gchar *drive = get_dvd_device_name(link->data);
            gchar *name = get_dvd_volume_name(link->data);

            widget = gtk_menu_item_new_with_label(name);
            gtk_buildable_set_name(GTK_BUILDABLE(widget), drive);
            gtk_widget_set_tooltip_text(widget, _("Scan this DVD source"));

            dvd_items = g_list_append(dvd_items, (gpointer)widget);
            gtk_menu_shell_insert(GTK_MENU_SHELL(menu), widget, pos++);

            gtk_widget_set_visible(widget, TRUE);

            // Connect signal to action (menu item)
            g_signal_connect(widget, "activate",
                (GCallback)dvd_source_activate_cb, ud);
            g_free(name);
            g_free(drive);
            free_drive(link->data);
            link = link->next;
        }

        g_list_free(drives);
    }

    return FALSE;
}

gboolean ghb_is_cd(GDrive *gd);

static GList*
dvd_device_list()
{
    GList *dvd_devices = NULL;

#if defined(_WIN32)
    gint ii, drives;
    gchar drive[5];

    strcpy(drive, "A:" G_DIR_SEPARATOR_S);
    drives = GetLogicalDrives();
    for (ii = 0; ii < 26; ii++)
    {
        if (drives & 0x01)
        {
            guint dtype;

            drive[0] = 'A' + ii;
            dtype = GetDriveType(drive);
            if (dtype == DRIVE_CDROM)
            {
                dvd_devices = g_list_append(dvd_devices,
                        (gpointer)g_strdup(drive));
            }
        }
        drives >>= 1;
    }
#else
    GVolumeMonitor *gvm;
    GList *drives, *link;

    gvm = g_volume_monitor_get ();
    drives = g_volume_monitor_get_connected_drives (gvm);
    link = drives;
    while (link != NULL)
    {
        GDrive *gd;

        gd = (GDrive*)link->data;
        if (ghb_is_cd(gd))
        {
            dvd_devices = g_list_append(dvd_devices, gd);
        }
        else
            g_object_unref (gd);
        link = link->next;
    }
    g_list_free(drives);
#endif

    return dvd_devices;
}

#if defined(__linux__)
static GUdevClient *udev_ctx = NULL;
#endif

gboolean
ghb_is_cd(GDrive *gd)
{
#if defined(__linux__)
    gchar *device;
    GUdevDevice *udd;

    if (udev_ctx == NULL)
        return FALSE;

    device = g_drive_get_identifier(gd, G_VOLUME_IDENTIFIER_KIND_UNIX_DEVICE);
    if (device == NULL)
        return FALSE;

    udd = g_udev_client_query_by_device_file(udev_ctx, device);
    g_free(device);

    if (udd == NULL)
    {
        g_message("udev: Failed to lookup device %s", device);
        return FALSE;
    }

    gint val;
    val = g_udev_device_get_property_as_int(udd, "ID_CDROM_DVD");
    g_object_unref(udd);
    if (val == 1)
        return TRUE;

    return FALSE;
#else
    return FALSE;
#endif
}

void
ghb_udev_init()
{
#if defined(__linux__)
    udev_ctx = g_udev_client_new(NULL);
#endif
}

#if defined(_WIN32)
static void
handle_media_change(const gchar *device, gboolean insert, signal_user_data_t *ud)
{
    guint dtype;
    static gint ins_count = 0;
    static gint rem_count = 0;

    // The media change event in windows bounces around a bit
    // so I debounce it here
    // DVD insertion detected.  Scan it.
    dtype = GetDriveType(device);
    if (dtype != DRIVE_CDROM)
        return;
    if (insert)
    {
        rem_count = 0;
        ins_count++;
        if (ins_count == 2)
        {
            GHB_THREAD_NEW("Cache Volume Names",
                    (GThreadFunc)ghb_cache_volnames, ud);
            if (ghb_settings_get_boolean(ud->prefs, "AutoScan") &&
                ud->current_dvd_device != NULL &&
                strcmp(device, ud->current_dvd_device) == 0)
            {
                show_scan_progress(ud);
                update_source_label(ud, device);
                gint preview_count;
                preview_count = ghb_settings_get_int(ud->prefs, "preview_count");
                ghb_settings_set_string(ud->globals, "scan_source", device);
                start_scan(ud, device, 0, preview_count);
            }
        }
    }
    else
    {
        ins_count = 0;
        rem_count++;
        if (rem_count == 2)
        {
            GHB_THREAD_NEW("Cache Volume Names",
                    (GThreadFunc)ghb_cache_volnames, ud);
            if (ud->current_dvd_device != NULL &&
                strcmp(device, ud->current_dvd_device) == 0)
            {
                ghb_hb_cleanup(TRUE);
                prune_logs(ud);
                ghb_settings_set_string(ud->globals, "scan_source", "/dev/null");
                start_scan(ud, "/dev/null", 0, 1);
            }
        }
    }
}

static gchar
FindDriveFromMask(ULONG unitmask)
{
    gchar cc;
    for (cc = 0; cc < 26; cc++)
    {
        if (unitmask & 0x01)
            return 'A' + cc;
        unitmask >>= 1;
    }
    return 0;
}

void
wm_drive_changed(MSG *msg, signal_user_data_t *ud)
{
    PDEV_BROADCAST_HDR bch = (PDEV_BROADCAST_HDR)msg->lParam;
    gchar drive[4];

    g_strlcpy(drive, "A:" G_DIR_SEPARATOR_S, 4);
    switch (msg->wParam)
    {
        case DBT_DEVICEARRIVAL:
        {
            if (bch->dbch_devicetype == DBT_DEVTYP_VOLUME)
            {
                PDEV_BROADCAST_VOLUME bcv = (PDEV_BROADCAST_VOLUME)bch;

                if (bcv->dbcv_flags & DBTF_MEDIA)
                {
                    drive[0] = FindDriveFromMask(bcv->dbcv_unitmask);
                    handle_media_change(drive, TRUE, ud);
                }
            }
        } break;

        case DBT_DEVICEREMOVECOMPLETE:
        {
            if (bch->dbch_devicetype == DBT_DEVTYP_VOLUME)
            {
                PDEV_BROADCAST_VOLUME bcv = (PDEV_BROADCAST_VOLUME)bch;

                if (bcv->dbcv_flags & DBTF_MEDIA)
                {
                    drive[0] = FindDriveFromMask(bcv->dbcv_unitmask);
                    handle_media_change(drive, FALSE, ud);
                }
            }
        } break;
        default: ;
    }
}

#else

G_MODULE_EXPORT void
drive_changed_cb(GVolumeMonitor *gvm, GDrive *gd, signal_user_data_t *ud)
{
    gchar *device;
    gint state;

    g_debug("drive_changed_cb()");
    GHB_THREAD_NEW("Cache Volume Names", (GThreadFunc)ghb_cache_volnames, ud);

    state = ghb_get_scan_state();
    device = g_drive_get_identifier(gd, G_VOLUME_IDENTIFIER_KIND_UNIX_DEVICE);
    if (ud->current_dvd_device == NULL ||
        strcmp(device, ud->current_dvd_device) != 0 ||
        state != GHB_STATE_IDLE )
    {
        return;
    }
    if (g_drive_has_media(gd))
    {
        if (ghb_settings_get_boolean(ud->prefs, "AutoScan"))
        {
            show_scan_progress(ud);
            update_source_label(ud, device);
            gint preview_count;
            preview_count = ghb_settings_get_int(ud->prefs, "preview_count");
            ghb_settings_set_string(ud->globals, "scan_source", device);
            start_scan(ud, device, 0, preview_count);
        }
    }
    else
    {
        ghb_hb_cleanup(TRUE);
        prune_logs(ud);
        ghb_settings_set_string(ud->globals, "scan_source", "/dev/null");
        start_scan(ud, "/dev/null", 0, 1);
    }
}
#endif

#if !defined(_WIN32)
#define GPM_DBUS_PM_SERVICE         "org.freedesktop.PowerManagement"
#define GPM_DBUS_PM_PATH            "/org/freedesktop/PowerManagement"
#define GPM_DBUS_PM_INTERFACE       "org.freedesktop.PowerManagement"
#define GPM_DBUS_INHIBIT_PATH       "/org/freedesktop/PowerManagement/Inhibit"
#define GPM_DBUS_INHIBIT_INTERFACE  "org.freedesktop.PowerManagement.Inhibit"
static gboolean gpm_inhibited = FALSE;
static guint gpm_cookie = -1;
#endif

static gboolean
ghb_can_suspend_gpm()
{
    gboolean can_suspend = FALSE;
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_can_suspend_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return FALSE;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_PM_PATH, GPM_DBUS_PM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return FALSE;
    }
    res = dbus_g_proxy_call(proxy, "CanSuspend", &error,
                            G_TYPE_INVALID,
                            G_TYPE_BOOLEAN, &can_suspend,
                            G_TYPE_INVALID);
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("CanSuspend failed: %s", error->message);
            g_error_free(error);
        }
        else
            g_warning("CanSuspend failed");
        // Try to shutdown anyway
        can_suspend = TRUE;
    }
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
#endif
    return can_suspend;
}

static void
ghb_suspend_gpm()
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_suspend_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_PM_PATH, GPM_DBUS_PM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "Suspend", &error,
                            G_TYPE_INVALID,
                            G_TYPE_INVALID);
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("Suspend failed: %s", error->message);
            g_error_free(error);
        }
        else
            g_warning("Suspend failed");
    }
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
#endif
}

#if !defined(_WIN32)
static gboolean
ghb_can_shutdown_gpm()
{
    gboolean can_shutdown = FALSE;
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_can_shutdown_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return FALSE;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_PM_PATH, GPM_DBUS_PM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return FALSE;
    }
    res = dbus_g_proxy_call(proxy, "CanShutdown", &error,
                            G_TYPE_INVALID,
                            G_TYPE_BOOLEAN, &can_shutdown,
                            G_TYPE_INVALID);
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("CanShutdown failed: %s", error->message);
            g_error_free(error);
        }
        else
            g_warning("CanShutdown failed");
        // Try to shutdown anyway
        can_shutdown = TRUE;
    }
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
    return can_shutdown;
}
#endif

#if !defined(_WIN32)
static void
ghb_shutdown_gpm()
{
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_shutdown_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_PM_PATH, GPM_DBUS_PM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "Shutdown", &error,
                            G_TYPE_INVALID,
                            G_TYPE_INVALID);
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("Shutdown failed: %s", error->message);
            g_error_free(error);
        }
        else
            g_warning("Shutdown failed");
    }
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
}
#endif

void
ghb_inhibit_gpm()
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    if (gpm_inhibited)
    {
        // Already inhibited
        return;
    }
    g_debug("ghb_inhibit_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_INHIBIT_PATH, GPM_DBUS_INHIBIT_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "Inhibit", &error,
                            G_TYPE_STRING, "ghb",
                            G_TYPE_STRING, "Encoding",
                            G_TYPE_INVALID,
                            G_TYPE_UINT, &gpm_cookie,
                            G_TYPE_INVALID);
    gpm_inhibited = TRUE;
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("Inhibit failed: %s", error->message);
            g_error_free(error);
            gpm_cookie = -1;
        }
        else
            g_warning("Inhibit failed");
        gpm_cookie = -1;
        gpm_inhibited = FALSE;
    }
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
#endif
}

void
ghb_uninhibit_gpm()
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;

    g_debug("ghb_uninhibit_gpm() gpm_cookie %u", gpm_cookie);

    if (!gpm_inhibited)
    {
        // Not inhibited
        return;
    }
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_PM_SERVICE,
                            GPM_DBUS_INHIBIT_PATH, GPM_DBUS_INHIBIT_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_PM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "UnInhibit", &error,
                            G_TYPE_UINT, gpm_cookie,
                            G_TYPE_INVALID,
                            G_TYPE_INVALID);
    if (!res)
    {
        if (error != NULL)
        {
            g_warning("UnInhibit failed: %s", error->message);
            g_error_free(error);
        }
        else
            g_warning("UnInhibit failed");
    }
    gpm_inhibited = FALSE;
    dbus_g_connection_unref(conn);
    g_object_unref(G_OBJECT(proxy));
#endif
}

#if !defined(_WIN32)

// For inhibit and shutdown
#define GPM_DBUS_SM_SERVICE         "org.gnome.SessionManager"
#define GPM_DBUS_SM_PATH            "/org/gnome/SessionManager"
#define GPM_DBUS_SM_INTERFACE       "org.gnome.SessionManager"

#endif

static gboolean
ghb_can_shutdown_gsm()
{
    gboolean can_shutdown = FALSE;
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_can_shutdown_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return FALSE;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_SM_SERVICE,
                            GPM_DBUS_SM_PATH, GPM_DBUS_SM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_SM_SERVICE);
        dbus_g_connection_unref(conn);
        return FALSE;
    }
    res = dbus_g_proxy_call(proxy, "CanShutdown", &error,
                            G_TYPE_INVALID,
                            G_TYPE_BOOLEAN, &can_shutdown,
                            G_TYPE_INVALID);
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
    if (!res)
    {
        if (error != NULL)
        {
            g_error_free(error);
        }
        // Try to shutdown anyway
        can_shutdown = TRUE;
        // Try the gpm version
        return ghb_can_shutdown_gpm();
    }
#endif
    return can_shutdown;
}

static void
ghb_shutdown_gsm()
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;


    g_debug("ghb_shutdown_gpm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_SM_SERVICE,
                            GPM_DBUS_SM_PATH, GPM_DBUS_SM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_SM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "Shutdown", &error,
                            G_TYPE_INVALID,
                            G_TYPE_INVALID);
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
    if (!res)
    {
        if (error != NULL)
        {
            g_error_free(error);
        }
        // Try the gpm version
        ghb_shutdown_gpm();
    }
#endif
}

void
ghb_inhibit_gsm(signal_user_data_t *ud)
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;
    guint xid;
    GtkWidget *widget;


    if (gpm_inhibited)
    {
        // Already inhibited
        return;
    }
    g_debug("ghb_inhibit_gsm()");
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_SM_SERVICE,
                            GPM_DBUS_SM_PATH, GPM_DBUS_SM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_SM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    widget = GHB_WIDGET(ud->builder, "hb_window");
    xid = GDK_WINDOW_XID(gtk_widget_get_window(widget));
    res = dbus_g_proxy_call(proxy, "Inhibit", &error,
                            G_TYPE_STRING, "ghb",
                            G_TYPE_UINT, xid,
                            G_TYPE_STRING, "Encoding",
                            G_TYPE_UINT, 1 | 4,
                            G_TYPE_INVALID,
                            G_TYPE_UINT, &gpm_cookie,
                            G_TYPE_INVALID);
    gpm_inhibited = TRUE;
    g_object_unref(G_OBJECT(proxy));
    dbus_g_connection_unref(conn);
    if (!res)
    {
        if (error != NULL)
        {
            g_error_free(error);
            gpm_cookie = -1;
        }
        gpm_cookie = -1;
        gpm_inhibited = FALSE;
        // Try the gpm version
        ghb_inhibit_gpm();
    }
#endif
}

void
ghb_uninhibit_gsm()
{
#if !defined(_WIN32)
    DBusGConnection *conn;
    DBusGProxy  *proxy;
    GError *error = NULL;
    gboolean res;

    g_debug("ghb_uninhibit_gsm() gpm_cookie %u", gpm_cookie);

    if (!gpm_inhibited)
    {
        // Not inhibited
        return;
    }
    conn = dbus_g_bus_get(DBUS_BUS_SESSION, &error);
    if (error != NULL)
    {
        g_warning("DBUS cannot connect: %s", error->message);
        g_error_free(error);
        return;
    }
    proxy = dbus_g_proxy_new_for_name(conn, GPM_DBUS_SM_SERVICE,
                            GPM_DBUS_SM_PATH, GPM_DBUS_SM_INTERFACE);
    if (proxy == NULL)
    {
        g_warning("Could not get DBUS proxy: %s", GPM_DBUS_SM_SERVICE);
        dbus_g_connection_unref(conn);
        return;
    }
    res = dbus_g_proxy_call(proxy, "Uninhibit", &error,
                            G_TYPE_UINT, gpm_cookie,
                            G_TYPE_INVALID,
                            G_TYPE_INVALID);
    dbus_g_connection_unref(conn);
    g_object_unref(G_OBJECT(proxy));
    if (!res)
    {
        if (error != NULL)
        {
            g_error_free(error);
        }
        ghb_uninhibit_gpm();
    }
    gpm_inhibited = FALSE;
#endif
}

G_MODULE_EXPORT gboolean
tweak_setting_cb(
    GtkWidget *widget,
    GdkEventButton *event,
    signal_user_data_t *ud)
{
    const gchar *name;
    gchar *tweak_name;
    gboolean ret = FALSE;
    gboolean allow_tweaks;

    g_debug("press %d %d", event->type, event->button);
    allow_tweaks = ghb_settings_get_boolean(ud->prefs, "allow_tweaks");
    if (allow_tweaks && event->type == GDK_BUTTON_PRESS && event->button == 3)
    { // Its a right mouse click
        GtkWidget *dialog;
        GtkEntry *entry;
        GtkResponseType response;
        gchar *tweak = NULL;

        name = ghb_get_setting_key(widget);
        if (g_str_has_prefix(name, "tweak_"))
        {
            tweak_name = g_strdup(name);
        }
        else
        {
            tweak_name = g_strdup_printf("tweak_%s", name);
        }

        tweak = ghb_settings_get_string (ud->settings, tweak_name);
        dialog = GHB_WIDGET(ud->builder, "tweak_dialog");
        gtk_window_set_title(GTK_WINDOW(dialog), tweak_name);
        entry = GTK_ENTRY(GHB_WIDGET(ud->builder, "tweak_setting"));
        if (tweak)
        {
            gtk_entry_set_text(entry, tweak);
            g_free(tweak);
        }
        response = gtk_dialog_run(GTK_DIALOG(dialog));
        gtk_widget_hide(dialog);
        if (response == GTK_RESPONSE_OK)
        {
            tweak = (gchar*)gtk_entry_get_text(entry);
            if (ghb_validate_filter_string(tweak, -1))
                ghb_settings_set_string(ud->settings, tweak_name, tweak);
            else
            {
                gchar *message;
                message = g_strdup_printf(
                            _("Invalid Settings:\n%s"),
                            tweak);
                ghb_message_dialog(GTK_MESSAGE_ERROR, message, "Cancel", NULL);
                g_free(message);
            }
        }
        g_free(tweak_name);
        ret = TRUE;
    }
    return ret;
}

G_MODULE_EXPORT gboolean
easter_egg_cb(
    GtkWidget *widget,
    GdkEventButton *event,
    signal_user_data_t *ud)
{
    g_debug("press %d %d", event->type, event->button);
    if (event->type == GDK_3BUTTON_PRESS && event->button == 1)
    { // Its a tripple left mouse button click
        GtkWidget *widget;
        widget = GHB_WIDGET(ud->builder, "allow_tweaks");
        gtk_widget_show(widget);
        widget = GHB_WIDGET(ud->builder, "hbfd_feature");
        gtk_widget_show(widget);
    }
    else if (event->type == GDK_BUTTON_PRESS && event->button == 1)
    {
        GtkWidget *widget;
        widget = GHB_WIDGET(ud->builder, "allow_tweaks");
        gtk_widget_hide(widget);
        widget = GHB_WIDGET(ud->builder, "hbfd_feature");
        gtk_widget_hide(widget);
    }
    return FALSE;
}

G_MODULE_EXPORT gchar*
format_deblock_cb(GtkScale *scale, gdouble val, signal_user_data_t *ud)
{
    if (val < 5.0)
    {
        return g_strdup_printf(_("Off"));
    }
    else
    {
        return g_strdup_printf("%d", (gint)val);
    }
}

G_MODULE_EXPORT gchar*
format_vquality_cb(GtkScale *scale, gdouble val, signal_user_data_t *ud)
{
    gint vcodec = ghb_settings_combo_int(ud->settings, "VideoEncoder");
    switch (vcodec)
    {
        case HB_VCODEC_X264:
        {
            if (val == 0.0)
            {
                return g_strdup_printf(_("RF: %.4g (Warning: lossless)"), val);
            }
            else
            {
                return g_strdup_printf("RF: %.4g", val);
            }
        } break;

        case HB_VCODEC_FFMPEG_MPEG4:
        case HB_VCODEC_FFMPEG_MPEG2:
        {
            return g_strdup_printf("QP: %d", (int)val);
        } break;

        case HB_VCODEC_THEORA:
        {
            return g_strdup_printf("QP: %d", (int)val);
        } break;

        default:
        {
        } break;
    }
    return g_strdup_printf("QP: %.4g", val);
}

static void
process_appcast(signal_user_data_t *ud)
{
    gchar *description = NULL, *build = NULL, *version = NULL, *msg;
#if !defined(_WIN32) && !defined(_NO_UPDATE_CHECK)
    GtkWidget *window;
    static GtkWidget *html = NULL;
#endif
    GtkWidget *dialog, *label;
    gint    response, ibuild = 0, skip;

    if (ud->appcast == NULL || ud->appcast_len < 15 ||
        strncmp(&(ud->appcast[9]), "200 OK", 6))
    {
        goto done;
    }
    ghb_appcast_parse(ud->appcast, &description, &build, &version);
    if (build)
        ibuild = g_strtod(build, NULL);
    skip = ghb_settings_get_int(ud->prefs, "update_skip_version");
    if (description == NULL || build == NULL || version == NULL
        || ibuild <= hb_get_build(NULL) || skip == ibuild)
    {
        goto done;
    }
    msg = g_strdup_printf(_("HandBrake %s/%s is now available (you have %s/%d)."),
            version, build, hb_get_version(NULL), hb_get_build(NULL));
    label = GHB_WIDGET(ud->builder, "update_message");
    gtk_label_set_text(GTK_LABEL(label), msg);

#if !defined(_WIN32) && !defined(_NO_UPDATE_CHECK)
    if (html == NULL)
    {
        html = webkit_web_view_new();
        window = GHB_WIDGET(ud->builder, "update_scroll");
        gtk_container_add(GTK_CONTAINER(window), html);
        // Show it
        gtk_widget_set_size_request(html, 420, 240);
        gtk_widget_show(html);
    }
    webkit_web_view_open(WEBKIT_WEB_VIEW(html), description);
#endif
    dialog = GHB_WIDGET(ud->builder, "update_dialog");
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_hide(dialog);
    if (response == GTK_RESPONSE_OK)
    {
        // Skip
        ghb_settings_set_int(ud->prefs, "update_skip_version", ibuild);
        ghb_pref_save(ud->prefs, "update_skip_version");
    }
    g_free(msg);

done:
    if (description) g_free(description);
    if (build) g_free(build);
    if (version) g_free(version);
    g_free(ud->appcast);
    ud->appcast_len = 0;
    ud->appcast = NULL;
    appcast_busy = FALSE;
}

void
ghb_net_close(GIOChannel *ioc)
{
    gint fd;

    g_debug("ghb_net_close");
    if (ioc == NULL) return;
    fd = g_io_channel_unix_get_fd(ioc);
    close(fd);
    g_io_channel_unref(ioc);
}

G_MODULE_EXPORT gboolean
ghb_net_recv_cb(GIOChannel *ioc, GIOCondition cond, gpointer data)
{
    gchar buf[2048];
    gsize len;
    GError *gerror = NULL;
    GIOStatus status;

    g_debug("ghb_net_recv_cb");
    signal_user_data_t *ud = (signal_user_data_t*)data;

    status = g_io_channel_read_chars (ioc, buf, 2048, &len, &gerror);
    if ((status == G_IO_STATUS_NORMAL || status == G_IO_STATUS_EOF) &&
        len > 0)
    {
        gint new_len = ud->appcast_len + len;
        ud->appcast = g_realloc(ud->appcast, new_len + 1);
        memcpy(&(ud->appcast[ud->appcast_len]), buf, len);
        ud->appcast_len = new_len;
    }
    if (status == G_IO_STATUS_EOF)
    {
        if ( ud->appcast != NULL )
        {
            ud->appcast[ud->appcast_len] = 0;
        }
        ghb_net_close(ioc);
        process_appcast(ud);
        return FALSE;
    }
    return TRUE;
}

GIOChannel*
ghb_net_open(signal_user_data_t *ud, gchar *address, gint port)
{
    GIOChannel *ioc;
    gint fd;

    struct sockaddr_in   sock;
    struct hostent     * host;

    g_debug("ghb_net_open");
    if( !( host = gethostbyname( address ) ) )
    {
        g_warning( "gethostbyname failed (%s)", address );
        appcast_busy = FALSE;
        return NULL;
    }

    memset( &sock, 0, sizeof( struct sockaddr_in ) );
    sock.sin_family = host->h_addrtype;
    sock.sin_port   = htons( port );
    memcpy( &sock.sin_addr, host->h_addr, host->h_length );

    fd = socket(host->h_addrtype, SOCK_STREAM, 0);
    if( fd < 0 )
    {
        g_debug( "socket failed" );
        appcast_busy = FALSE;
        return NULL;
    }

    if(connect(fd, (struct sockaddr*)&sock, sizeof(struct sockaddr_in )) < 0 )
    {
        g_debug( "connect failed" );
        appcast_busy = FALSE;
        return NULL;
    }
    ioc = g_io_channel_unix_new(fd);
    g_io_channel_set_encoding (ioc, NULL, NULL);
    g_io_channel_set_flags(ioc, G_IO_FLAG_NONBLOCK, NULL);
    g_io_add_watch (ioc, G_IO_IN, ghb_net_recv_cb, (gpointer)ud );

    return ioc;
}

gpointer
ghb_check_update(signal_user_data_t *ud)
{
    gchar *query;
    gsize len;
    GIOChannel *ioc;
    GError *gerror = NULL;
    GRegex *regex;
    GMatchInfo *mi;
    gchar *host, *appcast;

    g_debug("ghb_check_update");
    appcast_busy = TRUE;
    regex = g_regex_new("^http://(.+)/(.+)$", 0, 0, NULL);
    if (!g_regex_match(regex, HB_PROJECT_URL_APPCAST, 0, &mi))
    {
        return NULL;
    }

    host = g_match_info_fetch(mi, 1);
    appcast = g_match_info_fetch(mi, 2);

    if (host == NULL || appcast == NULL)
        return NULL;

    query = g_strdup_printf("GET /%s HTTP/1.0\r\nHost: %s\r\n\r\n",
                            appcast, host);

    ioc = ghb_net_open(ud, host, 80);
    if (ioc == NULL)
        goto free_resources;

    g_io_channel_write_chars(ioc, query, strlen(query), &len, &gerror);
    g_io_channel_flush(ioc, &gerror);

free_resources:
    g_free(query);
    g_free(host);
    g_free(appcast);
    g_match_info_free(mi);
    g_regex_unref(regex);
    return NULL;
}

G_MODULE_EXPORT gboolean
hb_visibility_event_cb(
    GtkWidget *widget,
    GdkEventVisibility *vs,
    signal_user_data_t *ud)
{
    ud->hb_visibility = vs->state;
    return FALSE;
}

G_MODULE_EXPORT void
status_activate_cb(GtkStatusIcon *si, signal_user_data_t *ud)
{
    GtkWindow *window;
    GdkWindowState state;

    window = GTK_WINDOW(GHB_WIDGET(ud->builder, "hb_window"));
    state = gdk_window_get_state(gtk_widget_get_window(GTK_WIDGET(window)));
    if ((state & GDK_WINDOW_STATE_ICONIFIED) ||
        (ud->hb_visibility != GDK_VISIBILITY_UNOBSCURED))
    {
        gtk_window_present(window);
        gtk_window_set_skip_taskbar_hint(window, FALSE);
    }
    else
    {
        gtk_window_set_skip_taskbar_hint(window, TRUE);
        gtk_window_iconify(window);
    }
}

G_MODULE_EXPORT void
show_hide_toggle_cb(GtkWidget *xwidget, signal_user_data_t *ud)
{
    GtkWindow *window;
    GdkWindowState state;

    window = GTK_WINDOW(GHB_WIDGET(ud->builder, "hb_window"));
    state = gdk_window_get_state(gtk_widget_get_window(GTK_WIDGET(window)));
    if ((state & GDK_WINDOW_STATE_ICONIFIED) ||
        (ud->hb_visibility != GDK_VISIBILITY_UNOBSCURED))
    {
        gtk_window_present(window);
        gtk_window_set_skip_taskbar_hint(window, FALSE);
    }
    else
    {
        gtk_window_set_skip_taskbar_hint(window, TRUE);
        gtk_window_iconify(window);
    }
}

#if !defined(_WIN32)
G_MODULE_EXPORT void
notify_closed_cb(NotifyNotification *notification, signal_user_data_t *ud)
{
    g_object_unref(G_OBJECT(notification));
}
#endif

void
ghb_notify_done(signal_user_data_t *ud)
{

    if (ghb_settings_combo_int(ud->prefs, "WhenComplete") == 0)
        return;

#if !defined(_WIN32)
    NotifyNotification *notification;
    notification = notify_notification_new(
        _("Encode Complete"),
        _("Put down that cocktail, Your HandBrake queue is done!"),
        NULL
#if NOTIFY_CHECK_VERSION (0, 7, 0)
                );
#else
        ,NULL);

    GtkStatusIcon *si = GTK_STATUS_ICON(GHB_OBJECT(ud->builder, "hb_status"));
    notify_notification_attach_to_status_icon(notification, si);
#endif
    GtkIconTheme *theme = gtk_icon_theme_get_default();
    GdkPixbuf *pb = gtk_icon_theme_load_icon(theme, "hb-icon", 32,
                                            GTK_ICON_LOOKUP_USE_BUILTIN, NULL);
    notify_notification_set_icon_from_pixbuf(notification, pb);
    g_signal_connect(notification, "closed", (GCallback)notify_closed_cb, ud);
    notify_notification_show(notification, NULL);
    g_object_unref(G_OBJECT(pb));
#endif

    if (ghb_settings_combo_int(ud->prefs, "WhenComplete") == 3)
    {
        if (ghb_can_shutdown_gsm())
        {
            ghb_countdown_dialog(GTK_MESSAGE_WARNING,
                _("Your encode is complete."),
                _("Shutting down the computer"),
                _("Cancel"), (GSourceFunc)shutdown_cb, ud, 60);
        }
    }
    if (ghb_settings_combo_int(ud->prefs, "WhenComplete") == 2)
    {
        if (ghb_can_suspend_gpm())
        {
            ghb_countdown_dialog(GTK_MESSAGE_WARNING,
                _("Your encode is complete."),
                _("Putting computer to sleep"),
                _("Cancel"), (GSourceFunc)suspend_cb, ud, 60);
        }
    }
    if (ghb_settings_combo_int(ud->prefs, "WhenComplete") == 4)
    {
        ghb_countdown_dialog(GTK_MESSAGE_WARNING,
                            _("Your encode is complete."),
                            _("Quiting Handbrake"),
                            _("Cancel"), (GSourceFunc)quit_cb, ud, 60);
    }
}

G_MODULE_EXPORT gboolean
window_configure_cb(
    GtkWidget *widget,
    GdkEventConfigure *event,
    signal_user_data_t *ud)
{
    //g_message("preview_configure_cb()");
    if (gtk_widget_get_visible(widget))
    {
        gint w, h;
        w = ghb_settings_get_int(ud->prefs, "window_width");
        h = ghb_settings_get_int(ud->prefs, "window_height");

        if ( w != event->width || h != event->height )
        {
            ghb_settings_set_int(ud->prefs, "window_width", event->width);
            ghb_settings_set_int(ud->prefs, "window_height", event->height);
            ghb_pref_set(ud->prefs, "window_width");
            ghb_pref_set(ud->prefs, "window_height");
            ghb_prefs_store();
        }
    }
    return FALSE;
}

static void container_empty_cb(GtkWidget *widget, gpointer data)
{
    gtk_widget_destroy(widget);
}

void ghb_container_empty(GtkContainer *c)
{
    gtk_container_foreach(c, container_empty_cb, NULL);
}

