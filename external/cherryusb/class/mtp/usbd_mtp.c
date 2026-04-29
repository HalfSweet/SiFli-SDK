/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "usbd_core.h"
#include "usbd_mtp.h"

#include <stdio.h>
#include <time.h>

#undef USB_DBG_TAG
#define USB_DBG_TAG "usbd_mtp"
#include "usb_log.h"

#ifndef CONFIG_USBDEV_MTP_MAX_BUFSIZE
#define CONFIG_USBDEV_MTP_MAX_BUFSIZE 2048
#endif

#ifndef CONFIG_USBDEV_MTP_MAX_OBJECTS
#define CONFIG_USBDEV_MTP_MAX_OBJECTS 256
#endif

#ifndef CONFIG_USBDEV_MTP_MAX_PATHNAME
#define CONFIG_USBDEV_MTP_MAX_PATHNAME 256
#endif

#ifndef CONFIG_USBDEV_MTP_MANUFACTURER
#define CONFIG_USBDEV_MTP_MANUFACTURER "CherryUSB"
#endif

#ifndef CONFIG_USBDEV_MTP_MODEL
#define CONFIG_USBDEV_MTP_MODEL "CherryUSB MTP"
#endif

#ifndef CONFIG_USBDEV_MTP_DEVICE_VERSION
#define CONFIG_USBDEV_MTP_DEVICE_VERSION "1.00"
#endif

#ifndef CONFIG_USBDEV_MTP_SERIAL_NUMBER
#define CONFIG_USBDEV_MTP_SERIAL_NUMBER "20260428"
#endif

#ifndef CONFIG_USBDEV_MTP_STORAGE_ID
#define CONFIG_USBDEV_MTP_STORAGE_ID 0x00010001U
#endif

#ifndef CONFIG_USBDEV_MTP_PRIO
#define CONFIG_USBDEV_MTP_PRIO 4
#endif

#ifndef CONFIG_USBDEV_MTP_STACKSIZE
#define CONFIG_USBDEV_MTP_STACKSIZE 4096
#endif

#define MTP_EP_OUT_IDX 0
#define MTP_EP_IN_IDX  1
#define MTP_EP_INT_IDX 2

#define MTP_STORAGE_ID CONFIG_USBDEV_MTP_STORAGE_ID
#define MTP_ALL_HANDLES 0xffffffffU
#define MTP_ROOT_PARENT 0x00000000U
#define MTP_GROUP_NONE  0x00000000U

enum mtp_rx_state {
    MTP_RX_COMMAND = 0,
    MTP_RX_SEND_OBJECT_INFO,
    MTP_RX_SEND_OBJECT
};

enum mtp_tx_state {
    MTP_TX_IDLE = 0,
    MTP_TX_SIMPLE_DATA,
    MTP_TX_RESPONSE,
    MTP_TX_GET_OBJECT,
    MTP_TX_EVENT
};

enum mtp_worker_event {
    MTP_WORK_OUT = 1,
    MTP_WORK_IN,
};

struct mtp_builder {
    uint8_t *buf;
    uint32_t cap;
    uint32_t len;
    bool overflow;
};

struct mtp_reader {
    const uint8_t *ptr;
    const uint8_t *end;
};

struct mtp_response {
    uint16_t code;
    uint8_t argc;
    uint32_t argv[5];
};

struct usbd_mtp_priv {
    struct usbd_endpoint ep_data[3];
    uint8_t out_ep;
    uint8_t in_ep;
    uint8_t int_ep;
    bool configured;

    enum mtp_rx_state rx_state;
    enum mtp_tx_state tx_state;
    uint32_t last_nbytes;

    bool session_open;
    uint32_t session_id;
    uint32_t transaction_id;
    uint16_t operation_code;
    bool cancelled;

    bool objects_valid;
    uint32_t next_handle;
    struct mtp_object objects[CONFIG_USBDEV_MTP_MAX_OBJECTS];

    bool send_info_valid;
    uint32_t send_storage_id;
    uint32_t send_parent_handle;
    uint32_t send_object_handle;
    uint16_t send_object_format;
    uint32_t send_object_size;
    char send_object_path[CONFIG_USBDEV_MTP_MAX_PATHNAME];

    int fd;
    uint32_t transfer_handle;
    uint32_t transfer_total;
    uint32_t transfer_done;
    uint32_t transfer_remaining;

    bool zlp_pending;
    bool response_after_data;
    struct mtp_response response;

#if defined(CONFIG_USBDEV_MTP_THREAD)
    usb_osal_mq_t mq;
    usb_osal_thread_t thread;
#endif
};

static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_mtp_rx_buffer[CONFIG_USBDEV_MAX_BUS][USB_ALIGN_UP(CONFIG_USBDEV_MTP_MAX_BUFSIZE, CONFIG_USB_ALIGN_SIZE)];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_mtp_tx_buffer[CONFIG_USBDEV_MAX_BUS][USB_ALIGN_UP(CONFIG_USBDEV_MTP_MAX_BUFSIZE, CONFIG_USB_ALIGN_SIZE)];
static USB_NOCACHE_RAM_SECTION USB_MEM_ALIGNX uint8_t g_mtp_event_buffer[CONFIG_USBDEV_MAX_BUS][USB_ALIGN_UP(32, CONFIG_USB_ALIGN_SIZE)];
static struct usbd_mtp_priv g_usbd_mtp[CONFIG_USBDEV_MAX_BUS];

static const uint16_t g_mtp_supported_operations[] = {
    MTP_OPERATION_GET_DEVICE_INFO,
    MTP_OPERATION_OPEN_SESSION,
    MTP_OPERATION_CLOSE_SESSION,
    MTP_OPERATION_GET_STORAGE_IDS,
    MTP_OPERATION_GET_STORAGE_INFO,
    MTP_OPERATION_GET_OBJECT_HANDLES,
    MTP_OPERATION_GET_OBJECT_INFO,
    MTP_OPERATION_GET_OBJECT,
    MTP_OPERATION_DELETE_OBJECT,
    MTP_OPERATION_SEND_OBJECT_INFO,
    MTP_OPERATION_SEND_OBJECT,
    MTP_OPERATION_GET_DEVICE_PROP_DESC,
    MTP_OPERATION_GET_OBJECT_PROPS_SUPPORTED,
    MTP_OPERATION_GET_OBJECT_PROP_DESC,
    MTP_OPERATION_GET_OBJECT_PROP_VALUE,
};

static const uint16_t g_mtp_supported_events[] = {
    MTP_EVENT_OBJECT_ADDED,
    MTP_EVENT_OBJECT_REMOVED,
    MTP_EVENT_STORAGE_INFO_CHANGED,
    MTP_EVENT_OBJECT_INFO_CHANGED,
    MTP_EVENT_DEVICE_PROP_CHANGED,
    MTP_EVENT_OBJECT_PROP_CHANGED,
};

static const uint16_t g_mtp_supported_device_properties[] = {
    MTP_DEVICE_PROPERTY_BATTERY_LEVEL,
    MTP_DEVICE_PROPERTY_DEVICE_FRIENDLY_NAME,
};

static const uint16_t g_mtp_supported_formats[] = {
    MTP_FORMAT_UNDEFINED,
    MTP_FORMAT_ASSOCIATION,
    MTP_FORMAT_TEXT,
    MTP_FORMAT_HTML,
    MTP_FORMAT_EXIF_JPEG,
    MTP_FORMAT_BMP,
    MTP_FORMAT_GIF,
    MTP_FORMAT_PNG,
    MTP_FORMAT_MP3,
    MTP_FORMAT_WAV,
    MTP_FORMAT_MPEG,
    MTP_FORMAT_AVI,
    MTP_FORMAT_WMA,
    MTP_FORMAT_OGG,
    MTP_FORMAT_AAC,
    MTP_FORMAT_FLAC,
    MTP_FORMAT_MP4_CONTAINER,
    MTP_FORMAT_3GP_CONTAINER,
    MTP_FORMAT_XML_DOCUMENT,
    MTP_FORMAT_MS_WORD_DOCUMENT,
    MTP_FORMAT_MS_EXCEL_SPREADSHEET,
    MTP_FORMAT_MS_POWERPOINT_PRESENTATION,
};

static const uint16_t g_mtp_supported_object_properties[] = {
    MTP_PROPERTY_STORAGE_ID,
    MTP_PROPERTY_OBJECT_FORMAT,
    MTP_PROPERTY_PROTECTION_STATUS,
    MTP_PROPERTY_OBJECT_SIZE,
    MTP_PROPERTY_OBJECT_FILE_NAME,
    MTP_PROPERTY_DATE_CREATED,
    MTP_PROPERTY_DATE_MODIFIED,
    MTP_PROPERTY_PARENT_OBJECT,
    MTP_PROPERTY_PERSISTENT_UID,
    MTP_PROPERTY_NAME,
    MTP_PROPERTY_DISPLAY_NAME,
    MTP_PROPERTY_ASSOCIATION_TYPE,
    MTP_PROPERTY_ASSOCIATION_DESC,
    MTP_PROPERTY_HIDDEN,
    MTP_PROPERTY_SYSTEM_OBJECT,
};

static void mtp_start_command_read(uint8_t busid);
static void mtp_send_response(uint8_t busid, uint16_t code, uint8_t argc, const uint32_t *argv);

static void mtp_put_u8(struct mtp_builder *b, uint8_t value)
{
    if (b->len + 1 > b->cap) {
        b->overflow = true;
        return;
    }
    b->buf[b->len++] = value;
}

static void mtp_put_u16(struct mtp_builder *b, uint16_t value)
{
    if (b->len + 2 > b->cap) {
        b->overflow = true;
        return;
    }
    b->buf[b->len++] = (uint8_t)value;
    b->buf[b->len++] = (uint8_t)(value >> 8);
}

static void mtp_put_u32(struct mtp_builder *b, uint32_t value)
{
    if (b->len + 4 > b->cap) {
        b->overflow = true;
        return;
    }
    b->buf[b->len++] = (uint8_t)value;
    b->buf[b->len++] = (uint8_t)(value >> 8);
    b->buf[b->len++] = (uint8_t)(value >> 16);
    b->buf[b->len++] = (uint8_t)(value >> 24);
}

static void mtp_put_u64(struct mtp_builder *b, uint64_t value)
{
    mtp_put_u32(b, (uint32_t)value);
    mtp_put_u32(b, (uint32_t)(value >> 32));
}

static void mtp_put_zeros(struct mtp_builder *b, uint32_t count)
{
    if (b->len + count > b->cap) {
        b->overflow = true;
        return;
    }
    memset(&b->buf[b->len], 0, count);
    b->len += count;
}

static uint32_t mtp_strlen_limit(const char *str)
{
    uint32_t len = 0;

    if (str == NULL) {
        return 0;
    }

    while (str[len] != '\0' && len < 254) {
        len++;
    }

    return len;
}

static void mtp_put_string(struct mtp_builder *b, const char *str)
{
    uint32_t len = mtp_strlen_limit(str);

    if (len == 0) {
        mtp_put_u8(b, 0);
        return;
    }

    mtp_put_u8(b, (uint8_t)(len + 1));
    for (uint32_t i = 0; i < len; i++) {
        mtp_put_u16(b, (uint8_t)str[i]);
    }
    mtp_put_u16(b, 0);
}

static void mtp_put_u16_array(struct mtp_builder *b, const uint16_t *array, uint32_t count)
{
    mtp_put_u32(b, count);
    for (uint32_t i = 0; i < count; i++) {
        mtp_put_u16(b, array[i]);
    }
}

static void mtp_put_u32_array(struct mtp_builder *b, const uint32_t *array, uint32_t count)
{
    mtp_put_u32(b, count);
    for (uint32_t i = 0; i < count; i++) {
        mtp_put_u32(b, array[i]);
    }
}

static uint16_t mtp_get_u16(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static uint32_t mtp_get_u32(const uint8_t *buf)
{
    return (uint32_t)buf[0] |
           ((uint32_t)buf[1] << 8) |
           ((uint32_t)buf[2] << 16) |
           ((uint32_t)buf[3] << 24);
}

static bool mtp_reader_get_u16(struct mtp_reader *r, uint16_t *value)
{
    if (r->ptr + 2 > r->end) {
        return false;
    }
    *value = mtp_get_u16(r->ptr);
    r->ptr += 2;
    return true;
}

static bool mtp_reader_get_u32(struct mtp_reader *r, uint32_t *value)
{
    if (r->ptr + 4 > r->end) {
        return false;
    }
    *value = mtp_get_u32(r->ptr);
    r->ptr += 4;
    return true;
}

static bool mtp_reader_skip(struct mtp_reader *r, uint32_t len)
{
    if (r->ptr + len > r->end) {
        return false;
    }
    r->ptr += len;
    return true;
}

static bool mtp_reader_get_string(struct mtp_reader *r, char *out, uint32_t out_len)
{
    uint8_t chars;
    uint32_t copy_len;

    if (out_len == 0 || r->ptr + 1 > r->end) {
        return false;
    }

    chars = *r->ptr++;
    out[0] = '\0';

    if (chars == 0) {
        return true;
    }

    if (r->ptr + ((uint32_t)chars * 2U) > r->end) {
        return false;
    }

    copy_len = MIN((uint32_t)chars, out_len - 1U);
    for (uint32_t i = 0; i < copy_len; i++) {
        uint16_t wc = mtp_get_u16(&r->ptr[i * 2U]);
        if (wc == 0) {
            out[i] = '\0';
            r->ptr += (uint32_t)chars * 2U;
            return true;
        }
        out[i] = (wc < 0x80U) ? (char)wc : '_';
    }
    out[copy_len] = '\0';
    r->ptr += (uint32_t)chars * 2U;
    return true;
}

static void mtp_write_container_header(uint8_t *buf, uint32_t len, uint16_t type, uint16_t code, uint32_t trans_id)
{
    buf[0] = (uint8_t)len;
    buf[1] = (uint8_t)(len >> 8);
    buf[2] = (uint8_t)(len >> 16);
    buf[3] = (uint8_t)(len >> 24);
    buf[4] = (uint8_t)type;
    buf[5] = (uint8_t)(type >> 8);
    buf[6] = (uint8_t)code;
    buf[7] = (uint8_t)(code >> 8);
    buf[8] = (uint8_t)trans_id;
    buf[9] = (uint8_t)(trans_id >> 8);
    buf[10] = (uint8_t)(trans_id >> 16);
    buf[11] = (uint8_t)(trans_id >> 24);
}

static void mtp_prepare_data_builder(uint8_t busid, struct mtp_builder *b, uint16_t code)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    b->buf = g_mtp_tx_buffer[busid];
    b->cap = CONFIG_USBDEV_MTP_MAX_BUFSIZE;
    b->len = MTP_CONTAINER_HEADER_SIZE;
    b->overflow = false;
    mtp_write_container_header(b->buf, MTP_CONTAINER_HEADER_SIZE, MTP_CONTAINER_TYPE_DATA, code, priv->transaction_id);
}

static void mtp_finalize_container(struct mtp_builder *b, uint16_t type, uint16_t code, uint32_t trans_id)
{
    mtp_write_container_header(b->buf, b->len, type, code, trans_id);
}

static void mtp_start_write(uint8_t busid, const uint8_t *buf, uint32_t len, enum mtp_tx_state state, bool allow_zlp)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    uint16_t mps;

    priv->tx_state = state;
    mps = usbd_get_ep_mps(busid, priv->in_ep);
    priv->zlp_pending = (allow_zlp && len != 0U && mps != 0U && ((len % mps) == 0U));
    usbd_ep_start_write(busid, priv->in_ep, (uint8_t *)buf, len);
}

static void mtp_start_response_write(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    struct mtp_builder b;

    b.buf = g_mtp_tx_buffer[busid];
    b.cap = CONFIG_USBDEV_MTP_MAX_BUFSIZE;
    b.len = MTP_CONTAINER_HEADER_SIZE;
    b.overflow = false;

    for (uint8_t i = 0; i < priv->response.argc; i++) {
        mtp_put_u32(&b, priv->response.argv[i]);
    }

    mtp_finalize_container(&b, MTP_CONTAINER_TYPE_RESPONSE, priv->response.code, priv->transaction_id);
    mtp_start_write(busid, b.buf, b.len, MTP_TX_RESPONSE, true);
}

static void mtp_send_response(uint8_t busid, uint16_t code, uint8_t argc, const uint32_t *argv)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (argc > 5U) {
        argc = 5U;
    }
    priv->response.code = code;
    priv->response.argc = argc;
    for (uint8_t i = 0; i < argc; i++) {
        priv->response.argv[i] = argv[i];
    }
    priv->response_after_data = false;
    mtp_start_response_write(busid);
}

static void mtp_send_data_response(uint8_t busid, struct mtp_builder *b, uint16_t response_code, uint8_t argc, const uint32_t *argv)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (b->overflow) {
        mtp_send_response(busid, MTP_RESPONSE_GENERAL_ERROR, 0, NULL);
        return;
    }

    if (argc > 5U) {
        argc = 5U;
    }
    priv->response.code = response_code;
    priv->response.argc = argc;
    for (uint8_t i = 0; i < argc; i++) {
        priv->response.argv[i] = argv[i];
    }
    priv->response_after_data = true;
    mtp_finalize_container(b, MTP_CONTAINER_TYPE_DATA, priv->operation_code, priv->transaction_id);
    mtp_start_write(busid, b->buf, b->len, MTP_TX_SIMPLE_DATA, true);
}

static const char *mtp_basename(const char *path)
{
    const char *base = path;

    if (path == NULL) {
        return "";
    }

    for (const char *p = path; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\') {
            base = p + 1;
        }
    }

    return base;
}

static bool mtp_is_dir_mode(uint32_t mode)
{
#ifdef S_ISDIR
    return S_ISDIR(mode);
#else
    return (mode & S_IFDIR) != 0;
#endif
}

static bool mtp_is_readonly_mode(uint32_t mode)
{
#if defined(S_IWUSR)
    return (mode & S_IWUSR) == 0;
#else
    (void)mode;
    return false;
#endif
}

static char mtp_ascii_lower(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return (char)(c - 'A' + 'a');
    }
    return c;
}

static bool mtp_ext_equal(const char *ext, const char *value)
{
    while (*ext != '\0' && *value != '\0') {
        if (mtp_ascii_lower(*ext) != mtp_ascii_lower(*value)) {
            return false;
        }
        ext++;
        value++;
    }
    return *ext == '\0' && *value == '\0';
}

static uint16_t mtp_format_from_name(const char *name, bool is_dir)
{
    const char *ext = NULL;

    if (is_dir) {
        return MTP_FORMAT_ASSOCIATION;
    }

    for (const char *p = name; *p != '\0'; p++) {
        if (*p == '.') {
            ext = p + 1;
        }
    }

    if (ext == NULL || *ext == '\0') {
        return MTP_FORMAT_UNDEFINED;
    }

    if (mtp_ext_equal(ext, "txt")) {
        return MTP_FORMAT_TEXT;
    } else if (mtp_ext_equal(ext, "htm") || mtp_ext_equal(ext, "html")) {
        return MTP_FORMAT_HTML;
    } else if (mtp_ext_equal(ext, "jpg") || mtp_ext_equal(ext, "jpeg")) {
        return MTP_FORMAT_EXIF_JPEG;
    } else if (mtp_ext_equal(ext, "bmp")) {
        return MTP_FORMAT_BMP;
    } else if (mtp_ext_equal(ext, "gif")) {
        return MTP_FORMAT_GIF;
    } else if (mtp_ext_equal(ext, "png")) {
        return MTP_FORMAT_PNG;
    } else if (mtp_ext_equal(ext, "mp3")) {
        return MTP_FORMAT_MP3;
    } else if (mtp_ext_equal(ext, "wav")) {
        return MTP_FORMAT_WAV;
    } else if (mtp_ext_equal(ext, "mpg") || mtp_ext_equal(ext, "mpeg")) {
        return MTP_FORMAT_MPEG;
    } else if (mtp_ext_equal(ext, "avi")) {
        return MTP_FORMAT_AVI;
    } else if (mtp_ext_equal(ext, "wma")) {
        return MTP_FORMAT_WMA;
    } else if (mtp_ext_equal(ext, "ogg")) {
        return MTP_FORMAT_OGG;
    } else if (mtp_ext_equal(ext, "aac")) {
        return MTP_FORMAT_AAC;
    } else if (mtp_ext_equal(ext, "flac")) {
        return MTP_FORMAT_FLAC;
    } else if (mtp_ext_equal(ext, "mp4") || mtp_ext_equal(ext, "m4v")) {
        return MTP_FORMAT_MP4_CONTAINER;
    } else if (mtp_ext_equal(ext, "3gp")) {
        return MTP_FORMAT_3GP_CONTAINER;
    } else if (mtp_ext_equal(ext, "xml")) {
        return MTP_FORMAT_XML_DOCUMENT;
    } else if (mtp_ext_equal(ext, "doc") || mtp_ext_equal(ext, "docx")) {
        return MTP_FORMAT_MS_WORD_DOCUMENT;
    } else if (mtp_ext_equal(ext, "xls") || mtp_ext_equal(ext, "xlsx")) {
        return MTP_FORMAT_MS_EXCEL_SPREADSHEET;
    } else if (mtp_ext_equal(ext, "ppt") || mtp_ext_equal(ext, "pptx")) {
        return MTP_FORMAT_MS_POWERPOINT_PRESENTATION;
    }

    return MTP_FORMAT_UNDEFINED;
}

static bool mtp_join_path(char *out, uint32_t out_len, const char *dir, const char *name)
{
    uint32_t dir_len;
    int ret;

    if (out_len == 0 || dir == NULL || name == NULL || name[0] == '\0') {
        return false;
    }

    dir_len = strlen(dir);
    if (dir_len > 0 && (dir[dir_len - 1] == '/' || dir[dir_len - 1] == '\\')) {
        ret = snprintf(out, out_len, "%s%s", dir, name);
    } else {
        ret = snprintf(out, out_len, "%s/%s", dir, name);
    }

    return ret > 0 && (uint32_t)ret < out_len;
}

static bool mtp_name_is_safe(const char *name)
{
    if (name == NULL || name[0] == '\0') {
        return false;
    }

    if ((name[0] == '.' && name[1] == '\0') ||
        (name[0] == '.' && name[1] == '.' && name[2] == '\0')) {
        return false;
    }

    for (const char *p = name; *p != '\0'; p++) {
        if (*p == '/' || *p == '\\' || *p == ':') {
            return false;
        }
    }

    return true;
}

static struct mtp_object *mtp_find_object(uint8_t busid, uint32_t handle)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (handle == 0) {
        return NULL;
    }

    for (uint32_t i = 0; i < CONFIG_USBDEV_MTP_MAX_OBJECTS; i++) {
        if (priv->objects[i].in_use && priv->objects[i].handle == handle) {
            return &priv->objects[i];
        }
    }

    return NULL;
}

static struct mtp_object *mtp_alloc_object(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    for (uint32_t i = 0; i < CONFIG_USBDEV_MTP_MAX_OBJECTS; i++) {
        if (!priv->objects[i].in_use) {
            memset(&priv->objects[i], 0, sizeof(priv->objects[i]));
            priv->objects[i].handle = ++priv->next_handle;
            priv->objects[i].storage_id = MTP_STORAGE_ID;
            priv->objects[i].in_use = true;
            return &priv->objects[i];
        }
    }

    return NULL;
}

static void mtp_fill_object_from_stat(struct mtp_object *object, const char *path, uint32_t parent, const struct stat *st)
{
    const char *base = mtp_basename(path);

    object->storage_id = MTP_STORAGE_ID;
    object->parent_handle = parent;
    object->is_dir = mtp_is_dir_mode(st->st_mode);
    object->is_readonly = mtp_is_readonly_mode(st->st_mode);
    object->is_hidden = base[0] == '.';
    object->format = mtp_format_from_name(base, object->is_dir);
    object->file_size = object->is_dir ? 0U : (uint32_t)st->st_size;
    object->file_full_name_length = strlen(path);
    strncpy(object->file_full_name, path, sizeof(object->file_full_name) - 1U);
    object->file_full_name[sizeof(object->file_full_name) - 1U] = '\0';
}

static int mtp_scan_dir(uint8_t busid, const char *path, uint32_t parent)
{
    MTP_DIR *dir;
    struct mtp_dirent *entry;
    int ret = 0;

    dir = usbd_mtp_opendir(path);
    if (dir == NULL) {
        return -1;
    }

    while ((entry = usbd_mtp_readdir(dir)) != NULL) {
        char child_path[CONFIG_USBDEV_MTP_MAX_PATHNAME];
        struct stat st;
        struct mtp_object *object;

        if (entry->d_name[0] == '\0' ||
            (entry->d_name[0] == '.' && entry->d_name[1] == '\0') ||
            (entry->d_name[0] == '.' && entry->d_name[1] == '.' && entry->d_name[2] == '\0')) {
            continue;
        }

        if (!mtp_join_path(child_path, sizeof(child_path), path, entry->d_name)) {
            ret = -1;
            break;
        }

        if (usbd_mtp_stat(child_path, &st) != 0) {
            continue;
        }

        object = mtp_alloc_object(busid);
        if (object == NULL) {
            ret = -1;
            break;
        }
        mtp_fill_object_from_stat(object, child_path, parent, &st);

        if (object->is_dir) {
            (void)mtp_scan_dir(busid, child_path, object->handle);
        }
    }

    (void)usbd_mtp_closedir(dir);
    return ret;
}

static void mtp_objects_rescan(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    const char *root = usbd_mtp_fs_root_path();

    memset(priv->objects, 0, sizeof(priv->objects));
    priv->next_handle = 0;
    if (root != NULL) {
        (void)mtp_scan_dir(busid, root, MTP_ROOT_PARENT);
    }
    priv->objects_valid = true;
}

static void mtp_ensure_objects(uint8_t busid)
{
    if (!g_usbd_mtp[busid].objects_valid) {
        mtp_objects_rescan(busid);
    }
}

static bool mtp_format_matches(uint16_t filter, uint16_t format)
{
    if (filter == 0 || filter == 0xffffU) {
        return true;
    }
    return filter == format;
}

static bool mtp_parent_matches(uint32_t filter, uint32_t parent)
{
    if (filter == MTP_ALL_HANDLES) {
        return true;
    }
    return filter == parent;
}

static void mtp_format_datetime(char *buf, uint32_t len, time_t t)
{
    struct tm *tm_value;

    if (buf == NULL || len == 0) {
        return;
    }

    buf[0] = '\0';
    if (t == (time_t)0) {
        return;
    }

    tm_value = localtime(&t);
    if (tm_value == NULL) {
        return;
    }

    (void)snprintf(buf, len, "%04d%02d%02dT%02d%02d%02d",
                   tm_value->tm_year + 1900,
                   tm_value->tm_mon + 1,
                   tm_value->tm_mday,
                   tm_value->tm_hour,
                   tm_value->tm_min,
                   tm_value->tm_sec);
}

static void mtp_put_object_info(struct mtp_builder *b, const struct mtp_object *object)
{
    const char *name = mtp_basename(object->file_full_name);
    struct stat st;
    char date[20];

    date[0] = '\0';
    if (usbd_mtp_stat(object->file_full_name, &st) == 0) {
        mtp_format_datetime(date, sizeof(date), st.st_mtime);
    }

    mtp_put_u32(b, object->storage_id);
    mtp_put_u16(b, object->format);
    mtp_put_u16(b, object->is_readonly ? 0x0001U : 0x0000U);
    mtp_put_u32(b, object->is_dir ? 0U : object->file_size);
    mtp_put_u16(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, object->parent_handle);
    mtp_put_u16(b, object->is_dir ? MTP_ASSOCIATION_TYPE_GENERIC_FOLDER : MTP_ASSOCIATION_TYPE_UNDEFINED);
    mtp_put_u32(b, 0);
    mtp_put_u32(b, 0);
    mtp_put_string(b, name);
    mtp_put_string(b, date);
    mtp_put_string(b, date);
    mtp_put_string(b, "");
}

static bool mtp_is_supported_object_property(uint16_t property)
{
    for (uint32_t i = 0; i < sizeof(g_mtp_supported_object_properties) / sizeof(g_mtp_supported_object_properties[0]); i++) {
        if (g_mtp_supported_object_properties[i] == property) {
            return true;
        }
    }
    return false;
}

static uint16_t mtp_property_type(uint16_t property)
{
    switch (property) {
        case MTP_PROPERTY_STORAGE_ID:
        case MTP_PROPERTY_PARENT_OBJECT:
        case MTP_PROPERTY_ASSOCIATION_DESC:
            return MTP_TYPE_UINT32;
        case MTP_PROPERTY_OBJECT_SIZE:
            return MTP_TYPE_UINT64;
        case MTP_PROPERTY_OBJECT_FORMAT:
        case MTP_PROPERTY_PROTECTION_STATUS:
        case MTP_PROPERTY_ASSOCIATION_TYPE:
        case MTP_PROPERTY_HIDDEN:
        case MTP_PROPERTY_SYSTEM_OBJECT:
            return MTP_TYPE_UINT16;
        case MTP_PROPERTY_PERSISTENT_UID:
            return MTP_TYPE_UINT128;
        case MTP_PROPERTY_OBJECT_FILE_NAME:
        case MTP_PROPERTY_DATE_CREATED:
        case MTP_PROPERTY_DATE_MODIFIED:
        case MTP_PROPERTY_NAME:
        case MTP_PROPERTY_DISPLAY_NAME:
            return MTP_TYPE_STR;
        default:
            return MTP_TYPE_UNDEFINED;
    }
}

static uint16_t mtp_device_property_type(uint16_t property)
{
    switch (property) {
        case MTP_DEVICE_PROPERTY_BATTERY_LEVEL:
            return MTP_TYPE_UINT16;
        case MTP_DEVICE_PROPERTY_DEVICE_FRIENDLY_NAME:
            return MTP_TYPE_STR;
        default:
            return MTP_TYPE_UNDEFINED;
    }
}

static void mtp_put_default_value(struct mtp_builder *b, uint16_t type)
{
    switch (type) {
        case MTP_TYPE_UINT8:
            mtp_put_u8(b, 0);
            break;
        case MTP_TYPE_UINT16:
            mtp_put_u16(b, 0);
            break;
        case MTP_TYPE_UINT32:
            mtp_put_u32(b, 0);
            break;
        case MTP_TYPE_UINT64:
            mtp_put_u64(b, 0);
            break;
        case MTP_TYPE_UINT128:
            mtp_put_zeros(b, 16);
            break;
        case MTP_TYPE_STR:
            mtp_put_string(b, "");
            break;
        default:
            break;
    }
}

static void mtp_put_object_property_value(struct mtp_builder *b, const struct mtp_object *object, uint16_t property, bool include_type)
{
    const char *name = mtp_basename(object->file_full_name);
    uint16_t type = mtp_property_type(property);
    struct stat st;
    char date[20];

    if (include_type) {
        mtp_put_u16(b, type);
    }

    date[0] = '\0';
    if ((property == MTP_PROPERTY_DATE_CREATED || property == MTP_PROPERTY_DATE_MODIFIED) &&
        usbd_mtp_stat(object->file_full_name, &st) == 0) {
        mtp_format_datetime(date, sizeof(date), st.st_mtime);
    }

    switch (property) {
        case MTP_PROPERTY_STORAGE_ID:
            mtp_put_u32(b, object->storage_id);
            break;
        case MTP_PROPERTY_OBJECT_FORMAT:
            mtp_put_u16(b, object->format);
            break;
        case MTP_PROPERTY_PROTECTION_STATUS:
            mtp_put_u16(b, object->is_readonly ? 0x0001U : 0x0000U);
            break;
        case MTP_PROPERTY_OBJECT_SIZE:
            mtp_put_u64(b, object->is_dir ? 0U : object->file_size);
            break;
        case MTP_PROPERTY_OBJECT_FILE_NAME:
        case MTP_PROPERTY_NAME:
        case MTP_PROPERTY_DISPLAY_NAME:
            mtp_put_string(b, name);
            break;
        case MTP_PROPERTY_DATE_CREATED:
        case MTP_PROPERTY_DATE_MODIFIED:
            mtp_put_string(b, date);
            break;
        case MTP_PROPERTY_PARENT_OBJECT:
            mtp_put_u32(b, object->parent_handle);
            break;
        case MTP_PROPERTY_PERSISTENT_UID:
            mtp_put_u32(b, object->handle);
            mtp_put_u32(b, object->storage_id);
            mtp_put_u32(b, 0);
            mtp_put_u32(b, 0);
            break;
        case MTP_PROPERTY_ASSOCIATION_TYPE:
            mtp_put_u16(b, object->is_dir ? MTP_ASSOCIATION_TYPE_GENERIC_FOLDER : MTP_ASSOCIATION_TYPE_UNDEFINED);
            break;
        case MTP_PROPERTY_ASSOCIATION_DESC:
            mtp_put_u32(b, 0);
            break;
        case MTP_PROPERTY_HIDDEN:
            mtp_put_u16(b, object->is_hidden ? 1U : 0U);
            break;
        case MTP_PROPERTY_SYSTEM_OBJECT:
            mtp_put_u16(b, 0);
            break;
        default:
            mtp_put_default_value(b, type);
            break;
    }
}

static void mtp_close_transfer_file(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (priv->fd >= 0) {
        (void)usbd_mtp_close(priv->fd);
        priv->fd = -1;
    }
}

static void mtp_clear_pending_transfer(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    mtp_close_transfer_file(busid);
    priv->rx_state = MTP_RX_COMMAND;
    priv->tx_state = MTP_TX_IDLE;
    priv->send_info_valid = false;
    priv->transfer_handle = 0;
    priv->transfer_total = 0;
    priv->transfer_done = 0;
    priv->transfer_remaining = 0;
    priv->response_after_data = false;
    priv->zlp_pending = false;
}

static void mtp_reset_state(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    mtp_clear_pending_transfer(busid);
    priv->session_open = false;
    priv->session_id = 0;
    priv->transaction_id = 0;
    priv->operation_code = 0;
    priv->cancelled = false;
    priv->objects_valid = false;
}

static int mtp_delete_path_recursive(const char *path)
{
    struct stat st;

    if (usbd_mtp_stat(path, &st) != 0) {
        return -1;
    }

    if (!mtp_is_dir_mode(st.st_mode)) {
        return usbd_mtp_unlink(path);
    }

    MTP_DIR *dir = usbd_mtp_opendir(path);
    if (dir != NULL) {
        struct mtp_dirent *entry;

        while ((entry = usbd_mtp_readdir(dir)) != NULL) {
            char child_path[CONFIG_USBDEV_MTP_MAX_PATHNAME];

            if (entry->d_name[0] == '\0' ||
                (entry->d_name[0] == '.' && entry->d_name[1] == '\0') ||
                (entry->d_name[0] == '.' && entry->d_name[1] == '.' && entry->d_name[2] == '\0')) {
                continue;
            }

            if (!mtp_join_path(child_path, sizeof(child_path), path, entry->d_name)) {
                (void)usbd_mtp_closedir(dir);
                return -1;
            }

            if (mtp_delete_path_recursive(child_path) != 0) {
                (void)usbd_mtp_closedir(dir);
                return -1;
            }
        }
        (void)usbd_mtp_closedir(dir);
    }

    return usbd_mtp_rmdir(path);
}

static void mtp_mark_object_removed(uint8_t busid, uint32_t handle)
{
    struct mtp_object *object = mtp_find_object(busid, handle);

    if (object != NULL) {
        object->in_use = false;
    }
}

static bool mtp_add_pending_object(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    struct mtp_object *object;
    struct stat st;

    object = mtp_alloc_object(busid);
    if (object == NULL) {
        return false;
    }

    if (usbd_mtp_stat(priv->send_object_path, &st) == 0) {
        mtp_fill_object_from_stat(object, priv->send_object_path, priv->send_parent_handle, &st);
    } else {
        object->storage_id = MTP_STORAGE_ID;
        object->parent_handle = priv->send_parent_handle;
        object->format = priv->send_object_format;
        object->is_dir = priv->send_object_format == MTP_FORMAT_ASSOCIATION;
        object->file_size = object->is_dir ? 0U : priv->send_object_size;
        object->file_full_name_length = strlen(priv->send_object_path);
        strncpy(object->file_full_name, priv->send_object_path, sizeof(object->file_full_name) - 1U);
        object->file_full_name[sizeof(object->file_full_name) - 1U] = '\0';
    }

    priv->send_object_handle = object->handle;
    return true;
}

static void mtp_operation_get_device_info(uint8_t busid)
{
    struct mtp_builder b;

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_DEVICE_INFO);
    mtp_put_u16(&b, MTP_STANDARD_VERSION);
    mtp_put_u32(&b, 0x00000006U);
    mtp_put_u16(&b, 100);
    mtp_put_string(&b, "microsoft.com: 1.0; android.com: 1.0;");
    mtp_put_u16(&b, 0);
    mtp_put_u16_array(&b, g_mtp_supported_operations, sizeof(g_mtp_supported_operations) / sizeof(g_mtp_supported_operations[0]));
    mtp_put_u16_array(&b, g_mtp_supported_events, sizeof(g_mtp_supported_events) / sizeof(g_mtp_supported_events[0]));
    mtp_put_u16_array(&b, g_mtp_supported_device_properties, sizeof(g_mtp_supported_device_properties) / sizeof(g_mtp_supported_device_properties[0]));
    mtp_put_u16_array(&b, NULL, 0);
    mtp_put_u16_array(&b, g_mtp_supported_formats, sizeof(g_mtp_supported_formats) / sizeof(g_mtp_supported_formats[0]));
    mtp_put_string(&b, CONFIG_USBDEV_MTP_MANUFACTURER);
    mtp_put_string(&b, CONFIG_USBDEV_MTP_MODEL);
    mtp_put_string(&b, CONFIG_USBDEV_MTP_DEVICE_VERSION);
    mtp_put_string(&b, CONFIG_USBDEV_MTP_SERIAL_NUMBER);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_storage_ids(uint8_t busid)
{
    struct mtp_builder b;
    uint32_t storage = MTP_STORAGE_ID;

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_STORAGE_IDS);
    mtp_put_u32_array(&b, &storage, 1);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_storage_info(uint8_t busid, uint32_t storage_id)
{
    struct mtp_builder b;
    struct mtp_statfs sfs;
    uint64_t total_bytes = 0;
    uint64_t free_bytes = 0;
    const char *description;

    if (storage_id != MTP_STORAGE_ID) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_STORAGE_ID, 0, NULL);
        return;
    }

    if (usbd_mtp_statfs(usbd_mtp_fs_root_path(), &sfs) == 0) {
        total_bytes = (uint64_t)sfs.f_bsize * (uint64_t)sfs.f_blocks;
        free_bytes = (uint64_t)sfs.f_bsize * (uint64_t)sfs.f_bfree;
    }

    description = usbd_mtp_fs_description();
    if (description == NULL) {
        description = CONFIG_USBDEV_MTP_MODEL;
    }

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_STORAGE_INFO);
    mtp_put_u16(&b, MTP_STORAGE_REMOVABLE_RAM);
    mtp_put_u16(&b, MTP_STORAGE_FILESYSTEM_HIERARCHICAL);
    mtp_put_u16(&b, MTP_STORAGE_READ_WRITE);
    mtp_put_u64(&b, total_bytes);
    mtp_put_u64(&b, free_bytes);
    mtp_put_u32(&b, CONFIG_USBDEV_MTP_MAX_OBJECTS);
    mtp_put_string(&b, description);
    mtp_put_string(&b, CONFIG_USBDEV_MTP_SERIAL_NUMBER);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_object_handles(uint8_t busid, uint32_t storage_id, uint16_t format, uint32_t parent)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    struct mtp_builder b;
    uint32_t count_offset;
    uint32_t count = 0;

    if (storage_id != MTP_STORAGE_ID && storage_id != MTP_ALL_HANDLES) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_STORAGE_ID, 0, NULL);
        return;
    }

    if (parent != MTP_ROOT_PARENT && parent != MTP_ALL_HANDLES) {
        struct mtp_object *parent_object = mtp_find_object(busid, parent);
        if (parent_object == NULL || !parent_object->is_dir) {
            mtp_send_response(busid, MTP_RESPONSE_INVALID_PARENT_OBJECT, 0, NULL);
            return;
        }
    }

    mtp_ensure_objects(busid);
    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_OBJECT_HANDLES);
    count_offset = b.len;
    mtp_put_u32(&b, 0);

    for (uint32_t i = 0; i < CONFIG_USBDEV_MTP_MAX_OBJECTS; i++) {
        struct mtp_object *object = &priv->objects[i];

        if (!object->in_use) {
            continue;
        }
        if (!mtp_format_matches(format, object->format)) {
            continue;
        }
        if (!mtp_parent_matches(parent, object->parent_handle)) {
            continue;
        }
        if (b.len + 4 > b.cap) {
            b.overflow = true;
            break;
        }
        mtp_put_u32(&b, object->handle);
        count++;
    }

    if (!b.overflow) {
        b.buf[count_offset] = (uint8_t)count;
        b.buf[count_offset + 1U] = (uint8_t)(count >> 8);
        b.buf[count_offset + 2U] = (uint8_t)(count >> 16);
        b.buf[count_offset + 3U] = (uint8_t)(count >> 24);
    }

    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_object_info(uint8_t busid, uint32_t handle)
{
    struct mtp_builder b;
    struct mtp_object *object;

    mtp_ensure_objects(busid);
    object = mtp_find_object(busid, handle);
    if (object == NULL) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_HANDLE, 0, NULL);
        return;
    }

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_OBJECT_INFO);
    mtp_put_object_info(&b, object);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_send_get_object_next(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    uint8_t *buf = g_mtp_tx_buffer[busid];
    uint32_t len;
    uint32_t payload_room = CONFIG_USBDEV_MTP_MAX_BUFSIZE;
    bool final_chunk;
    bool include_header = (priv->transfer_done == 0U);

    if (include_header) {
        payload_room -= MTP_CONTAINER_HEADER_SIZE;
        mtp_write_container_header(buf, MTP_CONTAINER_HEADER_SIZE + priv->transfer_total,
                                   MTP_CONTAINER_TYPE_DATA, MTP_OPERATION_GET_OBJECT,
                                   priv->transaction_id);
    }

    len = MIN(priv->transfer_remaining, payload_room);
    if (len > 0U) {
        int read_len = usbd_mtp_read(priv->fd,
                                     &buf[include_header ? MTP_CONTAINER_HEADER_SIZE : 0U],
                                     len);
        if (read_len <= 0) {
            mtp_close_transfer_file(busid);
            mtp_send_response(busid, MTP_RESPONSE_GENERAL_ERROR, 0, NULL);
            return;
        }
        len = (uint32_t)read_len;
    }

    priv->transfer_done += len;
    priv->transfer_remaining -= len;
    final_chunk = (priv->transfer_remaining == 0U);

    mtp_start_write(busid, buf,
                    len + (include_header ? MTP_CONTAINER_HEADER_SIZE : 0U),
                    MTP_TX_GET_OBJECT,
                    final_chunk);
}

static void mtp_operation_get_object(uint8_t busid, uint32_t handle)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    struct mtp_object *object;

    mtp_ensure_objects(busid);
    object = mtp_find_object(busid, handle);
    if (object == NULL) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_HANDLE, 0, NULL);
        return;
    }
    if (object->is_dir) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_HANDLE, 0, NULL);
        return;
    }

    priv->fd = usbd_mtp_open(object->file_full_name, O_RDONLY);
    if (priv->fd < 0) {
        mtp_send_response(busid, MTP_RESPONSE_ACCESS_DENIED, 0, NULL);
        return;
    }

    priv->transfer_handle = handle;
    priv->transfer_total = object->file_size;
    priv->transfer_done = 0;
    priv->transfer_remaining = object->file_size;
    mtp_send_get_object_next(busid);
}

static void mtp_operation_delete_object(uint8_t busid, uint32_t handle, uint16_t format)
{
    struct mtp_object *object;

    if (handle == MTP_ALL_HANDLES) {
        mtp_send_response(busid, MTP_RESPONSE_PARAMETER_NOT_SUPPORTED, 0, NULL);
        return;
    }

    mtp_ensure_objects(busid);
    object = mtp_find_object(busid, handle);
    if (object == NULL) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_HANDLE, 0, NULL);
        return;
    }
    if (!mtp_format_matches(format, object->format)) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_FORMAT_CODE, 0, NULL);
        return;
    }
    if (object->is_readonly) {
        mtp_send_response(busid, MTP_RESPONSE_OBJECT_WRITE_PROTECTED, 0, NULL);
        return;
    }

    if (mtp_delete_path_recursive(object->file_full_name) != 0) {
        mtp_send_response(busid, MTP_RESPONSE_ACCESS_DENIED, 0, NULL);
        return;
    }

    mtp_mark_object_removed(busid, handle);
    mtp_send_response(busid, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_device_prop_desc(uint8_t busid, uint16_t property)
{
    struct mtp_builder b;
    uint16_t type = mtp_device_property_type(property);

    if (type == MTP_TYPE_UNDEFINED) {
        mtp_send_response(busid, MTP_RESPONSE_DEVICE_PROP_NOT_SUPPORTED, 0, NULL);
        return;
    }

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_DEVICE_PROP_DESC);
    mtp_put_u16(&b, property);
    mtp_put_u16(&b, type);
    mtp_put_u8(&b, 0);
    if (property == MTP_DEVICE_PROPERTY_BATTERY_LEVEL) {
        mtp_put_u16(&b, 100);
        mtp_put_u16(&b, 100);
    } else {
        mtp_put_string(&b, CONFIG_USBDEV_MTP_MODEL);
        mtp_put_string(&b, CONFIG_USBDEV_MTP_MODEL);
    }
    mtp_put_u8(&b, 0);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_object_props_supported(uint8_t busid, uint16_t format)
{
    struct mtp_builder b;

    (void)format;
    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_OBJECT_PROPS_SUPPORTED);
    mtp_put_u16_array(&b, g_mtp_supported_object_properties, sizeof(g_mtp_supported_object_properties) / sizeof(g_mtp_supported_object_properties[0]));
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_object_prop_desc(uint8_t busid, uint16_t property, uint16_t format)
{
    struct mtp_builder b;
    uint16_t type = mtp_property_type(property);

    (void)format;
    if (!mtp_is_supported_object_property(property) || type == MTP_TYPE_UNDEFINED) {
        mtp_send_response(busid, MTP_RESPONSE_OBJECT_PROP_NOT_SUPPORTED, 0, NULL);
        return;
    }

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_OBJECT_PROP_DESC);
    mtp_put_u16(&b, property);
    mtp_put_u16(&b, type);
    mtp_put_u8(&b, property == MTP_PROPERTY_OBJECT_FILE_NAME ? 1U : 0U);
    mtp_put_default_value(&b, type);
    mtp_put_u32(&b, 0);
    mtp_put_u8(&b, 0);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_operation_get_object_prop_value(uint8_t busid, uint32_t handle, uint16_t property)
{
    struct mtp_builder b;
    struct mtp_object *object;

    if (!mtp_is_supported_object_property(property)) {
        mtp_send_response(busid, MTP_RESPONSE_OBJECT_PROP_NOT_SUPPORTED, 0, NULL);
        return;
    }

    mtp_ensure_objects(busid);
    object = mtp_find_object(busid, handle);
    if (object == NULL) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_OBJECT_HANDLE, 0, NULL);
        return;
    }

    mtp_prepare_data_builder(busid, &b, MTP_OPERATION_GET_OBJECT_PROP_VALUE);
    mtp_put_object_property_value(&b, object, property, false);
    mtp_send_data_response(busid, &b, MTP_RESPONSE_OK, 0, NULL);
}

static void mtp_process_send_object_info(uint8_t busid, uint32_t nbytes)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    struct mtp_reader r;
    const uint8_t *buf = g_mtp_rx_buffer[busid];
    uint32_t container_len;
    uint16_t container_type;
    uint16_t code;
    uint32_t trans_id;
    uint32_t storage_id;
    uint16_t format;
    uint32_t size;
    uint32_t parent;
    char filename[CONFIG_USBDEV_MTP_MAX_PATHNAME];
    char target_path[CONFIG_USBDEV_MTP_MAX_PATHNAME];
    const char *parent_path;
    uint32_t response_params[3];

    if (nbytes < MTP_CONTAINER_HEADER_SIZE) {
        mtp_send_response(busid, MTP_RESPONSE_INCOMPLETE_TRANSFER, 0, NULL);
        return;
    }

    container_len = mtp_get_u32(&buf[MTP_CONTAINER_LENGTH_OFFSET]);
    container_type = mtp_get_u16(&buf[MTP_CONTAINER_TYPE_OFFSET]);
    code = mtp_get_u16(&buf[MTP_CONTAINER_CODE_OFFSET]);
    trans_id = mtp_get_u32(&buf[MTP_CONTAINER_TRANSACTION_ID_OFFSET]);

    if (container_type != MTP_CONTAINER_TYPE_DATA ||
        code != MTP_OPERATION_SEND_OBJECT_INFO ||
        trans_id != priv->transaction_id ||
        container_len > nbytes) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_DATASET, 0, NULL);
        return;
    }

    r.ptr = &buf[MTP_CONTAINER_HEADER_SIZE];
    r.end = &buf[container_len];

    if (!mtp_reader_get_u32(&r, &storage_id) ||
        !mtp_reader_get_u16(&r, &format) ||
        !mtp_reader_skip(&r, 2) ||
        !mtp_reader_get_u32(&r, &size) ||
        !mtp_reader_skip(&r, 2 + 4 + 4 + 4 + 4 + 4 + 4) ||
        !mtp_reader_get_u32(&r, &parent) ||
        !mtp_reader_skip(&r, 2 + 4 + 4) ||
        !mtp_reader_get_string(&r, filename, sizeof(filename))) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_DATASET, 0, NULL);
        return;
    }

    if (storage_id == 0U) {
        storage_id = priv->send_storage_id;
    }
    if (storage_id == 0U) {
        storage_id = MTP_STORAGE_ID;
    }
    if (storage_id != MTP_STORAGE_ID) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_STORAGE_ID, 0, NULL);
        return;
    }

    if (parent == MTP_ALL_HANDLES) {
        parent = priv->send_parent_handle;
    }
    if (parent == MTP_ALL_HANDLES) {
        parent = MTP_ROOT_PARENT;
    }

    if (!mtp_name_is_safe(filename)) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_DATASET, 0, NULL);
        return;
    }

    if (parent == MTP_ROOT_PARENT) {
        parent_path = usbd_mtp_fs_root_path();
    } else {
        struct mtp_object *parent_object;

        mtp_ensure_objects(busid);
        parent_object = mtp_find_object(busid, parent);
        if (parent_object == NULL || !parent_object->is_dir) {
            mtp_send_response(busid, MTP_RESPONSE_INVALID_PARENT_OBJECT, 0, NULL);
            return;
        }
        parent_path = parent_object->file_full_name;
    }

    if (parent_path == NULL || !mtp_join_path(target_path, sizeof(target_path), parent_path, filename)) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_DATASET, 0, NULL);
        return;
    }

    priv->send_storage_id = storage_id;
    priv->send_parent_handle = parent;
    priv->send_object_format = format;
    priv->send_object_size = size;
    strncpy(priv->send_object_path, target_path, sizeof(priv->send_object_path) - 1U);
    priv->send_object_path[sizeof(priv->send_object_path) - 1U] = '\0';
    priv->send_info_valid = true;
    priv->send_object_handle = 0;

    if (format == MTP_FORMAT_ASSOCIATION) {
        if (usbd_mtp_mkdir(priv->send_object_path) != 0) {
            mtp_send_response(busid, MTP_RESPONSE_ACCESS_DENIED, 0, NULL);
            return;
        }
        if (!mtp_add_pending_object(busid)) {
            (void)usbd_mtp_rmdir(priv->send_object_path);
            mtp_send_response(busid, MTP_RESPONSE_STORAGE_FULL, 0, NULL);
            return;
        }
        priv->send_info_valid = false;
    } else {
        if (!mtp_add_pending_object(busid)) {
            mtp_send_response(busid, MTP_RESPONSE_STORAGE_FULL, 0, NULL);
            return;
        }
    }

    response_params[0] = storage_id;
    response_params[1] = parent;
    response_params[2] = priv->send_object_handle;
    mtp_send_response(busid, MTP_RESPONSE_OK, 3, response_params);
}

static void mtp_process_send_object_chunk(uint8_t busid, uint32_t nbytes)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    const uint8_t *buf = g_mtp_rx_buffer[busid];
    uint32_t payload_offset = 0;
    uint32_t payload_len = nbytes;

    if (priv->transfer_done == 0U) {
        uint32_t container_len;
        uint16_t container_type;
        uint16_t code;
        uint32_t trans_id;

        if (nbytes < MTP_CONTAINER_HEADER_SIZE) {
            mtp_close_transfer_file(busid);
            mtp_send_response(busid, MTP_RESPONSE_INCOMPLETE_TRANSFER, 0, NULL);
            return;
        }

        container_len = mtp_get_u32(&buf[MTP_CONTAINER_LENGTH_OFFSET]);
        container_type = mtp_get_u16(&buf[MTP_CONTAINER_TYPE_OFFSET]);
        code = mtp_get_u16(&buf[MTP_CONTAINER_CODE_OFFSET]);
        trans_id = mtp_get_u32(&buf[MTP_CONTAINER_TRANSACTION_ID_OFFSET]);

        if (container_type != MTP_CONTAINER_TYPE_DATA ||
            code != MTP_OPERATION_SEND_OBJECT ||
            trans_id != priv->transaction_id ||
            container_len < MTP_CONTAINER_HEADER_SIZE) {
            mtp_close_transfer_file(busid);
            mtp_send_response(busid, MTP_RESPONSE_INVALID_DATASET, 0, NULL);
            return;
        }

        priv->transfer_total = container_len - MTP_CONTAINER_HEADER_SIZE;
        priv->transfer_remaining = priv->transfer_total;
        payload_offset = MTP_CONTAINER_HEADER_SIZE;
        payload_len = nbytes - MTP_CONTAINER_HEADER_SIZE;
    }

    if (payload_len > priv->transfer_remaining) {
        payload_len = priv->transfer_remaining;
    }

    if (payload_len > 0U) {
        int written = usbd_mtp_write(priv->fd, &buf[payload_offset], payload_len);
        if (written < 0 || (uint32_t)written != payload_len) {
            mtp_close_transfer_file(busid);
            mtp_send_response(busid, MTP_RESPONSE_GENERAL_ERROR, 0, NULL);
            return;
        }
    }

    priv->transfer_done += payload_len;
    priv->transfer_remaining -= payload_len;

    if (priv->transfer_remaining == 0U) {
        struct mtp_object *object = mtp_find_object(busid, priv->transfer_handle);

        mtp_close_transfer_file(busid);
        if (object != NULL) {
            struct stat st;
            if (usbd_mtp_stat(object->file_full_name, &st) == 0) {
                object->file_size = (uint32_t)st.st_size;
            } else {
                object->file_size = priv->transfer_total;
            }
        }
        priv->send_info_valid = false;
        mtp_send_response(busid, MTP_RESPONSE_OK, 0, NULL);
    } else {
        usbd_ep_start_read(busid, priv->out_ep, g_mtp_rx_buffer[busid], CONFIG_USBDEV_MTP_MAX_BUFSIZE);
    }
}

static void mtp_operation_send_object(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (!priv->send_info_valid || priv->send_object_handle == 0U) {
        mtp_send_response(busid, MTP_RESPONSE_NO_VALID_OBJECT_INFO, 0, NULL);
        return;
    }

    priv->fd = usbd_mtp_open(priv->send_object_path, O_WRONLY);
    if (priv->fd < 0) {
        mtp_mark_object_removed(busid, priv->send_object_handle);
        priv->send_info_valid = false;
        mtp_send_response(busid, MTP_RESPONSE_ACCESS_DENIED, 0, NULL);
        return;
    }

    priv->transfer_handle = priv->send_object_handle;
    priv->transfer_total = 0;
    priv->transfer_done = 0;
    priv->transfer_remaining = 0;
    priv->rx_state = MTP_RX_SEND_OBJECT;
    usbd_ep_start_read(busid, priv->out_ep, g_mtp_rx_buffer[busid], CONFIG_USBDEV_MTP_MAX_BUFSIZE);
}

static void mtp_operation_send_object_info(uint8_t busid, uint32_t storage_id, uint32_t parent)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (storage_id == 0U || storage_id == MTP_ALL_HANDLES) {
        storage_id = MTP_STORAGE_ID;
    }
    if (storage_id != MTP_STORAGE_ID) {
        mtp_send_response(busid, MTP_RESPONSE_INVALID_STORAGE_ID, 0, NULL);
        return;
    }
    if (parent == MTP_ALL_HANDLES) {
        parent = MTP_ROOT_PARENT;
    }

    priv->send_storage_id = storage_id;
    priv->send_parent_handle = parent;
    priv->send_info_valid = false;
    priv->rx_state = MTP_RX_SEND_OBJECT_INFO;
    usbd_ep_start_read(busid, priv->out_ep, g_mtp_rx_buffer[busid], CONFIG_USBDEV_MTP_MAX_BUFSIZE);
}

static void mtp_dispatch_command(uint8_t busid, uint32_t nbytes)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    const uint8_t *buf = g_mtp_rx_buffer[busid];
    uint32_t container_len;
    uint16_t container_type;
    uint16_t op;
    uint32_t params[5] = { 0 };
    uint8_t argc = 0;

    if (nbytes < MTP_CONTAINER_HEADER_SIZE) {
        mtp_start_command_read(busid);
        return;
    }

    container_len = mtp_get_u32(&buf[MTP_CONTAINER_LENGTH_OFFSET]);
    container_type = mtp_get_u16(&buf[MTP_CONTAINER_TYPE_OFFSET]);
    op = mtp_get_u16(&buf[MTP_CONTAINER_CODE_OFFSET]);
    priv->transaction_id = mtp_get_u32(&buf[MTP_CONTAINER_TRANSACTION_ID_OFFSET]);
    priv->operation_code = op;
    priv->cancelled = false;

    if (container_type != MTP_CONTAINER_TYPE_COMMAND ||
        container_len < MTP_CONTAINER_HEADER_SIZE ||
        container_len > nbytes) {
        mtp_start_command_read(busid);
        return;
    }

    argc = (uint8_t)MIN((container_len - MTP_CONTAINER_HEADER_SIZE) / 4U, 5U);
    for (uint8_t i = 0; i < argc; i++) {
        params[i] = mtp_get_u32(&buf[MTP_CONTAINER_PARAMETER_OFFSET + ((uint32_t)i * 4U)]);
    }

    if (op != MTP_OPERATION_GET_DEVICE_INFO &&
        op != MTP_OPERATION_OPEN_SESSION &&
        op != MTP_OPERATION_CLOSE_SESSION &&
        !priv->session_open) {
        mtp_send_response(busid, MTP_RESPONSE_SESSION_NOT_OPEN, 0, NULL);
        return;
    }

    switch (op) {
        case MTP_OPERATION_GET_DEVICE_INFO:
            mtp_operation_get_device_info(busid);
            break;
        case MTP_OPERATION_OPEN_SESSION:
            if (priv->session_open) {
                mtp_send_response(busid, MTP_RESPONSE_SESSION_ALREADY_OPEN, 0, NULL);
            } else {
                priv->session_open = true;
                priv->session_id = params[0];
                priv->objects_valid = false;
                mtp_send_response(busid, MTP_RESPONSE_OK, 0, NULL);
            }
            break;
        case MTP_OPERATION_CLOSE_SESSION:
            priv->session_open = false;
            priv->session_id = 0;
            mtp_clear_pending_transfer(busid);
            mtp_send_response(busid, MTP_RESPONSE_OK, 0, NULL);
            break;
        case MTP_OPERATION_GET_STORAGE_IDS:
            mtp_operation_get_storage_ids(busid);
            break;
        case MTP_OPERATION_GET_STORAGE_INFO:
            mtp_operation_get_storage_info(busid, params[0]);
            break;
        case MTP_OPERATION_GET_OBJECT_HANDLES:
            mtp_operation_get_object_handles(busid, params[0], (uint16_t)params[1], params[2]);
            break;
        case MTP_OPERATION_GET_OBJECT_INFO:
            mtp_operation_get_object_info(busid, params[0]);
            break;
        case MTP_OPERATION_GET_OBJECT:
            mtp_operation_get_object(busid, params[0]);
            break;
        case MTP_OPERATION_DELETE_OBJECT:
            mtp_operation_delete_object(busid, params[0], (uint16_t)params[1]);
            break;
        case MTP_OPERATION_SEND_OBJECT_INFO:
            mtp_operation_send_object_info(busid, params[0], params[1]);
            break;
        case MTP_OPERATION_SEND_OBJECT:
            mtp_operation_send_object(busid);
            break;
        case MTP_OPERATION_GET_DEVICE_PROP_DESC:
            mtp_operation_get_device_prop_desc(busid, (uint16_t)params[0]);
            break;
        case MTP_OPERATION_GET_OBJECT_PROPS_SUPPORTED:
            mtp_operation_get_object_props_supported(busid, (uint16_t)params[0]);
            break;
        case MTP_OPERATION_GET_OBJECT_PROP_DESC:
            mtp_operation_get_object_prop_desc(busid, (uint16_t)params[0], (uint16_t)params[1]);
            break;
        case MTP_OPERATION_GET_OBJECT_PROP_VALUE:
            mtp_operation_get_object_prop_value(busid, params[0], (uint16_t)params[1]);
            break;
        default:
            mtp_send_response(busid, MTP_RESPONSE_OPERATION_NOT_SUPPORTED, 0, NULL);
            break;
    }
}

static void mtp_handle_out(uint8_t busid, uint32_t nbytes)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    switch (priv->rx_state) {
        case MTP_RX_COMMAND:
            mtp_dispatch_command(busid, nbytes);
            break;
        case MTP_RX_SEND_OBJECT_INFO:
            priv->rx_state = MTP_RX_COMMAND;
            mtp_process_send_object_info(busid, nbytes);
            break;
        case MTP_RX_SEND_OBJECT:
            mtp_process_send_object_chunk(busid, nbytes);
            break;
        default:
            mtp_start_command_read(busid);
            break;
    }
}

static void mtp_handle_in(uint8_t busid, uint32_t nbytes)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    (void)nbytes;

    if (priv->zlp_pending) {
        priv->zlp_pending = false;
        usbd_ep_start_write(busid, priv->in_ep, NULL, 0);
        return;
    }

    switch (priv->tx_state) {
        case MTP_TX_SIMPLE_DATA:
            if (priv->response_after_data) {
                priv->response_after_data = false;
                mtp_start_response_write(busid);
            } else {
                mtp_start_command_read(busid);
            }
            break;
        case MTP_TX_RESPONSE:
            priv->tx_state = MTP_TX_IDLE;
            mtp_start_command_read(busid);
            break;
        case MTP_TX_GET_OBJECT:
            if (priv->transfer_remaining > 0U) {
                mtp_send_get_object_next(busid);
            } else {
                mtp_close_transfer_file(busid);
                mtp_send_response(busid, MTP_RESPONSE_OK, 0, NULL);
            }
            break;
        default:
            priv->tx_state = MTP_TX_IDLE;
            break;
    }
}

static void mtp_worker_run(uint8_t busid, uintptr_t event)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    switch (event) {
        case MTP_WORK_OUT:
            mtp_handle_out(busid, priv->last_nbytes);
            break;
        case MTP_WORK_IN:
            mtp_handle_in(busid, priv->last_nbytes);
            break;
        default:
            break;
    }
}

#if defined(CONFIG_USBDEV_MTP_THREAD)
static void usbdev_mtp_thread(CONFIG_USB_OSAL_THREAD_SET_ARGV)
{
    uintptr_t event;
    int ret;
    uint8_t busid = (uint8_t)CONFIG_USB_OSAL_THREAD_GET_ARGV;

    while (1) {
        ret = usb_osal_mq_recv(g_usbd_mtp[busid].mq, &event, USB_OSAL_WAITING_FOREVER);
        if (ret < 0) {
            continue;
        }
        mtp_worker_run(busid, event);
    }
}
#endif

static void mtp_bulk_out(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;

    g_usbd_mtp[busid].last_nbytes = nbytes;
#if defined(CONFIG_USBDEV_MTP_THREAD)
    (void)usb_osal_mq_send(g_usbd_mtp[busid].mq, MTP_WORK_OUT);
#else
    mtp_worker_run(busid, MTP_WORK_OUT);
#endif
}

static void mtp_bulk_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)ep;

    g_usbd_mtp[busid].last_nbytes = nbytes;
#if defined(CONFIG_USBDEV_MTP_THREAD)
    (void)usb_osal_mq_send(g_usbd_mtp[busid].mq, MTP_WORK_IN);
#else
    mtp_worker_run(busid, MTP_WORK_IN);
#endif
}

static void mtp_int_in(uint8_t busid, uint8_t ep, uint32_t nbytes)
{
    (void)busid;
    (void)ep;
    (void)nbytes;
}

static void mtp_start_command_read(uint8_t busid)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    if (!priv->configured) {
        return;
    }

    priv->rx_state = MTP_RX_COMMAND;
    usbd_ep_start_read(busid, priv->out_ep, g_mtp_rx_buffer[busid], CONFIG_USBDEV_MTP_MAX_BUFSIZE);
}

static int mtp_class_interface_request_handler(uint8_t busid, struct usb_setup_packet *setup, uint8_t **data, uint32_t *len)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    switch (setup->bRequest) {
        case MTP_REQUEST_CANCEL:
            priv->cancelled = true;
            mtp_clear_pending_transfer(busid);
            *len = 0;
            break;
        case MTP_REQUEST_RESET:
            mtp_reset_state(busid);
            *len = 0;
            break;
        case MTP_REQUEST_GET_DEVICE_STATUS:
            (*data)[0] = 0x04;
            (*data)[1] = 0x00;
            (*data)[2] = MTP_RESPONSE_OK & 0xff;
            (*data)[3] = MTP_RESPONSE_OK >> 8;
            *len = 4;
            break;
        default:
            return -1;
    }

    return 0;
}

static void mtp_notify_handler(uint8_t busid, uint8_t event, void *arg)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

    (void)arg;

    switch (event) {
        case USBD_EVENT_INIT:
#if defined(CONFIG_USBDEV_MTP_THREAD)
            priv->mq = usb_osal_mq_create(1);
            if (priv->mq == NULL) {
                USB_LOG_ERR("No memory to alloc for mtp mq\r\n");
            }
            priv->thread = usb_osal_thread_create("usbd_mtp", CONFIG_USBDEV_MTP_STACKSIZE,
                                                  CONFIG_USBDEV_MTP_PRIO, usbdev_mtp_thread,
                                                  (void *)(uintptr_t)busid);
            if (priv->thread == NULL) {
                USB_LOG_ERR("No memory to alloc for mtp thread\r\n");
            }
#endif
            break;
        case USBD_EVENT_DEINIT:
#if defined(CONFIG_USBDEV_MTP_THREAD)
            if (priv->mq != NULL) {
                usb_osal_mq_delete(priv->mq);
                priv->mq = NULL;
            }
            if (priv->thread != NULL) {
                usb_osal_thread_delete(priv->thread);
                priv->thread = NULL;
            }
#endif
            mtp_reset_state(busid);
            priv->configured = false;
            break;
        case USBD_EVENT_RESET:
        case USBD_EVENT_DISCONNECTED:
            mtp_reset_state(busid);
            priv->configured = false;
            break;
        case USBD_EVENT_CONFIGURED:
            mtp_reset_state(busid);
            priv->configured = true;
            mtp_start_command_read(busid);
            break;
        default:
            break;
    }
}

static void mtp_send_event(uint8_t busid, uint16_t event_code, uint32_t param)
{
    struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];
    uint8_t *buf = g_mtp_event_buffer[busid];

    if (!priv->configured || !priv->session_open) {
        return;
    }

    mtp_write_container_header(buf, 16, MTP_CONTAINER_TYPE_EVENT, event_code, priv->transaction_id);
    buf[12] = (uint8_t)param;
    buf[13] = (uint8_t)(param >> 8);
    buf[14] = (uint8_t)(param >> 16);
    buf[15] = (uint8_t)(param >> 24);
    usbd_ep_start_write(busid, priv->int_ep, buf, 16);
}

struct usbd_interface *usbd_mtp_init_intf(struct usbd_interface *intf,
                                          const uint8_t out_ep,
                                          const uint8_t in_ep,
                                          const uint8_t int_ep)
{
    intf->class_interface_handler = mtp_class_interface_request_handler;
    intf->class_endpoint_handler = NULL;
    intf->vendor_handler = NULL;
    intf->notify_handler = mtp_notify_handler;

    for (uint8_t busid = 0; busid < CONFIG_USBDEV_MAX_BUS; busid++) {
        struct usbd_mtp_priv *priv = &g_usbd_mtp[busid];

        memset(priv, 0, sizeof(*priv));
        priv->out_ep = out_ep;
        priv->in_ep = in_ep;
        priv->int_ep = int_ep;
        priv->fd = -1;

        priv->ep_data[MTP_EP_OUT_IDX].ep_addr = out_ep;
        priv->ep_data[MTP_EP_OUT_IDX].ep_cb = mtp_bulk_out;
        priv->ep_data[MTP_EP_IN_IDX].ep_addr = in_ep;
        priv->ep_data[MTP_EP_IN_IDX].ep_cb = mtp_bulk_in;
        priv->ep_data[MTP_EP_INT_IDX].ep_addr = int_ep;
        priv->ep_data[MTP_EP_INT_IDX].ep_cb = mtp_int_in;

        usbd_add_endpoint(busid, &priv->ep_data[MTP_EP_OUT_IDX]);
        usbd_add_endpoint(busid, &priv->ep_data[MTP_EP_IN_IDX]);
        usbd_add_endpoint(busid, &priv->ep_data[MTP_EP_INT_IDX]);
    }

    return intf;
}

int usbd_mtp_notify_object_add(const char *path)
{
    (void)path;

    for (uint8_t busid = 0; busid < CONFIG_USBDEV_MAX_BUS; busid++) {
        g_usbd_mtp[busid].objects_valid = false;
        mtp_send_event(busid, MTP_EVENT_STORAGE_INFO_CHANGED, MTP_STORAGE_ID);
    }

    return 0;
}

int usbd_mtp_notify_object_remove(const char *path)
{
    (void)path;

    for (uint8_t busid = 0; busid < CONFIG_USBDEV_MAX_BUS; busid++) {
        g_usbd_mtp[busid].objects_valid = false;
        mtp_send_event(busid, MTP_EVENT_STORAGE_INFO_CHANGED, MTP_STORAGE_ID);
    }

    return 0;
}

__WEAK const char *usbd_mtp_fs_root_path(void)
{
    return "/";
}

__WEAK const char *usbd_mtp_fs_description(void)
{
    return CONFIG_USBDEV_MTP_MODEL;
}

__WEAK int usbd_mtp_mkdir(const char *path)
{
    (void)path;
    return -1;
}

__WEAK int usbd_mtp_rmdir(const char *path)
{
    (void)path;
    return -1;
}

__WEAK MTP_DIR *usbd_mtp_opendir(const char *name)
{
    (void)name;
    return NULL;
}

__WEAK int usbd_mtp_closedir(MTP_DIR *d)
{
    (void)d;
    return -1;
}

__WEAK struct mtp_dirent *usbd_mtp_readdir(MTP_DIR *d)
{
    (void)d;
    return NULL;
}

__WEAK int usbd_mtp_statfs(const char *path, struct mtp_statfs *buf)
{
    (void)path;
    (void)buf;
    return -1;
}

__WEAK int usbd_mtp_stat(const char *file, struct stat *buf)
{
    (void)file;
    (void)buf;
    return -1;
}

__WEAK int usbd_mtp_open(const char *path, uint8_t mode)
{
    (void)path;
    (void)mode;
    return -1;
}

__WEAK int usbd_mtp_close(int fd)
{
    (void)fd;
    return 0;
}

__WEAK int usbd_mtp_read(int fd, void *buf, size_t len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return -1;
}

__WEAK int usbd_mtp_write(int fd, const void *buf, size_t len)
{
    (void)fd;
    (void)buf;
    (void)len;
    return -1;
}

__WEAK int usbd_mtp_unlink(const char *path)
{
    (void)path;
    return -1;
}
