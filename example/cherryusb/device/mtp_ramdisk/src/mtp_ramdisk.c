/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "usbd_core.h"
#include "usbd_mtp.h"

#include <string.h>

#ifndef CONFIG_USBDEV_MTP_RAM_MAX_NODES
#define CONFIG_USBDEV_MTP_RAM_MAX_NODES 17
#endif

#ifndef CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE
#define CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE 8192
#endif

#ifndef CONFIG_USBDEV_MTP_RAM_MAX_FDS
#define CONFIG_USBDEV_MTP_RAM_MAX_FDS 4
#endif

#ifndef CONFIG_USBDEV_MTP_RAM_MAX_DIRS
#define CONFIG_USBDEV_MTP_RAM_MAX_DIRS 8
#endif

#ifndef CONFIG_USBDEV_MTP_RAM_NAME_LEN
#define CONFIG_USBDEV_MTP_RAM_NAME_LEN 64
#endif

#ifndef DT_REG
#define DT_REG 0x01
#endif

#ifndef DT_DIR
#define DT_DIR 0x02
#endif

#define MTP_ROOT_PATH "/ram"
#define WCID_VENDOR_CODE 0x01

#define MTP_IN_EP  0x81
#define MTP_OUT_EP 0x02
#define MTP_INT_EP 0x83

#define USBD_VID        0x38f4
#define USBD_PID        0xffff
#define USBD_MAX_POWER  100
#define USB_CONFIG_SIZE (9 + MTP_DESCRIPTOR_LEN)

#define MTP_RAM_ROOT_NODE 0U
#define MTP_RAM_INVALID   0xffffU

#define MTP_RAM_FD_READ  0x01
#define MTP_RAM_FD_WRITE 0x02

#ifdef CONFIG_USB_HS
#define MTP_BULK_MPS USB_BULK_EP_MPS_HS
#else
#define MTP_BULK_MPS USB_BULK_EP_MPS_FS
#endif

struct mtp_ram_node {
    bool used;
    bool is_dir;
    uint16_t parent;
    char name[CONFIG_USBDEV_MTP_RAM_NAME_LEN];
    uint32_t size;
    uint8_t data[CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE];
    time_t mtime;
};

struct mtp_ram_fd {
    bool used;
    uint8_t flags;
    uint16_t node;
    uint32_t offset;
};

struct mtp_ram_dir {
    bool used;
    uint16_t parent;
    uint16_t cursor;
    struct mtp_dirent entry;
};

static struct mtp_ram_node g_ram_nodes[CONFIG_USBDEV_MTP_RAM_MAX_NODES];
static struct mtp_ram_fd g_ram_fds[CONFIG_USBDEV_MTP_RAM_MAX_FDS];
static struct mtp_ram_dir g_ram_dirs[CONFIG_USBDEV_MTP_RAM_MAX_DIRS];

__ALIGN_BEGIN static const uint8_t WCID_StringDescriptor_MSOS[18] __ALIGN_END = {
    0x12,
    USB_DESCRIPTOR_TYPE_STRING,
    'M', 0x00, 'S', 0x00, 'F', 0x00, 'T', 0x00,
    '1', 0x00, '0', 0x00, '0', 0x00,
    WCID_VENDOR_CODE,
    0x00,
};

__ALIGN_BEGIN static const uint8_t WINUSB_WCIDDescriptor[40] __ALIGN_END = {
    0x28, 0x00, 0x00, 0x00,
    0x00, 0x01,
    0x04, 0x00,
    0x01,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,
    0x01,
    'M', 'T', 'P', 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

__ALIGN_BEGIN static const uint8_t WINUSB_IF0_WCIDProperties[142] __ALIGN_END = {
    0x8e, 0x00, 0x00, 0x00,
    0x00, 0x01,
    0x05, 0x00,
    0x01, 0x00,
    0x84, 0x00, 0x00, 0x00,
    0x01, 0x00, 0x00, 0x00,
    0x28, 0x00,
    'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00,
    'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00,
    't', 0x00, 'e', 0x00, 'r', 0x00, 'f', 0x00,
    'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00,
    'U', 0x00, 'I', 0x00, 'D', 0x00, 0x00, 0x00,
    0x4e, 0x00, 0x00, 0x00,
    '{', 0x00, '1', 0x00, 'D', 0x00, '4', 0x00,
    'B', 0x00, '2', 0x00, '3', 0x00, '6', 0x00,
    '5', 0x00, '-', 0x00, '4', 0x00, '7', 0x00,
    '4', 0x00, '9', 0x00, '-', 0x00, '4', 0x00,
    '8', 0x00, 'E', 0x00, 'A', 0x00, '-', 0x00,
    'B', 0x00, '3', 0x00, '8', 0x00, 'A', 0x00,
    '-', 0x00, '7', 0x00, 'C', 0x00, '6', 0x00,
    'F', 0x00, 'D', 0x00, 'D', 0x00, 'D', 0x00,
    'D', 0x00, '7', 0x00, 'E', 0x00, '2', 0x00,
    '6', 0x00, '}', 0x00, 0x00, 0x00,
};

static const uint8_t device_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, USBD_VID, USBD_PID, 0x0200, 0x01),
};

static const uint8_t config_descriptor[] = {
    USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_SIZE, 0x01, 0x01, USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),
    MTP_DESCRIPTOR_INIT(0x00, MTP_OUT_EP, MTP_IN_EP, MTP_INT_EP, MTP_BULK_MPS, 0x02),
};

static const uint8_t device_quality_descriptor[] = {
    USB_DEVICE_QUALIFIER_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, 0x01),
};

static const char *string_descriptors[] = {
    (const char[]){ 0x09, 0x04 },
    CONFIG_USBDEV_MTP_MANUFACTURER,
    CONFIG_USBDEV_MTP_MODEL,
    CONFIG_USBDEV_MTP_SERIAL_NUMBER,
};

static const uint8_t *WINUSB_IFx_WCIDProperties[] = {
    WINUSB_IF0_WCIDProperties,
};

static const struct usb_msosv1_descriptor msosv1_desc = {
    .string = WCID_StringDescriptor_MSOS,
    .vendor_code = WCID_VENDOR_CODE,
    .compat_id = WINUSB_WCIDDescriptor,
    .comp_id_property = WINUSB_IFx_WCIDProperties,
};

static const uint8_t *device_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_descriptor;
}

static const uint8_t *config_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return config_descriptor;
}

static const uint8_t *device_quality_descriptor_callback(uint8_t speed)
{
    (void)speed;
    return device_quality_descriptor;
}

static const char *string_descriptor_callback(uint8_t speed, uint8_t index)
{
    (void)speed;

    if (index >= sizeof(string_descriptors) / sizeof(string_descriptors[0])) {
        return NULL;
    }

    return string_descriptors[index];
}

static const struct usb_descriptor mtp_descriptor = {
    .device_descriptor_callback = device_descriptor_callback,
    .config_descriptor_callback = config_descriptor_callback,
    .device_quality_descriptor_callback = device_quality_descriptor_callback,
    .string_descriptor_callback = string_descriptor_callback,
    .msosv1_descriptor = &msosv1_desc,
};

static void usbd_event_handler(uint8_t busid, uint8_t event)
{
    (void)busid;
    (void)event;
}

static int mtp_ram_find_child_n(uint16_t parent, const char *name, size_t name_len)
{
    for (uint16_t i = 0; i < CONFIG_USBDEV_MTP_RAM_MAX_NODES; i++) {
        if (!g_ram_nodes[i].used || g_ram_nodes[i].parent != parent) {
            continue;
        }
        if (strlen(g_ram_nodes[i].name) == name_len &&
            memcmp(g_ram_nodes[i].name, name, name_len) == 0) {
            return (int)i;
        }
    }

    return -1;
}

static int mtp_ram_find_child(uint16_t parent, const char *name)
{
    return mtp_ram_find_child_n(parent, name, strlen(name));
}

static bool mtp_ram_name_is_valid(const char *name, size_t name_len)
{
    if (name_len == 0 || name_len >= CONFIG_USBDEV_MTP_RAM_NAME_LEN) {
        return false;
    }

    if ((name_len == 1 && name[0] == '.') ||
        (name_len == 2 && name[0] == '.' && name[1] == '.')) {
        return false;
    }

    for (size_t i = 0; i < name_len; i++) {
        if (name[i] == '/' || name[i] == '\\' || name[i] == ':') {
            return false;
        }
    }

    return true;
}

static int mtp_ram_resolve(const char *path)
{
    size_t root_len = strlen(MTP_ROOT_PATH);
    uint16_t node = MTP_RAM_ROOT_NODE;
    const char *p;

    if (path == NULL || strncmp(path, MTP_ROOT_PATH, root_len) != 0) {
        return -1;
    }

    if (path[root_len] == '\0') {
        return (int)MTP_RAM_ROOT_NODE;
    }

    if (path[root_len] != '/') {
        return -1;
    }

    p = path + root_len + 1U;
    if (*p == '\0') {
        return (int)MTP_RAM_ROOT_NODE;
    }

    while (*p != '\0') {
        const char *slash = strchr(p, '/');
        size_t name_len = slash ? (size_t)(slash - p) : strlen(p);
        int child;

        if (!mtp_ram_name_is_valid(p, name_len)) {
            return -1;
        }

        child = mtp_ram_find_child_n(node, p, name_len);
        if (child < 0) {
            return -1;
        }

        node = (uint16_t)child;
        if (slash == NULL) {
            return child;
        }

        p = slash + 1U;
        if (*p == '\0') {
            return child;
        }
    }

    return (int)node;
}

static int mtp_ram_split_parent(const char *path, uint16_t *parent, char *name, size_t name_size)
{
    size_t root_len = strlen(MTP_ROOT_PATH);
    uint16_t current = MTP_RAM_ROOT_NODE;
    const char *p;

    if (path == NULL || parent == NULL || name == NULL || name_size == 0 ||
        strncmp(path, MTP_ROOT_PATH, root_len) != 0 ||
        path[root_len] != '/') {
        return -1;
    }

    p = path + root_len + 1U;
    while (*p != '\0') {
        const char *slash = strchr(p, '/');
        size_t name_len = slash ? (size_t)(slash - p) : strlen(p);

        if (!mtp_ram_name_is_valid(p, name_len)) {
            return -1;
        }

        if (slash == NULL) {
            if (!g_ram_nodes[current].is_dir || name_len >= name_size) {
                return -1;
            }
            memcpy(name, p, name_len);
            name[name_len] = '\0';
            *parent = current;
            return 0;
        }

        int child = mtp_ram_find_child_n(current, p, name_len);
        if (child < 0 || !g_ram_nodes[child].is_dir) {
            return -1;
        }
        current = (uint16_t)child;
        p = slash + 1U;
    }

    return -1;
}

static int mtp_ram_alloc_node(uint16_t parent, const char *name, bool is_dir)
{
    if (mtp_ram_find_child(parent, name) >= 0) {
        return -1;
    }

    for (uint16_t i = 1; i < CONFIG_USBDEV_MTP_RAM_MAX_NODES; i++) {
        if (!g_ram_nodes[i].used) {
            memset(&g_ram_nodes[i], 0, sizeof(g_ram_nodes[i]));
            g_ram_nodes[i].used = true;
            g_ram_nodes[i].is_dir = is_dir;
            g_ram_nodes[i].parent = parent;
            strncpy(g_ram_nodes[i].name, name, sizeof(g_ram_nodes[i].name) - 1U);
            return (int)i;
        }
    }

    return -1;
}

static bool mtp_ram_dir_is_empty(uint16_t node)
{
    for (uint16_t i = 0; i < CONFIG_USBDEV_MTP_RAM_MAX_NODES; i++) {
        if (g_ram_nodes[i].used && g_ram_nodes[i].parent == node) {
            return false;
        }
    }

    return true;
}

static int mtp_ram_alloc_fd(uint16_t node, uint8_t flags)
{
    for (int i = 0; i < CONFIG_USBDEV_MTP_RAM_MAX_FDS; i++) {
        if (!g_ram_fds[i].used) {
            g_ram_fds[i].used = true;
            g_ram_fds[i].flags = flags;
            g_ram_fds[i].node = node;
            g_ram_fds[i].offset = 0;
            return i;
        }
    }

    return -1;
}

static void mtp_ramfs_init(void)
{
    static const char readme[] =
        "SiFli CherryUSB MTP RAM disk demo\r\n"
        "\r\n"
        "This storage is backed by RAM and is cleared after reset.\r\n";
    int readme_node;

    memset(g_ram_nodes, 0, sizeof(g_ram_nodes));
    memset(g_ram_fds, 0, sizeof(g_ram_fds));
    memset(g_ram_dirs, 0, sizeof(g_ram_dirs));

    g_ram_nodes[MTP_RAM_ROOT_NODE].used = true;
    g_ram_nodes[MTP_RAM_ROOT_NODE].is_dir = true;
    g_ram_nodes[MTP_RAM_ROOT_NODE].parent = MTP_RAM_INVALID;

    readme_node = mtp_ram_alloc_node(MTP_RAM_ROOT_NODE, "readme.txt", false);
    if (readme_node >= 0) {
        g_ram_nodes[readme_node].size = sizeof(readme) - 1U;
        memcpy(g_ram_nodes[readme_node].data, readme, sizeof(readme) - 1U);
    }

    (void)mtp_ram_alloc_node(MTP_RAM_ROOT_NODE, "upload", true);
}

static struct usbd_interface intf0;

void mtp_ramdisk_init(uint8_t busid, uintptr_t reg_base)
{
    mtp_ramfs_init();

    usbd_desc_register(busid, &mtp_descriptor);
    usbd_add_interface(busid, usbd_mtp_init_intf(&intf0, MTP_OUT_EP, MTP_IN_EP, MTP_INT_EP));
    usbd_initialize(busid, reg_base, usbd_event_handler);

    rt_kprintf("MTP: RAM disk ready, %u nodes, %u bytes per file.\n",
               CONFIG_USBDEV_MTP_RAM_MAX_NODES - 1U,
               CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE);
}

const char *usbd_mtp_fs_root_path(void)
{
    return MTP_ROOT_PATH;
}

const char *usbd_mtp_fs_description(void)
{
    return "RAM Disk";
}

int usbd_mtp_mkdir(const char *path)
{
    uint16_t parent;
    char name[CONFIG_USBDEV_MTP_RAM_NAME_LEN];

    if (mtp_ram_split_parent(path, &parent, name, sizeof(name)) != 0) {
        return -1;
    }

    return mtp_ram_alloc_node(parent, name, true) >= 0 ? 0 : -1;
}

int usbd_mtp_rmdir(const char *path)
{
    int node = mtp_ram_resolve(path);

    if (node <= 0 || !g_ram_nodes[node].is_dir || !mtp_ram_dir_is_empty((uint16_t)node)) {
        return -1;
    }

    memset(&g_ram_nodes[node], 0, sizeof(g_ram_nodes[node]));
    return 0;
}

MTP_DIR *usbd_mtp_opendir(const char *name)
{
    int node = mtp_ram_resolve(name);

    if (node < 0 || !g_ram_nodes[node].is_dir) {
        return NULL;
    }

    for (uint16_t i = 0; i < CONFIG_USBDEV_MTP_RAM_MAX_DIRS; i++) {
        if (!g_ram_dirs[i].used) {
            memset(&g_ram_dirs[i], 0, sizeof(g_ram_dirs[i]));
            g_ram_dirs[i].used = true;
            g_ram_dirs[i].parent = (uint16_t)node;
            return (MTP_DIR *)&g_ram_dirs[i];
        }
    }

    return NULL;
}

int usbd_mtp_closedir(MTP_DIR *d)
{
    struct mtp_ram_dir *dir = (struct mtp_ram_dir *)d;

    if (dir == NULL || !dir->used) {
        return -1;
    }

    dir->used = false;
    return 0;
}

struct mtp_dirent *usbd_mtp_readdir(MTP_DIR *d)
{
    struct mtp_ram_dir *dir = (struct mtp_ram_dir *)d;

    if (dir == NULL || !dir->used) {
        return NULL;
    }

    while (dir->cursor < CONFIG_USBDEV_MTP_RAM_MAX_NODES) {
        uint16_t node = dir->cursor++;

        if (!g_ram_nodes[node].used || g_ram_nodes[node].parent != dir->parent) {
            continue;
        }

        memset(&dir->entry, 0, sizeof(dir->entry));
        dir->entry.d_type = g_ram_nodes[node].is_dir ? DT_DIR : DT_REG;
        dir->entry.d_namlen = (uint8_t)strlen(g_ram_nodes[node].name);
        dir->entry.d_reclen = sizeof(dir->entry);
        strncpy(dir->entry.d_name, g_ram_nodes[node].name, sizeof(dir->entry.d_name) - 1U);
        return &dir->entry;
    }

    return NULL;
}

int usbd_mtp_statfs(const char *path, struct mtp_statfs *buf)
{
    uint32_t free_bytes = 0;
    uint32_t total_bytes = (CONFIG_USBDEV_MTP_RAM_MAX_NODES - 1U) * CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE;

    if (buf == NULL || mtp_ram_resolve(path) < 0) {
        return -1;
    }

    for (uint16_t i = 1; i < CONFIG_USBDEV_MTP_RAM_MAX_NODES; i++) {
        if (!g_ram_nodes[i].used) {
            free_bytes += CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE;
        } else if (!g_ram_nodes[i].is_dir) {
            free_bytes += CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE - g_ram_nodes[i].size;
        }
    }

    buf->f_bsize = 512;
    buf->f_blocks = total_bytes / buf->f_bsize;
    buf->f_bfree = free_bytes / buf->f_bsize;
    return 0;
}

int usbd_mtp_stat(const char *file, struct stat *buf)
{
    int node = mtp_ram_resolve(file);

    if (node < 0 || buf == NULL) {
        return -1;
    }

    memset(buf, 0, sizeof(*buf));
    buf->st_mode = g_ram_nodes[node].is_dir ? (S_IFDIR | 0777) : (S_IFREG | 0666);
    buf->st_size = g_ram_nodes[node].is_dir ? 0U : g_ram_nodes[node].size;
    buf->st_atime = g_ram_nodes[node].mtime;
    buf->st_mtime = g_ram_nodes[node].mtime;
    buf->st_ctime = g_ram_nodes[node].mtime;
    return 0;
}

int usbd_mtp_open(const char *path, uint8_t mode)
{
    int node = mtp_ram_resolve(path);
    uint8_t flags;

    if (mode == O_RDONLY) {
        if (node < 0 || g_ram_nodes[node].is_dir) {
            return -1;
        }
        flags = MTP_RAM_FD_READ;
    } else if (mode == O_WRONLY) {
        if (node < 0) {
            uint16_t parent;
            char name[CONFIG_USBDEV_MTP_RAM_NAME_LEN];

            if (mtp_ram_split_parent(path, &parent, name, sizeof(name)) != 0) {
                return -1;
            }
            node = mtp_ram_alloc_node(parent, name, false);
        }
        if (node < 0 || g_ram_nodes[node].is_dir) {
            return -1;
        }
        g_ram_nodes[node].size = 0;
        flags = MTP_RAM_FD_WRITE;
    } else if (mode == O_RDWR) {
        if (node < 0) {
            uint16_t parent;
            char name[CONFIG_USBDEV_MTP_RAM_NAME_LEN];

            if (mtp_ram_split_parent(path, &parent, name, sizeof(name)) != 0) {
                return -1;
            }
            node = mtp_ram_alloc_node(parent, name, false);
        }
        if (node < 0 || g_ram_nodes[node].is_dir) {
            return -1;
        }
        flags = MTP_RAM_FD_READ | MTP_RAM_FD_WRITE;
    } else {
        return -1;
    }

    return mtp_ram_alloc_fd((uint16_t)node, flags);
}

int usbd_mtp_close(int fd)
{
    if (fd < 0 || fd >= CONFIG_USBDEV_MTP_RAM_MAX_FDS || !g_ram_fds[fd].used) {
        return -1;
    }

    g_ram_fds[fd].used = false;
    return 0;
}

int usbd_mtp_read(int fd, void *buf, size_t len)
{
    struct mtp_ram_fd *ram_fd;
    struct mtp_ram_node *node;
    uint32_t remain;
    uint32_t read_len;

    if (fd < 0 || fd >= CONFIG_USBDEV_MTP_RAM_MAX_FDS || buf == NULL || !g_ram_fds[fd].used) {
        return -1;
    }

    ram_fd = &g_ram_fds[fd];
    if ((ram_fd->flags & MTP_RAM_FD_READ) == 0) {
        return -1;
    }

    node = &g_ram_nodes[ram_fd->node];
    remain = node->size - ram_fd->offset;
    read_len = (len < remain) ? (uint32_t)len : remain;
    if (read_len > 0U) {
        memcpy(buf, &node->data[ram_fd->offset], read_len);
        ram_fd->offset += read_len;
    }

    return (int)read_len;
}

int usbd_mtp_write(int fd, const void *buf, size_t len)
{
    struct mtp_ram_fd *ram_fd;
    struct mtp_ram_node *node;

    if (fd < 0 || fd >= CONFIG_USBDEV_MTP_RAM_MAX_FDS || buf == NULL || !g_ram_fds[fd].used) {
        return -1;
    }

    ram_fd = &g_ram_fds[fd];
    if ((ram_fd->flags & MTP_RAM_FD_WRITE) == 0) {
        return -1;
    }

    if (len > CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE ||
        ram_fd->offset > CONFIG_USBDEV_MTP_RAM_MAX_FILE_SIZE - len) {
        return -1;
    }

    node = &g_ram_nodes[ram_fd->node];
    memcpy(&node->data[ram_fd->offset], buf, len);
    ram_fd->offset += (uint32_t)len;
    if (node->size < ram_fd->offset) {
        node->size = ram_fd->offset;
    }

    return (int)len;
}

int usbd_mtp_unlink(const char *path)
{
    int node = mtp_ram_resolve(path);

    if (node <= 0 || g_ram_nodes[node].is_dir) {
        return -1;
    }

    memset(&g_ram_nodes[node], 0, sizeof(g_ram_nodes[node]));
    return 0;
}
