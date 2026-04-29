/*
 * Copyright (c) 2026, SiFli Technologies(Nanjing) Co, Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "rtthread.h"
#include "rtdevice.h"
#include "dfs_fs.h"
#include "dfs_posix.h"
#include "usbd_core.h"
#include "usbd_mtp.h"

#define MTP_ROOT_PATH "/mtp"
#define MTP_BLOCK_DEV "sd0"

#define WCID_VENDOR_CODE 0x01

#define MTP_IN_EP  0x81
#define MTP_OUT_EP 0x02
#define MTP_INT_EP 0x83

#define USBD_VID           0x38f4
#define USBD_PID           0xffff
#define USBD_MAX_POWER     100
#define USB_CONFIG_SIZE    (9 + MTP_DESCRIPTOR_LEN)

#ifdef CONFIG_USB_HS
#define MTP_BULK_MPS USB_BULK_EP_MPS_HS
#else
#define MTP_BULK_MPS USB_BULK_EP_MPS_FS
#endif

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

static const uint8_t *WINUSB_IFx_WCIDProperties[] = {
    WINUSB_IF0_WCIDProperties,
};

static const struct usb_msosv1_descriptor msosv1_desc = {
    .string = WCID_StringDescriptor_MSOS,
    .vendor_code = WCID_VENDOR_CODE,
    .compat_id = WINUSB_WCIDDescriptor,
    .comp_id_property = WINUSB_IFx_WCIDProperties,
};

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

static int mtp_wait_sdcard(void)
{
    for (uint16_t i = 0; i < 200; i++) {
        if (rt_device_find(MTP_BLOCK_DEV) != RT_NULL) {
            return 0;
        }
        rt_thread_mdelay(20);
    }

    return -1;
}

static void mtp_create_readme(void)
{
    static const char readme[] =
        "SiFli CherryUSB MTP SD card demo\r\n"
        "\r\n"
        "This directory is exposed to the host through MTP.\r\n";
    int fd;

    fd = open(MTP_ROOT_PATH "/readme.txt", O_WRONLY | O_CREAT | O_TRUNC, 0);
    if (fd >= 0) {
        (void)write(fd, readme, sizeof(readme) - 1U);
        (void)close(fd);
    }
}

static int mtp_storage_init(void)
{
    int ret;

    if (mtp_wait_sdcard() != 0) {
        rt_kprintf("MTP: sd0 is not ready.\n");
        return -1;
    }

    if (access(MTP_ROOT_PATH, 0) != 0) {
        (void)mkdir(MTP_ROOT_PATH, 0);
    }

    ret = dfs_mount(MTP_BLOCK_DEV, MTP_ROOT_PATH, "elm", 0, 0);
    if (ret != 0) {
        rt_kprintf("MTP: mount %s to %s failed, try mkfs.\n", MTP_BLOCK_DEV, MTP_ROOT_PATH);
        if (dfs_mkfs("elm", MTP_BLOCK_DEV) == 0) {
            ret = dfs_mount(MTP_BLOCK_DEV, MTP_ROOT_PATH, "elm", 0, 0);
        }
    }

    if (ret != 0) {
        rt_kprintf("MTP: mount %s to %s failed.\n", MTP_BLOCK_DEV, MTP_ROOT_PATH);
        return -1;
    }

    mtp_create_readme();
    rt_kprintf("MTP: storage mounted at %s.\n", MTP_ROOT_PATH);
    return 0;
}

static struct usbd_interface intf0;

void mtp_sdcard_init(uint8_t busid, uintptr_t reg_base)
{
    (void)mtp_storage_init();

    usbd_desc_register(busid, &mtp_descriptor);
    usbd_add_interface(busid, usbd_mtp_init_intf(&intf0, MTP_OUT_EP, MTP_IN_EP, MTP_INT_EP));
    usbd_initialize(busid, reg_base, usbd_event_handler);
}

const char *usbd_mtp_fs_root_path(void)
{
    return MTP_ROOT_PATH;
}

const char *usbd_mtp_fs_description(void)
{
    return "SD Card";
}

int usbd_mtp_mkdir(const char *path)
{
    return mkdir(path, 0);
}

int usbd_mtp_rmdir(const char *path)
{
    return rmdir(path);
}

MTP_DIR *usbd_mtp_opendir(const char *name)
{
    return (MTP_DIR *)opendir(name);
}

int usbd_mtp_closedir(MTP_DIR *d)
{
    return closedir((DIR *)d);
}

struct mtp_dirent *usbd_mtp_readdir(MTP_DIR *d)
{
    struct dirent *dirent;
    static struct mtp_dirent mtp_dirent;

    dirent = readdir((DIR *)d);
    if (dirent == NULL) {
        return NULL;
    }

    memset(&mtp_dirent, 0, sizeof(mtp_dirent));
    mtp_dirent.d_type = dirent->d_type;
    mtp_dirent.d_namlen = dirent->d_namlen;
    mtp_dirent.d_reclen = dirent->d_reclen;
    strncpy(mtp_dirent.d_name, dirent->d_name, sizeof(mtp_dirent.d_name) - 1U);
    mtp_dirent.d_name[sizeof(mtp_dirent.d_name) - 1U] = '\0';

    return &mtp_dirent;
}

int usbd_mtp_statfs(const char *path, struct mtp_statfs *buf)
{
    struct statfs statfs_buf;

    if (statfs(path, &statfs_buf) != 0) {
        return -1;
    }

    buf->f_bsize = statfs_buf.f_bsize;
    buf->f_blocks = statfs_buf.f_blocks;
    buf->f_bfree = statfs_buf.f_bfree;
    return 0;
}

int usbd_mtp_stat(const char *file, struct stat *buf)
{
    return stat(file, buf);
}

int usbd_mtp_open(const char *path, uint8_t mode)
{
    int flags;

    if (mode == O_RDONLY) {
        flags = O_RDONLY;
    } else if (mode == O_WRONLY) {
        flags = O_WRONLY | O_CREAT | O_TRUNC;
    } else if (mode == O_RDWR) {
        flags = O_RDWR | O_CREAT;
    } else {
        return -1;
    }

    return open(path, flags, 0);
}

int usbd_mtp_close(int fd)
{
    return close(fd);
}

int usbd_mtp_read(int fd, void *buf, size_t len)
{
    return read(fd, buf, len);
}

int usbd_mtp_write(int fd, const void *buf, size_t len)
{
    return write(fd, buf, len);
}

int usbd_mtp_unlink(const char *path)
{
    return unlink(path);
}
