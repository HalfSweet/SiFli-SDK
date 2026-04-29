MTP Device
=================

MTP demo references the `demo/mtp_template.c` template. Adapted for FatFS file system by default (`platform/fatfs/usbd_fatfs_mtp.c`).

The device class supports enumeration, storage/object queries, file download, file upload, and file/directory deletion through the `usbd_mtp_fs_*` filesystem adapter interface.
