MTP Device
=================

MTP demo 参考 `demo/mtp_template.c` 模板。 默认适配 fatfs 文件系统(`platform/fatfs/usbd_fatfs_mtp.c`)。

设备类通过 `usbd_mtp_fs_*` 文件系统适配接口支持枚举、storage/object 查询、文件下载、文件上传、文件和目录删除。
