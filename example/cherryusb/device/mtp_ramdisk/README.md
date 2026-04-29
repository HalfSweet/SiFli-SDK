# CherryUSB MTP RAM 设备示例

本示例使用 CherryUSB MTP device class 暴露一个内存文件系统，方便在没有 SD 卡的情况下验证 MTP 枚举、浏览、上传、下载、创建目录和删除。

## 功能

- USB 接口枚举为 Still Image / MTP。
- PC 端可浏览内存文件系统中的目录和文件。
- 支持下载、上传、创建目录、删除文件和目录。
- 启动时创建 `readme.txt` 和 `upload` 目录。
- 默认最多 16 个对象，单文件最大 8 KB，掉电后内容丢失。

## 编译

在 SDK 根目录执行：

```sh
source export.sh
```

进入工程目录编译：

```sh
cd example/cherryusb/device/mtp_ramdisk/project
scons --board=sf32lb52-lcd_n16r8 -j8
```

## 验证

连接 USB 后，可使用 Windows Explorer、Linux `mtp-detect` 或 `lsusb -v` 检查设备。主机端应能看到 `RAM Disk` storage 和 `readme.txt`。
