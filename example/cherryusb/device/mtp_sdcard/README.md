# CherryUSB MTP SD 卡设备示例

本示例使用 CherryUSB MTP device class 将 SD 卡挂载到 RT-Thread DFS/ElmFat，并只通过 MTP 暴露 `/mtp` 目录。

## 功能

- USB 接口枚举为 Still Image / MTP。
- PC 端可浏览 `/mtp` 下的目录和文件。
- 支持下载、上传、创建目录、删除文件和目录。
- 启动时等待 `sd0`，挂载到 `/mtp`，并创建 `/mtp/readme.txt`。

## 编译

在 SDK 根目录执行：

```sh
source export.sh
```

进入工程目录编译：

```sh
cd example/cherryusb/device/mtp_sdcard/project
scons --board=sf32lb52-lcd_n16r8 -j8
```

## 验证

连接 USB 后，可使用 Windows Explorer、Linux `mtp-detect` 或 `lsusb -v` 检查设备。主机端应能看到 `SD Card` storage 和 `readme.txt`。
