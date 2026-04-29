# CherryUSB MTP SD Card Device Example

This example exposes an SD card through the CherryUSB MTP device class. The card is mounted with RT-Thread DFS/ElmFat and only the `/mtp` directory is visible to the MTP host.

## Features

- Enumerates as a Still Image / MTP USB interface.
- Allows the PC to browse files and directories under `/mtp`.
- Supports download, upload, directory creation, and file/directory deletion.
- Waits for `sd0`, mounts it at `/mtp`, and creates `/mtp/readme.txt` at startup.

## Build

From the SDK root:

```sh
source export.sh
```

Then build the project:

```sh
cd example/cherryusb/device/mtp_sdcard/project
scons --board=sf32lb52-lcd_n16r8 -j8
```

## Test

After connecting USB, check the device with Windows Explorer, Linux `mtp-detect`, or `lsusb -v`. The host should show an `SD Card` storage containing `readme.txt`.
