# CherryUSB MTP RAM Device Example

This example exposes a RAM-backed file system through the CherryUSB MTP device class. It is useful for validating MTP enumeration, browsing, upload, download, directory creation, and deletion without an SD card.

## Features

- Enumerates as a Still Image / MTP USB interface.
- Allows the PC to browse files and directories in RAM.
- Supports download, upload, directory creation, and file/directory deletion.
- Creates `readme.txt` and an `upload` directory at startup.
- Defaults to 16 objects, 8 KB per file, and volatile contents.

## Build

From the SDK root:

```sh
source export.sh
```

Then build the project:

```sh
cd example/cherryusb/device/mtp_ramdisk/project
scons --board=sf32lb52-lcd_n16r8 -j8
```

## Test

After connecting USB, check the device with Windows Explorer, Linux `mtp-detect`, or `lsusb -v`. The host should show a `RAM Disk` storage containing `readme.txt`.
