#/bin/bash

dir="bin/$1"
lib_dir="bin"

esptool.py                                        \
    --chip esp32                                  \
    --port /dev/ttyUSB0                           \
    --baud 921600                                 \
    --before default_reset                        \
    --after hard_reset write_flash                \
    -z                                            \
    --flash_mode dio                              \
    --flash_freq 80m                              \
    --flash_size 4MB                              \
        0x1000  $dir/g-1000dxc.ino.bootloader.bin \
        0x8000  $dir/g-1000dxc.ino.partitions.bin \
        0xe000  $lib_dir/boot_app0.bin            \
        0x10000 $dir/g-1000dxc.ino.bin
