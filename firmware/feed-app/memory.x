MEMORY {
    BOOT2 : ORIGIN = 0x10000000, LENGTH = 0x100
    /* 2 MB flash; the last 4 KB sector holds the settings blob */
    FLASH : ORIGIN = 0x10000100, LENGTH = 2048K - 0x100 - 4K
    RAM   : ORIGIN = 0x20000000, LENGTH = 256K
}
