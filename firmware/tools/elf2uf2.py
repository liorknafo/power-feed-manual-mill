"""Convert the firmware ELF to a UF2 file for drag-and-drop flashing of the Pico (RP2040).

usage: python3 firmware/tools/elf2uf2.py <in.elf> <out.uf2>
Hold BOOTSEL while plugging in USB, then copy the .uf2 onto the RPI-RP2 drive.
"""
import struct
import sys

FAMILY_RP2040 = 0xE48BFF56
FLASH_START, FLASH_END = 0x10000000, 0x10000000 + 2 * 1024 * 1024


def load_segments(path):
    data = open(path, "rb").read()
    assert data[:4] == b"\x7fELF" and data[4] == 1, "need a 32-bit ELF"
    (phoff,) = struct.unpack_from("<I", data, 0x1C)
    phentsize, phnum = struct.unpack_from("<HH", data, 0x2A)
    segs = []
    for i in range(phnum):
        p_type, p_offset, p_vaddr, p_paddr, p_filesz = struct.unpack_from("<IIIII", data, phoff + i * phentsize)
        if p_type == 1 and p_filesz and FLASH_START <= p_paddr < FLASH_END:  # PT_LOAD into flash (LMA)
            segs.append((p_paddr, data[p_offset:p_offset + p_filesz]))
    return segs


def main(src, dst):
    image = {}
    for addr, blob in load_segments(src):
        for i, b in enumerate(blob):
            image[addr + i] = b
    pages = sorted({a & ~0xFF for a in image})
    blocks = []
    for n, page in enumerate(pages):
        payload = bytes(image.get(page + i, 0) for i in range(256))
        hdr = struct.pack("<IIIIIIII", 0x0A324655, 0x9E5D5157, 0x00002000, page, 256, n, len(pages), FAMILY_RP2040)
        blocks.append(hdr + payload + b"\x00" * (476 - 256) + struct.pack("<I", 0x0AB16F30))
    open(dst, "wb").write(b"".join(blocks))
    print("%s: %d blocks, %d bytes of flash" % (dst, len(blocks), len(pages) * 256))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
