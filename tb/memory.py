"""
Simple byte-addressable memory model for MC68030 testbench.

Big-endian (68K byte order). Supports byte, word, and long accesses.
"""


class Memory:
    """Byte-addressable big-endian memory model."""

    def __init__(self, size=1 << 20):
        """Initialize memory with the given size in bytes (default 1 MB)."""
        self._mem = bytearray(size)
        self._size = size

    def load_binary(self, addr, data_bytes):
        """Load a block of raw bytes starting at addr.

        Args:
            addr: Starting byte address.
            data_bytes: bytes or bytearray to load.
        """
        length = len(data_bytes)
        if addr + length > self._size:
            raise ValueError(
                f"load_binary: addr 0x{addr:08X} + length {length} exceeds memory size 0x{self._size:X}"
            )
        self._mem[addr : addr + length] = data_bytes

    def load_words(self, addr, word_list):
        """Load a list of 16-bit words starting at addr (big-endian).

        Args:
            addr: Starting byte address (should be word-aligned).
            word_list: list of integers, each 0x0000..0xFFFF.
        """
        for i, word in enumerate(word_list):
            word &= 0xFFFF
            offset = addr + i * 2
            self._mem[offset] = (word >> 8) & 0xFF
            self._mem[offset + 1] = word & 0xFF

    def load_long(self, addr, value):
        """Load a 32-bit long word at addr (big-endian).

        Args:
            addr: Starting byte address (should be long-aligned).
            value: 32-bit integer.
        """
        value &= 0xFFFFFFFF
        self._mem[addr] = (value >> 24) & 0xFF
        self._mem[addr + 1] = (value >> 16) & 0xFF
        self._mem[addr + 2] = (value >> 8) & 0xFF
        self._mem[addr + 3] = value & 0xFF

    def read(self, addr, size):
        """Read 1, 2, or 4 bytes from addr and return a 32-bit value.

        Args:
            addr: Byte address.
            size: Number of bytes to read (1, 2, or 4).

        Returns:
            Integer value (zero-extended to 32 bits).
        """
        addr &= self._size - 1  # Wrap around
        if size == 1:
            return self._mem[addr]
        elif size == 2:
            return (self._mem[addr] << 8) | self._mem[addr + 1]
        elif size == 4:
            return (
                (self._mem[addr] << 24)
                | (self._mem[addr + 1] << 16)
                | (self._mem[addr + 2] << 8)
                | self._mem[addr + 3]
            )
        else:
            raise ValueError(f"read: invalid size {size}, must be 1, 2, or 4")

    def write(self, addr, size, value):
        """Write 1, 2, or 4 bytes to addr.

        Args:
            addr: Byte address.
            size: Number of bytes to write (1, 2, or 4).
            value: Integer value to write.
        """
        addr &= self._size - 1  # Wrap around
        value &= 0xFFFFFFFF
        if size == 1:
            self._mem[addr] = value & 0xFF
        elif size == 2:
            self._mem[addr] = (value >> 8) & 0xFF
            self._mem[addr + 1] = value & 0xFF
        elif size == 4:
            self._mem[addr] = (value >> 24) & 0xFF
            self._mem[addr + 1] = (value >> 16) & 0xFF
            self._mem[addr + 2] = (value >> 8) & 0xFF
            self._mem[addr + 3] = value & 0xFF
        else:
            raise ValueError(f"write: invalid size {size}, must be 1, 2, or 4")

    def dump(self, addr, length=16):
        """Return a hex dump string for debugging."""
        lines = []
        for offset in range(0, length, 16):
            a = addr + offset
            hexbytes = " ".join(f"{self._mem[a + i]:02X}" for i in range(min(16, length - offset)))
            lines.append(f"  0x{a:08X}: {hexbytes}")
        return "\n".join(lines)
