import struct

# ── Sentinel values ────────────────────────────────────────────────────────────
# SMA and other vendors return these to indicate a register is unavailable
SENTINEL_U16 = 0xFFFF
SENTINEL_U32 = 0xFFFFFFFF


# ── Helpers ────────────────────────────────────────────────────────────────────

def to_s32(value: int) -> int:
    """Reinterpret an unsigned 32-bit integer as signed."""
    return value - 0x100000000 if value & 0x80000000 else value


def to_s16(value: int) -> int:
    """Reinterpret an unsigned 16-bit integer as signed."""
    return value - 0x10000 if value & 0x8000 else value


# ── Register decode ────────────────────────────────────────────────────────────

def decode_register(flat: list, hi_off: int, lo_off, scale: float, dtype: str):
    """
    Decode a single register value from a flat list of raw register words.

    flat   : flat list of raw u16 words built from poll blocks
    hi_off : index into flat for the high (or only) word
    lo_off : index into flat for the low word, or None for 16-bit registers
    scale  : multiplier applied to the raw integer value
    dtype  : one of U16, S16, U32, S32, F32

    Returns a rounded float, or None if a sentinel value is detected.
    """
    hi = flat[hi_off]

    # ── 16-bit ────────────────────────────────────────────────────────────────
    if lo_off is None:
        if hi == SENTINEL_U16:
            return None
        if dtype == "S16":
            hi = to_s16(hi)
        return round(hi * scale, 2)

    # ── 32-bit ────────────────────────────────────────────────────────────────
    lo  = flat[lo_off]
    raw = (hi << 16) | lo

    if raw == SENTINEL_U32:
        return None

    # F32 — byte order is lo-word first, hi-word second (big-endian words, swapped)
    if dtype == "F32":
        raw_bytes = struct.pack('>HH', lo, hi)
        return round(struct.unpack('>f', raw_bytes)[0], 2)

    if dtype == "S32":
        raw = to_s32(raw)

    return round(raw * scale, 2)


# ── Write encode ───────────────────────────────────────────────────────────────

def encode_write_register(value: float, scale: float, dtype: str) -> int:
    """
    Encode a physical value into a raw u16 word ready to send over Modbus.

    For write registers, raw = value / scale (inverse of read direction).
    Returns an unsigned 16-bit word.

    Example: c_pac percent=75.0, scale=0.01 → raw=7500 → word=0x1D4C
    """
    raw = int(round(value / scale))

    if dtype == "S16":
        raw = max(-32768, min(32767, raw))   # clamp to S16 range
        return raw & 0xFFFF                  # send as unsigned word

    if dtype == "U16":
        raw = max(0, min(65535, raw))
        return raw

    raise ValueError(f"[ENCODER] Unsupported write dtype: {dtype}")
