# bus_broker/core/encoder.py

from .frames import CANFrame, Protocol, CAN_FD_DLC_MAP


class CANEncoder:
    """
    Converts a CANFrame into a raw bit stream exactly as it would
    appear on the physical bus wire.

    Bit encoding follows the CAN 2.0 specification:
      - dominant  = 0
      - recessive = 1

    Frame structure (standard, non-FD):
      SOF | Arbitration ID | RTR | IDE | r0 | DLC | Data | CRC | CRC_DEL | ACK | ACK_DEL | EOF

    Frame structure (extended):
      SOF | Base ID | SRR | IDE | Extended ID | RTR | r1 | r0 | DLC | Data | CRC | CRC_DEL | ACK | ACK_DEL | EOF

    Bit stuffing is applied to everything from SOF up to and including
    the CRC field. After 5 consecutive identical bits, one opposite bit
    is inserted.
    """

    EOF_BITS          = 7    # End of frame recessive bits
    INTERFRAME_BITS   = 3    # Minimum interframe space

    def encode(self, frame: CANFrame) -> list[int]:
        """
        Main entry point.
        Returns a list of integers (0 or 1) representing the full frame
        on the wire including bit stuffing.
        """
        if frame.protocol == Protocol.CAN_FD:
            bits, _ = self._encode_fd(frame)
            return bits
        return self._encode_standard(frame)

    def encode_with_metadata(self, frame: CANFrame) -> tuple[list[int], int]:
        """
        Encode a frame and return (bits, brs_index).

        brs_index is the bit position (in the stuffed bit stream) where
        the CAN FD data-rate phase begins (right after the BRS bit).
        For classic CAN frames, brs_index is always 0.
        """
        if frame.protocol == Protocol.CAN_FD:
            return self._encode_fd(frame)
        return self._encode_standard(frame), 0

    # ── Standard CAN (2.0A and 2.0B) ─────────────────────────────────────────

    def _encode_standard(self, frame: CANFrame) -> list[int]:
        # We build the pre-stuffing bits first, then apply stuffing
        raw = []

        # 1. Start of Frame - single dominant bit
        raw.append(0)

        # 2. Arbitration field
        if frame.is_extended:
            # 2a. Extended frame (29-bit ID)
            # Base ID = upper 11 bits
            base_id = (frame.arbitration_id >> 18) & 0x7FF
            raw += self._int_to_bits(base_id, 11)
            # SRR - substitute remote request, always recessive
            raw.append(1)
            # IDE - recessive = extended frame
            raw.append(1)
            # Extended ID = lower 18 bits
            ext_id = frame.arbitration_id & 0x3FFFF
            raw += self._int_to_bits(ext_id, 18)
            # RTR
            raw.append(1 if frame.is_remote else 0)
        else:
            # 2b. Standard frame (11-bit ID)
            raw += self._int_to_bits(frame.arbitration_id, 11)
            # RTR
            raw.append(1 if frame.is_remote else 0)
            # IDE - dominant = standard frame
            raw.append(0)

        # 3. Control field
        # r0 - reserved, dominant
        raw.append(0)
        if frame.is_extended:
            # r1 - reserved, dominant (extended frames have two reserved bits)
            raw.append(0)
        # DLC - 4 bits
        raw += self._int_to_bits(frame.dlc, 4)

        # 4. Data field (empty for remote frames)
        for byte in frame.data:
            raw += self._int_to_bits(byte, 8)

        # 5. CRC - calculated over everything from SOF to end of data
        crc = self._calculate_crc15(raw)
        raw += self._int_to_bits(crc, 15)

        # Apply bit stuffing to everything up to end of CRC
        stuffed = self._apply_bit_stuffing(raw)

        # 6. CRC delimiter - recessive, NOT stuffed
        stuffed.append(1)

        # 7. ACK field - transmitter sends recessive slot
        # (a receiver would pull this dominant but we are the bus)
        stuffed.append(0)   # ACK slot - dominant (bus acknowledges)
        stuffed.append(1)   # ACK delimiter - recessive

        # 8. End of Frame - 7 recessive bits, NOT stuffed
        stuffed += [1] * self.EOF_BITS

        # 9. Interframe space
        stuffed += [1] * self.INTERFRAME_BITS

        return stuffed

    # ── CAN FD ────────────────────────────────────────────────────────────────

    def _encode_fd(self, frame: CANFrame) -> tuple[list[int], int]:
        """
        CAN FD follows ISO 11898-1:2015.
        Key differences from classic CAN:
          - FDF bit distinguishes from classic CAN
          - BRS bit switches to faster data bit rate
          - ESI bit signals error state
          - Longer CRC (17 or 21 bit depending on payload)
          - Different stuffing in CRC field

        Returns (stuffed_bits, brs_index) where brs_index is the
        bit position in the stuffed stream where the data-rate phase
        begins. If BRS=0 (no switching), brs_index is 0.
        """
        raw = []

        # 1. Start of Frame
        raw.append(0)

        # 2. Arbitration field - same as classic CAN
        if frame.is_extended:
            base_id = (frame.arbitration_id >> 18) & 0x7FF
            raw += self._int_to_bits(base_id, 11)
            raw.append(1)   # SRR
            raw.append(1)   # IDE
            ext_id = frame.arbitration_id & 0x3FFFF
            raw += self._int_to_bits(ext_id, 18)
            raw.append(0)   # RTR - always dominant in FD
        else:
            raw += self._int_to_bits(frame.arbitration_id, 11)
            raw.append(0)   # RTR - always dominant in FD
            raw.append(0)   # IDE

        # 3. Control field for FD
        raw.append(0)   # r0 reserved
        raw.append(1)   # FDF (FD Frame) - recessive, distinguishes from classic
        raw.append(0)   # res - reserved

        # BRS bit - recessive(1) = switch to fast rate, dominant(0) = no switch
        brs_bit = 1 if frame.brs else 0
        # Track position BEFORE stuffing for BRS
        brs_raw_index = len(raw)
        raw.append(brs_bit)

        raw.append(0)   # ESI - Error State Indicator (dominant = error active)
        raw += self._int_to_bits(frame.dlc, 4)

        # 4. Data field
        for byte in frame.data:
            raw += self._int_to_bits(byte, 8)

        # 5. CRC - FD uses 17-bit CRC for <=16 bytes, 21-bit for >16 bytes
        data_bytes = CAN_FD_DLC_MAP[frame.dlc]
        if data_bytes <= 16:
            crc = self._calculate_crc17(raw)
            raw += self._int_to_bits(crc, 17)
        else:
            crc = self._calculate_crc21(raw)
            raw += self._int_to_bits(crc, 21)

        # Apply bit stuffing
        stuffed = self._apply_bit_stuffing(raw)

        # Calculate BRS index in the stuffed stream.
        # The BRS bit itself is at brs_raw_index in the raw stream.
        # In the stuffed stream, the data-rate phase begins right AFTER
        # the BRS bit. We find the stuffed position of brs_raw_index + 1.
        brs_index = 0
        if frame.brs:
            brs_index = self._raw_to_stuffed_index(raw, stuffed, brs_raw_index + 1)

        # 6. CRC delimiter
        stuffed.append(1)

        # 7. ACK
        stuffed.append(0)
        stuffed.append(1)

        # 8. EOF
        stuffed += [1] * self.EOF_BITS

        # 9. Interframe space
        stuffed += [1] * self.INTERFRAME_BITS

        return stuffed, brs_index

    # ── Bit stuffing ──────────────────────────────────────────────────────────

    def _apply_bit_stuffing(self, bits: list[int]) -> list[int]:
        """
        After 5 consecutive identical bits, insert one opposite bit.
        This is applied from SOF through end of CRC field.
        The stuff bit itself does not count toward the next group of 5.
        """
        result        = []
        consecutive   = 1
        last_bit      = bits[0]
        result.append(bits[0])

        for bit in bits[1:]:
            if bit == last_bit:
                consecutive += 1
                if consecutive == 5:
                    result.append(bit)
                    # Insert stuff bit
                    stuff_bit = 1 - bit
                    result.append(stuff_bit)
                    last_bit    = stuff_bit
                    consecutive = 1
                    continue
            else:
                consecutive = 1

            result.append(bit)
            last_bit = bit

        return result

    # ── CRC calculations ──────────────────────────────────────────────────────

    def _calculate_crc15(self, bits: list[int]) -> int:
        """
        CAN CRC-15.
        Generator polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
        = 0x4599
        Calculated over all bits from SOF to end of data field.
        """
        crc = 0
        polynomial = 0x4599

        for bit in bits:
            crc_msb = (crc >> 14) & 1
            crc = ((crc << 1) & 0x7FFF) | bit
            if crc_msb:
                crc ^= polynomial

        return crc

    def _calculate_crc17(self, bits: list[int]) -> int:
        """
        CAN FD CRC-17.
        Generator polynomial: 0x3685B
        Used for CAN FD frames with 0-16 data bytes.
        """
        crc = 0
        polynomial = 0x3685B

        for bit in bits:
            crc_msb = (crc >> 16) & 1
            crc = ((crc << 1) & 0x1FFFF) | bit
            if crc_msb:
                crc ^= polynomial

        return crc

    def _calculate_crc21(self, bits: list[int]) -> int:
        """
        CAN FD CRC-21.
        Generator polynomial: 0x302899
        Used for CAN FD frames with 20-64 data bytes.
        """
        crc = 0
        polynomial = 0x302899

        for bit in bits:
            crc_msb = (crc >> 20) & 1
            crc = ((crc << 1) & 0x1FFFFF) | bit
            if crc_msb:
                crc ^= polynomial

        return crc

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _int_to_bits(self, value: int, length: int) -> list[int]:
        """Convert an integer to a list of bits, MSB first."""
        return [(value >> (length - 1 - i)) & 1 for i in range(length)]

    def _raw_to_stuffed_index(
        self,
        raw: list[int],
        stuffed: list[int],
        raw_index: int
    ) -> int:
        """
        Map a raw (pre-stuffing) bit index to its position in the
        stuffed bit stream.

        Replays the bit stuffing logic, counting how many stuff bits
        have been inserted before the target position.
        """
        stuffed_pos = 0
        consecutive = 1
        last_bit    = raw[0]
        stuffed_pos += 1

        for i in range(1, min(raw_index, len(raw))):
            bit = raw[i]
            if bit == last_bit:
                consecutive += 1
                if consecutive == 5:
                    stuffed_pos += 1    # the raw bit itself
                    stuffed_pos += 1    # the inserted stuff bit
                    last_bit = 1 - bit  # stuff bit value
                    consecutive = 1
                    continue
            else:
                consecutive = 1

            stuffed_pos += 1
            last_bit = bit

        return stuffed_pos
