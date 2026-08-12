#!/usr/bin/env python3
'''
Just enough H.264 to read the picture size out of a sequence parameter set.

The boat changes resolution by sending a new parameter set mid-stream, which
downstream consumers generally cannot follow: an RTSP session negotiates the
parameter sets once, up front. Reading the dimensions lets a receiver notice
that the geometry it agreed to has changed, and lets it ignore parameter sets
left over from a stream that was already running at another resolution.
'''
from typing import Optional, Tuple


NAL_TYPE_SPS = 7
NAL_TYPE_PPS = 8

# Profiles that carry the chroma_format_idc block near the top of the SPS.
_EXTENDED_PROFILES = frozenset(
    (100, 110, 122, 244, 44, 83, 86, 118, 128, 138, 139, 134, 135))


class _BitReader:
    def __init__(self, data: bytes) -> None:
        self.data = data
        self.pos = 0  # in bits

    def bit(self) -> int:
        index, offset = divmod(self.pos, 8)
        if index >= len(self.data):
            raise ValueError('Truncated SPS')
        self.pos += 1
        return (self.data[index] >> (7 - offset)) & 1

    def bits(self, count: int) -> int:
        value = 0
        for _ in range(count):
            value = (value << 1) | self.bit()
        return value

    def ue(self) -> int:
        '''Unsigned exponential Golomb code.'''
        leading = 0
        while self.bit() == 0:
            leading += 1
            if leading > 32:
                raise ValueError('Malformed exponential Golomb code')
        return (1 << leading) - 1 + (self.bits(leading) if leading else 0)

    def se(self) -> int:
        '''Signed exponential Golomb code.'''
        value = self.ue()
        return (value + 1) // 2 if value % 2 else -(value // 2)


def strip_start_code(nal: bytes) -> bytes:
    if nal.startswith(b'\x00\x00\x00\x01'):
        return nal[4:]
    if nal.startswith(b'\x00\x00\x01'):
        return nal[3:]
    return nal


def nal_type(nal: bytes) -> Optional[int]:
    payload = strip_start_code(nal)
    return payload[0] & 0x1F if payload else None


def unescape(data: bytes) -> bytes:
    '''Remove emulation prevention bytes, turning 00 00 03 back into 00 00.'''
    out = bytearray()
    zeros = 0
    for byte in data:
        if zeros == 2 and byte == 0x03:
            zeros = 0
            continue
        out.append(byte)
        zeros = zeros + 1 if byte == 0x00 else 0
    return bytes(out)


def parse_sps(nal: bytes) -> Tuple[int, int]:
    '''
    Return the (width, height) described by a sequence parameter set, which may
    be given with or without an Annex B start code.
    '''
    payload = strip_start_code(nal)
    if not payload or payload[0] & 0x1F != NAL_TYPE_SPS:
        raise ValueError('Not a sequence parameter set')

    rbsp = unescape(payload[1:])
    if len(rbsp) < 4:
        raise ValueError('Truncated SPS')

    profile_idc = rbsp[0]
    reader = _BitReader(rbsp[3:])  # skip profile, constraint flags, level

    reader.ue()  # seq_parameter_set_id

    chroma_format_idc = 1  # 4:2:0 unless the SPS says otherwise
    separate_colour_plane = 0
    if profile_idc in _EXTENDED_PROFILES:
        chroma_format_idc = reader.ue()
        if chroma_format_idc == 3:
            separate_colour_plane = reader.bit()
        reader.ue()  # bit_depth_luma_minus8
        reader.ue()  # bit_depth_chroma_minus8
        reader.bit()  # qpprime_y_zero_transform_bypass_flag
        if reader.bit():  # seq_scaling_matrix_present_flag
            count = 8 if chroma_format_idc != 3 else 12
            for i in range(count):
                if reader.bit():  # seq_scaling_list_present_flag
                    _skip_scaling_list(reader, 16 if i < 6 else 64)

    reader.ue()  # log2_max_frame_num_minus4

    pic_order_cnt_type = reader.ue()
    if pic_order_cnt_type == 0:
        reader.ue()  # log2_max_pic_order_cnt_lsb_minus4
    elif pic_order_cnt_type == 1:
        reader.bit()  # delta_pic_order_always_zero_flag
        reader.se()  # offset_for_non_ref_pic
        reader.se()  # offset_for_top_to_bottom_field
        for _ in range(reader.ue()):  # num_ref_frames_in_pic_order_cnt_cycle
            reader.se()

    reader.ue()  # max_num_ref_frames
    reader.bit()  # gaps_in_frame_num_value_allowed_flag

    width_in_mbs = reader.ue() + 1
    height_in_map_units = reader.ue() + 1

    frame_mbs_only = reader.bit()
    if not frame_mbs_only:
        reader.bit()  # mb_adaptive_frame_field_flag
    reader.bit()  # direct_8x8_inference_flag

    crop_left = crop_right = crop_top = crop_bottom = 0
    if reader.bit():  # frame_cropping_flag
        crop_left = reader.ue()
        crop_right = reader.ue()
        crop_top = reader.ue()
        crop_bottom = reader.ue()

    # Cropping is counted in chroma samples, except when the colour planes are
    # coded separately or the picture is monochrome.
    if chroma_format_idc == 0 or separate_colour_plane:
        sub_width = sub_height = 1
    else:
        sub_width = 2 if chroma_format_idc in (1, 2) else 1
        sub_height = 2 if chroma_format_idc == 1 else 1

    width = width_in_mbs * 16 - sub_width * (crop_left + crop_right)
    height = ((2 - frame_mbs_only) * height_in_map_units * 16
              - sub_height * (2 - frame_mbs_only) * (crop_top + crop_bottom))
    return width, height


def _skip_scaling_list(reader: _BitReader, size: int) -> None:
    last = next_scale = 8
    for _ in range(size):
        if next_scale:
            next_scale = (last + reader.se() + 256) % 256
        last = next_scale or last
