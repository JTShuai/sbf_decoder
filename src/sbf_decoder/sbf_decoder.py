# -*- coding: UTF-8 -*-
'''
@Author  ：Jiangtao Shuai
@Date    ：03/04/2023 12:05 PM
'''

import struct
from sbf_decoder.blocks import BLOCK_NUMBERS, BLOCK_NAMES, BODY_PARSERS
from gps_time import GPSTime
from datetime import timezone
import numpy as np
import sbf_decoder.body_parser as body_parser

HEADER_LEN = 8
name_paser_dict = dict(zip(BLOCK_NAMES, BODY_PARSERS))
num_name_dict = dict(zip(BLOCK_NUMBERS, BLOCK_NAMES))


def gpsTime2Utc(tow: int, wnc: int):
    """ GPS time to Utc Unix epochs in Milliseconds """
    gps_time_obj = GPSTime(week_number=wnc, time_of_week=tow / 1000)
    gps_time_obj = gps_time_obj.to_datetime().replace(tzinfo=timezone.utc)

    return gps_time_obj.strftime("%d/%m/%Y-%H:%M:%S"), gps_time_obj.timestamp() * 1000


def crc_ccitt(data, size, initial_value):
    """CRC check"""
    crc = initial_value
    for i in range(size):
        crc ^= (data[i] << 8)
        for j in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def sbfDecoder(msg: bytes, is_online=True):
    """
    decode sbf message,
    For sbf log file, set is_online=False, otherwise it's too deep for recursion

    yield blockname, block_dict

    """
    if type(msg) is not bytearray:
        msg = bytearray(msg)

    if len(msg) <= 8:
        return

    recursive_flag = False

    header = msg[:8]

    # sync, crc, id, length
    header_fields = struct.unpack('<HHHH', header)  # (16420, 54007, 4050, 72)
    if header_fields[0] == 16420:
        if header_fields[-1] > HEADER_LEN and header_fields[-1] % 4 == 0:
            body_len = header_fields[-1] - 8

            crc = header_fields[1]
            id = header_fields[2]

            header_msg_len = header_fields[-1]

            if is_online:
                if len(msg) > header_msg_len:
                    # receive multiple blocks in one tcp/ip package
                    # do recursion in later
                    recursive_flag = True
                    sub_msg = msg[header_msg_len:]

                elif len(msg) < header_msg_len:
                    return

            msg_body = msg[8:header_msg_len]

            # check crc
            crc_init = crc_ccitt(header[4:8], 4, 0)
            body_crc = crc_ccitt(msg_body, body_len, crc_init)

            if crc == body_crc:
                blockno = id & 0x1fff
                blockname = num_name_dict.get(blockno, False)

                if not blockname:
                    # unknown block number
                    del msg[:header_msg_len]
                    return

                parser_name = name_paser_dict.get(blockname)
                blockPaser = getattr(body_parser, parser_name)

                block_dict = blockPaser(msg_body)

                # convert WNc + TOW to utc and unix epoch in milliseconds
                if block_dict:
                    block_dict['utc'], block_dict['ts'] = gpsTime2Utc(tow=block_dict['TOW'], wnc=block_dict['WNc'])

                    yield blockname, block_dict

                # only for real-time, one package has multi-blocks
                if recursive_flag:
                    for blockname, block_dict in sbfDecoder(sub_msg):
                        yield blockname, block_dict


if __name__ == '__main__':
    pass