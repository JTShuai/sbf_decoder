# -*- coding: UTF-8 -*-
'''
@Author  ：Jiangtao Shuai
@Date    ：03/04/2023 12:05 PM 
'''


import struct
import numpy as np


# INSNavCart datum
Datum = (
    (0, 'WGS84/ITRS'),
    (19, 'DGNSS/RTK base station'),
    (30, 'ETRS89 (ETRF2000 realization)'),
    (31, 'NAD83(2011)'),
    (32, 'NAD83(PA11)'),
    (33, 'NAD83(MA11)'),
    (34, 'GDA94(2010)'),
    (35, 'GDA2020'),
    (36, 'JGD2011'),
    (250, 'First user-defined datum'),
    (251, 'Second user-defined datum'),
)

datum_dict = dict(Datum)

# INSNavCart sub-blocks LSB
INS_SB = (
    (0, 'PositionStdDev'),
    (1, 'Attitude'),
    (2, 'AttitudeStdDev'),
    (3, 'Velocity'),
    (4, 'VelocityStdDev'),
    (5, 'PositionCov'),
    (6, 'AttitudeCov'),
    (7, 'VelocityCov'),
)

ins_sb_dict = dict(INS_SB)


def extSensorMeasParser(msg_body: bytes):
    """
    parse ExtSensorMeas message body

    @return: parsed sbf message body data
    @rtype: dict
    """

    ext_sensor_meas = {}

    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    ext_sensor_meas['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2
    ext_sensor_meas['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # N: u1; number of sub-blocks
    ext_sensor_meas['N'] = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]

    # SBLength: u1; length of sub-block
    sub_len = struct.unpack('<B', body_array[:1])[0]

    ext_sensor_meas['SBLength'] = sub_len
    del body_array[:1]

    # sub_blocks
    sb_list = []
    ext_sensor_meas['sub-blocks'] = sb_list
    for _ in range(ext_sensor_meas['N']):

        # read sub-block
        sub_blocks_array = body_array[:sub_len]
        del body_array[:sub_len]

        sb_dict = {}

        # Source: u1
        sb_dict['Source'] = struct.unpack('<B', sub_blocks_array[:1])[0]
        del sub_blocks_array[:1]

        # SensorModel: u1
        sb_dict['SensorModel'] = struct.unpack('<B', sub_blocks_array[:1])[0]
        del sub_blocks_array[:1]

        # Type: u1
        type_indicator = struct.unpack('<B', sub_blocks_array[:1])[0]
        sb_dict['Type'] = type_indicator
        del sub_blocks_array[:1]

        # ObsInfo: u1
        sb_dict['ObsInfo'] = struct.unpack('<B', sub_blocks_array[:1])[0]
        del sub_blocks_array[:1]

        # # Padding: u1, skip
        # del sub_blocks_array[:1]

        data_dict = {}
        sb_dict['data_dict'] = data_dict
        # switch depends on type number

        if type_indicator == 0:
            # acceleration
            accs = struct.unpack('<ddd', sub_blocks_array[:24])
            del sub_blocks_array[:24]

            data_dict['acc_x'] = accs[0]
            data_dict['acc_y'] = accs[1]
            data_dict['acc_z'] = accs[2]

        elif type_indicator == 1:
            # AngularRate
            angular_rates = struct.unpack('<ddd', sub_blocks_array[:24])
            del sub_blocks_array[:24]

            data_dict['angular_rate_x'] = angular_rates[0]
            data_dict['angular_rate_y'] = angular_rates[1]
            data_dict['angular_rate_z'] = angular_rates[2]

        elif type_indicator == 3:
            # info
            sensor_temperature = struct.unpack('<h', sub_blocks_array[:2])[0]
            # del info + reserved
            del sub_blocks_array[:3]

            data_dict['sensor_temperature'] = sensor_temperature

        elif type_indicator == 4:
            # Velocity
            vels = struct.unpack('<llllll', sub_blocks_array[:24])
            del sub_blocks_array[:24]

            data_dict['velocity_x'] = vels[0]
            data_dict['velocity_y'] = vels[1]
            data_dict['velocity_z'] = vels[2]

            data_dict['std_dev_x'] = vels[3]
            data_dict['std_dev_y'] = vels[4]
            data_dict['std_dev_z'] = vels[5]

        elif type_indicator == 20:
            # ZeroVelocityFlag
            flag = struct.unpack('<d', sub_blocks_array[:8])
            # del info + reserved
            del sub_blocks_array[:9]

            data_dict['flag'] = flag

        sb_list.append(sb_dict)

    return ext_sensor_meas


def insNavCartParser(msg_body: bytes):
    """
    parse INSNavCart message body

    @return: parsed sbf message body data
    @rtype: dict
    """

    ins_cart = {}

    # tow = struct.unpack('<HHHH', header)
    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    ins_cart['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2
    ins_cart['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # GNSSMode: u1; GNSS mode used in the INS solution
    ins_cart['GNSSMode'] = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]

    # Error: u1; Error code
    ins_cart['Error'] = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]

    # Info: u2;
    ins_cart['Info'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # GNSSAge: u2; Duration that no GNSS measurements were received; Unit: 0.01 second
    ins_cart['GNSSAge'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # xyz frame specified by Datum, each one is f8; Unit: 1 Meter
    x = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]
    y = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]
    z = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]
    ins_cart['pos'] = (x, y, z)

    # Accuracy: u2; 3D 2-sigma position accuracy; Unit: 0.01 Meter
    ins_cart['Accuracy'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Latency: u2; receiver processing time + IMU processing time + IMU communication latency; Unit: 0.0001 second
    ins_cart['Latency'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Datum: u1
    datum_index = struct.unpack('<B', body_array[:1])[0]
    if datum_index in datum_dict.keys():
        ins_cart['Datum'] = datum_dict[datum_index]
    else:
        ins_cart['Datum'] = "UNKNOW COORDINATES"
    del body_array[:1]

    # delete reserved
    del body_array[:1]

    # SBList: u2; read bits from LSB
    # sb_list = []
    # ins_cart['SBList'] = sb_list
    value = int.from_bytes(body_array[:2], byteorder='little')
    del body_array[:2]

    # extract the first eight bits starting from the LSB
    bit_mask = 0b11111111
    first_eight_bits = value & bit_mask
    # bits string
    first_eight_bits = bin(first_eight_bits)[2:]

    if len(first_eight_bits) < 8:
        for _ in range(8 - len(first_eight_bits)):
            first_eight_bits = '0' + first_eight_bits

    # get sub-blocks, each sub-block is 12 bytes in f4,f4,f4
    cnt = 0
    for bit_char in reversed(first_eight_bits):

        sub_dict = insNavSubParser(bit_index=cnt, bit_char=bit_char, body_array=body_array)

        for k, v in sub_dict.items():
            ins_cart[k] = v

        cnt += 1

    return ins_cart


def insNavSubParser(bit_index: int, bit_char: str, body_array: bytearray):
    """
    parse the sub block in INSNavCart
    """

    sb_name = ins_sb_dict[bit_index]
    sb_dict = {}

    if int(bit_char) == 0:
        # the bit value is zero
        sb_dict[sb_name] = np.nan
        return sb_dict
    else:
        sb_dict[sb_name] = struct.unpack('<fff', body_array[:12])
        del body_array[:12]

    return sb_dict


def insNavGeodParser(msg_body: bytes):
    """
    parse INSNavGeod message body

    @return: parsed sbf message body data
    @rtype: dict
    """
    ins_nav_geo = {}
    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    ins_nav_geo['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2; unsigned integer 2 bytes
    ins_nav_geo['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # GNSSMode: u1; GNSS mode used in the INS solution; unsigned integer 1bytes
    ins_nav_geo['GNSSMode'] = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]

    # Error: u1; Error code; unsigned integer 1bytes
    ins_nav_geo['Error'] = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]

    # Info: u2;
    ins_nav_geo['Info'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # GNSSAge: u2; Duration that no GNSS measurements were received; Unit: 0.01 second
    ins_nav_geo['GNSSAge'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Latitude; f8; double 8 bytes
    ins_nav_geo['Latitude'] = np.degrees(struct.unpack('<d', body_array[:8])[0])
    del body_array[:8]

    # Longitude; f8
    ins_nav_geo['Longitude'] = np.degrees(struct.unpack('<d', body_array[:8])[0])
    del body_array[:8]

    # Height; f8
    ins_nav_geo['Height'] = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]

    # Undulation; f4; float 4 bytes
    ins_nav_geo['Undulation'] = struct.unpack('<f', body_array[:4])[0]
    del body_array[:4]

    # Accuracy; u2;
    ins_nav_geo['Accuracy'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Latency; u2;
    ins_nav_geo['Latency'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Datum: u1
    datum_index = struct.unpack('<B', body_array[:1])[0]
    if datum_index in datum_dict.keys():
        ins_nav_geo['Datum'] = datum_dict[datum_index]
    else:
        ins_nav_geo['Datum'] = "UNKNOW COORDINATES"
    del body_array[:1]

    # Reserved: u1
    del body_array[:1]

    # SBList: u2; read bits from LSB
    # sb_list = []
    # ins_nav_geo['SBList'] = sb_list
    value = int.from_bytes(body_array[:2], byteorder='little')
    del body_array[:2]

    # extract the first eight bits starting from the LSB
    bit_mask = 0b11111111
    first_eight_bits = value & bit_mask
    # bits string
    first_eight_bits = bin(first_eight_bits)[2:]

    if len(first_eight_bits) < 8:
        for _ in range(8 - len(first_eight_bits)):
            first_eight_bits = '0' + first_eight_bits

    # get sub-blocks, each sub-block is 12 bytes in f4,f4,f4
    cnt = 0
    for bit_char in reversed(first_eight_bits):

        sub_dict = insNavSubParser(bit_index=cnt, bit_char=bit_char, body_array=body_array)
        for k, v in sub_dict.items():
            ins_nav_geo[k] = v

        cnt += 1

    return ins_nav_geo

