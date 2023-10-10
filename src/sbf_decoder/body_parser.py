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

DIFF_CORR_MODE = (
    (0, 'RTCMv2'),
    (1, 'CMRv2'),
    (2, 'RTCMv3'),
    (3, 'RTCMV'),
    (4, 'SPARTN'),
    (5, 'Reserved number'),
)
diff_corr_mode_dict = dict(DIFF_CORR_MODE)

DIFF_CORR_SOURCE = (
    (0, "COM1"),
    (1, "COM2"),
    (2, "COM3"),
    (3, "COM4"),
    (4, "USB1"),
    (5, "USB2"),
    (6, "IP connection"),
    (7, "SBF file"),
    (8, "L-Band"),
    (9, "NTRIP"),
    (10, "OTG1"),
    (11, "OTG2"),
    (12, "Bluetooth"),
    (15, "UHF modem"),
    (16, "IPR connection"),
    (17, "Direct callport"),
    (18, "IPS connection")
)
diff_corr_source_dict = dict(DIFF_CORR_SOURCE)

BASE_TYPE = (
    (0, "Fixed"),
    (1, "Moving"),
    (255, "Unknown"),
)
base_type_dict = dict(BASE_TYPE)

# Source of the base station coordinates:
BASE_SOURCE = (
    (0, "RTCM 2.x (Msg 3)"),
    (2, "RTCM 2.x (Msg 24)"),
    (4, "CMR 2.x (Msg 1)"),
    (8, "RTCM 3.x (Msg 1005 or 1006)"),
    (9, "RTCMV (Msg 3)"),
    (10, "CMR+ (Type 2)"),
)
base_source_dict = dict(BASE_SOURCE)


RTCM_HEIGHT_TYPE = (
    (0, "Geometrical height"),
    (1, "Physical height (height definition in target CRS)"),
    (2, "Physical height (height definition in source CRS)")
)
rtcm_height_dict = dict(RTCM_HEIGHT_TYPE)


RTCM_QUALITY_IND = (
    (0, "Unknown quality"),
    (1, "Quality better than 21 mm (from MT1021/1022)"),
    (2, "Quality 21 to 50 mm (from MT1021/1022)"),
    (3, "Quality 51 to 200 mm (from MT1021/1022)"),
    (4, "Quality 201 to 500 mm (from MT1021/1022)"),
    (5, "Quality 501 to 2000 mm (from MT1021/1022)"),
    (6, "Quality 2001 to 5000 mm (from MT1021/1022)"),
    (7, "Quality worse than 5001 mm (from MT1021/1022)"),
    (9, "Quality 0 to 10 mm (from MT1023/1024)"),
    (10, "Quality 11 to 20 mm (from MT1023/1024)"),
    (11, "Quality 21 to 50 mm (from MT1023/1024)"),
    (12, "Quality 51 to 100 mm (from MT1023/1024)"),
    (13, "Quality 101 to 200 mm (from MT1023/1024)"),
    (14, "Quality 201 to 500 mm (from MT1023/1024)"),
    (15, "Quality worse than 501 mm (from MT1023/1024)"),
)
rtcm_quality_dict = dict(RTCM_QUALITY_IND)

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


def diffCorrInParser(msg_body: bytes):
    """
    parse DiffCorrIn message body

    @return: parsed sbf message body data
    @rtype: dict
    """

    diff_corr = {}

    # tow = struct.unpack('<HHHH', header)
    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    diff_corr['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2
    diff_corr['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # Mode: u1; Differential correction mode
    mode_index = struct.unpack('<B', body_array[:1])[0]
    diff_corr['Mode'] = diff_corr_mode_dict[mode_index]
    del body_array[:1]

    # Source: u1; Indicates the receiver connection from which the message has been received
    source_code = struct.unpack('<B', body_array[:1])[0]
    if source_code in diff_corr_source_dict.keys():
        diff_corr['Source'] = diff_corr_source_dict[source_code]
    else:
        diff_corr['Source'] = 'Unknown'
    del body_array[:1]

    # get sub-blocks
    if mode_index in (0, 1, 2, 3):
        sub_dict = diffCorrInSubParser(mode_index=mode_index, body_array=body_array)
        for k, v in sub_dict.items():
            diff_corr[k] = v

    # delete padding
    del body_array[:1]

    return diff_corr


def diffCorrInSubParser(mode_index, body_array):
    """
    parse the sub block in INSNavCart

    0: RTCMv2
    1: CMRv2
    2: RTCMv3
    3: RTCMV (a proprietary variant of RTCM2)
    4: SPARTN
    """
    mode_data = (
        (0, 'RTCM2Words'),
        (1, 'CMRMessage'),
        (2, 'RTCM3Message'),
        (3, 'RTCMVMessage'),
    )
    mode_data_dict = dict(mode_data)

    sb_name = mode_data_dict[mode_index]
    sb_dict = {}

    if mode_index == 0:
        # RTCM2Words; u4
        # Here is not real N; N = 2 + ((RTCM2Words[1]»9) & 0x1f);
        sb_dict[sb_name] = struct.unpack('<I', body_array[:4])[0]
        del body_array[:4]
    else:
        # u1
        sb_dict[sb_name] = struct.unpack('<B', body_array[:1])[0]
        del body_array[:1]

    return sb_dict


def baseStationParser(msg_body: bytes) -> dict:
    """
    The BaseStation block contains the ECEF coordinates of the base station
    the receiver is currently connected to.

    @return: parsed sbf message body data
    @rtype: dict
    """

    base_station = {}

    # tow = struct.unpack('<HHHH', header)
    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    base_station['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2
    base_station['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # BaseStationID: u2. The base station ID
    base_station['BaseStationID'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # BaseType: u1. Base station type: 0: Fixed; 1: Moving (reserved for future use); 255: Unknown
    base_type_code = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]
    base_station['BaseType'] = base_type_dict[base_type_code]

    # Source: u1; Source of the base station coordinates:
    source_code = struct.unpack('<B', body_array[:1])[0]
    if source_code in base_source_dict.keys():
        base_station['Source'] = [source_code, base_source_dict[source_code]]
    else:
        base_station['Source'] = [source_code, 'Unknown']
    del body_array[:1]

    # Datum; u1; 255; Not applicable
    del body_array[:1]
    # Reserved; u1; Reserved for future use, to be ignored by decoding software
    del body_array[:1]

    # X: f8; Unit in meter. Antenna X coordinate expressed in the datum specified by the Datum field
    base_station['X'] = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]
    # Y: f8; Unit in meter. Antenna Y coordinate
    base_station['Y'] = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]
    # Z: f8; Unit in meter. Antenna Z coordinate
    base_station['Z'] = struct.unpack('<d', body_array[:8])[0]
    del body_array[:8]

    # delete padding
    del body_array[:1]

    return base_station


def rtcmDatumParser(msg_body: bytes) -> dict:
    """
    This block reports the source and target datum names as transmitted in RTCM 3.x message
    types 1021 or 1022. It also reports the corresponding height and quality indicators.

    If a service provider only sends out message types 1021 or 1022, this block is transmitted
    immediately after reception of MT1021 or MT1022. If message types 1023 or 1024
    are also sent out, this block is transmitted after the reception of these messages and the
    QualityInd field is set accordingly.


    @return: parsed sbf message body data
    @rtype: dict
    """

    rtcm_datum = {}

    # tow = struct.unpack('<HHHH', header)
    body_array = bytearray(msg_body)

    # TOW: u4; unsigned integer 4 bytes
    rtcm_datum['TOW'] = struct.unpack('<I', body_array[:4])[0]
    del body_array[:4]

    # WNc: u2
    rtcm_datum['WNc'] = struct.unpack('<H', body_array[:2])[0]
    del body_array[:2]

    # SourceCRS c1[32]. Name of the source Coordinate Reference System, right-padded with zeros.
    source_crs_bytes = struct.unpack('32s', body_array[:32])[0]
    # remove the right-padded zeros.
    rtcm_datum['SourceCRS'] = source_crs_bytes.rstrip(b'\x00').decode("utf-8")
    del body_array[:32]

    # TargetCRS c1[32]. Name of the target Coordinate Reference System, right-padded with zeros.
    target_crs_bytes = struct.unpack('32s', body_array[:32])[0]
    # remove the right-padded zeros.
    rtcm_datum['TargetCRS'] = target_crs_bytes.rstrip(b'\x00').decode("utf-8")
    del body_array[:32]

    # Datum: u1;
    datum_code = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]
    if datum_code == 255:
        rtcm_datum['Datum'] = "SourceCRS/TargetCRS pair is currently not used by the receiver"
    else:
        rtcm_datum['Datum'] = datum_code

    # HeightType: u1; rtcm_height_dict
    height_type_code = struct.unpack('<B', body_array[:1])[0]
    del body_array[:1]
    if height_type_code in rtcm_height_dict.keys():
        rtcm_datum["HeightType"] = rtcm_height_dict[height_type_code]
    else:
        rtcm_datum["HeightType"] = "Unknown"

    # QualityInd: u1; maximum approximation error after applying the transformation
    quality_byte = body_array[0]
    del body_array[:1]
    horizontal_quality = quality_byte & 0x0F
    vertical_quality = (quality_byte >> 4) & 0x0F

    if horizontal_quality in rtcm_quality_dict.keys():
        horizontal_q_list = [horizontal_quality, rtcm_quality_dict[horizontal_quality]]
    else:
        horizontal_q_list = [horizontal_quality, "Unknown"]

    if vertical_quality in rtcm_quality_dict.keys():
        vertical_q_list = [vertical_quality, rtcm_quality_dict[vertical_quality]]
    else:
        vertical_q_list = [vertical_quality, "Unknown"]

    rtcm_datum["QualityInd"] = {"horizontal": horizontal_q_list, "vertical": vertical_q_list}

    # delete padding
    del body_array[:1]

    return rtcm_datum