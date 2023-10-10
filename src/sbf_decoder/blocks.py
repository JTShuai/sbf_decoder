# -*- coding: UTF-8 -*-
'''
@Author  ：Jiangtao Shuai
@Date    ：03/04/2023 12:05 PM
'''


BLOCK_NUMBERS = [
    4050, 4225, 4226,
    5919, 5949, 4049,
]

BLOCK_NAMES = [
    'ExtSensorMeas',        'INSNavCart',         'INSNavGeod',
    'DiffCorrIn',           'BaseStation',        'RTCMDatum',
]

# body parsers list
BODY_PARSERS = [
    'extSensorMeasParser', 'insNavCartParser', 'insNavGeodParser',
    'diffCorrInParser', 'baseStationParser', 'rtcmDatumParser'
]
