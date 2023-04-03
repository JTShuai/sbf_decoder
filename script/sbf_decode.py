# -*- coding: UTF-8 -*-
'''
@Author  ：Jiangtao Shuai
@Date    ：03/04/2023 1:41 PM 
'''

from sbf_decoder.sbf_decoder import sbfDecoder
import struct
import socket

#################################################
### decode sbf data streaming
#################################################
def readSbfDataStream(ip:str, port:int):
    """
    decode the sensor data stream

    :param ip: device ip
    :param port: sbf streaming port
    :return: generator; sbf-blockname + sbf-block dictionary
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_address = (ip, port)

    # bind(): local
    # connect(): remote
    s.connect(server_address)

    while True:
        msg, address = s.recvfrom(1024*2)
        for name, data in sbfDecoder(msg,True):
            yield name, data


#################################################
### decode .sbf log file
#################################################
def readSbfLogFile(filename):
    """
    decode a sbf log file
    :param filename: .sbf log file
    :return: generator; sbf-blockname + sbf-block dictionary
    """
    with open(filename, 'rb') as f:
        """DON'T REMOVE NEWLINE CHARACTERS"""
        b_array = bytearray(f.read())

    while True:

        # header length: 8 bytes
        if len(b_array) <= 8:
            print('\n', "DON'T FIND HEADER")
            break

        # read log file, offline
        for blockname, block_dict in sbfDecoder(b_array, is_online=False):
            header_fields = struct.unpack('<HHHH', b_array[:8])
            msg_len = header_fields[-1]

            del b_array[:msg_len]

            yield blockname, block_dict


if __name__ == "__main__":
    test_b = bytearray(
    b'$@\x04\xc1\xd2\x0fH\x00\xf1}\x83\x08\xce\x08\x02\x1c \n\x00\x01\xe5a\xa1\xd64\xef\xa0?-\xeci\x87\xbf\xa6\xd2\xbf\xa2\x7f\x82\x8b\x15\x95#@ \n\x01\x01\xf7\xff\xff\xff\xff\xff\x9f\xbf.33333\xe5?\xc9\xcc\xcc\xcc\xcc\xcc\xc0\xbf$@\xc6\xac\xd2\x0fH\x00\xf6}\x83\x08\xce\x08\x02\x1c \n\x00\x01^\x11\xfco%;\xae?\xfc\xe3\xbdjeB\xd2\xbf\x9b\x8fkC\xc5\x94#@ \n\x01\x01v\x14\xaeG\xe1z\xd2\xbf\x8c\xc2\xf5(\\\x8f\xb2?j=\n\xd7\xa3p\xbd\xbf'
    )

    for name, data in sbfDecoder(msg=test_b, is_online=True):
        print(name, data)
