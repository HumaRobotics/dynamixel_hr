#!/usr/bin/env python

# Dynamixel library for MX28 and MX64

# WINDOWS WARNING: For best performance, parameters of the COM Port should be set to maximum baud rate, and 1ms delay (Device Manager, COM Ports, properties, advanced)

class DxlException(Exception):                    pass
class DxlConfigurationException(DxlException):    pass
class DxlCommunicationException(DxlException):    pass


class Dxl:
    BROADCAST      = 0xFE
    
    CMD_PING       = 0x01
    CMD_READ_DATA  = 0x02
    CMD_WRITE_DATA = 0x03
    CMD_REG_WRITE  = 0x04
    CMD_ACTION     = 0x05
    CMD_RESET      = 0x06
    CMD_SYNC_WRITE = 0x83
    