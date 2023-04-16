from .exceptions import *
from machine import UART, Pin
from .registers import IFCNT
import sys
import struct
import uasyncio

def cast(T, a):
    if False:
        from typing import cast
        return cast(T, a)
    return a

class TMC_UART(object):

    rFrame  = [0x55, 0, 0, 0]
    wFrame  = [0x55, 0, 0, 0, 0, 0, 0, 0]
    communication_pause = 0
    
    def __init__(self, serialport=0, baudrate=115200, tx: int | Pin | None = None, rx: int | Pin | None = None, addr = 0):
        if serialport == 0:
            self.serial = UART(serialport, baudrate)
        else:
            if tx is None and rx is None:
                raise TMC_UART_MissingRXTXException()
            tx_pin = cast(Pin, Pin(tx, mode=Pin.OUT) if type(tx) is int else tx)
            rx_pin = cast(Pin, Pin(rx) if type(rx) is int else rx)
            self.serial = UART(serialport, baudrate, tx=tx_pin, rx=rx_pin) # type: ignore
        self.addr = addr
        self.communication_pause = 2000/baudrate     # adjust per baud and hardware. Sequential reads without some delay fail.
        
    def __del__(self):
        self.serial.deinit()

    def compute_crc8_atm(self, datagram, initial_value=0):
        crc = initial_value
        for byte in datagram:
            for _ in range(0, 8):
                if (crc >> 7) ^ (byte & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                byte = byte >> 1
        return crc
    
    async def read_reg(self, reg):
        
        rtn = bytes()
        
        self.rFrame[1] = self.addr
        self.rFrame[2] = reg
        self.rFrame[3] = self.compute_crc8_atm(self.rFrame[:-1])

        rt = self.serial.write(bytes(self.rFrame))
        if rt != len(self.rFrame):
            raise TMC_UART_WriteError()
        await uasyncio.sleep_ms(int(self.communication_pause))
        if self.serial.any():
            rtn = self.serial.read()
        await uasyncio.sleep_ms(int(self.communication_pause))
        if rtn == None:
            raise TMC_UART_ReadError()
        return rtn[7:11]

    async def read_int(self, reg):
        tries = 0
        while True:
            rtn = await self.read_reg(reg)
            tries += 1
            if rtn is None or len(rtn)>=4 :
                break
            if tries >= 10:
                raise TMC_UART_RepeatedInvalidResponseException()
        val = struct.unpack('>i',rtn)[0]
        return val

    async def write_reg(self, reg, val):
        
        self.wFrame[1] = self.addr
        self.wFrame[2] =  reg | 0x80;  # set write bit
        
        self.wFrame[3] = 0xFF & (val>>24)
        self.wFrame[4] = 0xFF & (val>>16)
        self.wFrame[5] = 0xFF & (val>>8)
        self.wFrame[6] = 0xFF & val
        
        self.wFrame[7] = self.compute_crc8_atm(self.wFrame[:-1])

        rtn = self.serial.write(bytes(self.wFrame))
        if rtn != len(self.wFrame):
            raise TMC_UART_WriteError()
        await uasyncio.sleep_ms(int(self.communication_pause))

        return True

    async def write_reg_check(self, reg, val):
        ifcnt1 = await self.read_int(IFCNT)
        await self.write_reg(reg, val)
        ifcnt2 = await self.read_int(IFCNT)
        
        if(ifcnt1 >= ifcnt2):
            # raise TMC_UART_WriteCheckFailedException(reg, val, ifcnt1, ifcnt2)
            return False
        else:
            return True

    def flushSerialBuffer(self):
        if hasattr(self.serial, 'flush'):
            try:
                self.serial.flush()     # type:ignore
            except:
                pass
        return
