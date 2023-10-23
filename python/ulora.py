import time
import math
from ucollections import namedtuple
from urandom import getrandbits
from machine import SPI
from machine import Pin
import ubinascii
import json
import machine
import socket
import mm_wlan
import struct

#Constants
FLAGS_ACK = 0x80
BROADCAST_ADDRESS = 255

REG_00_FIFO = 0x00
REG_01_OP_MODE = 0x01
REG_06_FRF_MSB = 0x06
REG_07_FRF_MID = 0x07
REG_08_FRF_LSB = 0x08
REG_0E_FIFO_TX_BASE_ADDR = 0x0e
REG_0F_FIFO_RX_BASE_ADDR = 0x0f
REG_10_FIFO_RX_CURRENT_ADDR = 0x10
REG_12_IRQ_FLAGS = 0x12
REG_13_RX_NB_BYTES = 0x13
REG_1D_MODEM_CONFIG1 = 0x1d
REG_1E_MODEM_CONFIG2 = 0x1e
REG_19_PKT_SNR_VALUE = 0x19
REG_1A_PKT_RSSI_VALUE = 0x1a
REG_20_PREAMBLE_MSB = 0x20
REG_21_PREAMBLE_LSB = 0x21
REG_22_PAYLOAD_LENGTH = 0x22
REG_26_MODEM_CONFIG3 = 0x26

REG_4D_PA_DAC = 0x4d
REG_40_DIO_MAPPING1 = 0x40
REG_0D_FIFO_ADDR_PTR = 0x0d

PA_DAC_ENABLE = 0x07
PA_DAC_DISABLE = 0x04
PA_SELECT = 0x80

CAD_DETECTED_MASK = 0x01
RX_DONE = 0x40
CRC_ERROR = 0x20
TX_DONE = 0x08
CAD_DONE = 0x04
CAD_DETECTED = 0x01

LONG_RANGE_MODE = 0x80
MODE_SLEEP = 0x00 | 0x8
MODE_STDBY = 0x01 | 0x8
MODE_TX = 0x03 | 0x8
MODE_RXCONTINUOUS = 0x05 | 0x8
MODE_CAD = 0x07 | 0x8

REG_09_PA_CONFIG = 0x09
FXOSC = 32000000.0
FSTEP = (FXOSC / 524288)

maxRegLen = 23
maxFieldLen = maxRegLen
serverIp = '192.168.87.57'
server_port_up = 1700
server_port_down = 1700

rtc = machine.RTC()
def getFormattedTime(now, stat=False) -> str:
    """Generate an ISO datetime string for the time provided in now
    
    now is an 8 tuple used by machine.RTC with the following format:

    (year, month, day, weekday, hours, minutes, seconds, subseconds)

    """
    #dateformat = "{year}-{month}-{day:02d}-T{hour:02d}:{minute:02d}:{second:02d}.{subsecond}Z"
    #dateformat = "{year}-{month}-{day:02d} {hour:02d}:{minute:02d}:{second:02d} GMT"
    #dateformat = "{year}-{month}-{day:02d}-T{hour:02d}:{minute:02d}:{second:02d}"
    if stat:
        dateformat = "{year}-{month}-{day:02d} {hour:02d}:{minute:02d}:{second:02d} GMT"
    else:
        dateformat = "{year}-{month}-{day:02d}T{hour:02d}:{minute:02d}:{second:02d}.{subsecond}Z"

    date = dateformat.format(year=now[0], month=now[1], day=now[2], weekday=now[3], hour=now[4], minute=now[5], second=now[6], subsecond=now[7])
    return date

def dumpCfg(lora):
    print("-----------------------------------------------")
    OP_MODE = lora._spi_read(REG_01_OP_MODE)
    regName = f"OPMODE(0x{REG_01_OP_MODE})"
    print(f"{regName:>{maxRegLen}}: 0x{OP_MODE:02x}")
    print(f"\t{'LongRangeMode':>{maxFieldLen}}: 0x{(OP_MODE & 0x80) >> 7}")
    print(f"\t{'AccessSharedReg':>{maxFieldLen}}: 0x{(OP_MODE & 0x40) >> 6}")
    print(f"\t{'LowFrequencyModeOn':>{maxFieldLen}}: 0x{(OP_MODE & 0x08) >> 3}")
    print(f"\t{'Mode':>{maxFieldLen}}: {OP_MODE & 0x07:03b}")

    print("-----------------------------------------------")
    REGFRMSB = lora._spi_read(REG_06_FRF_MSB)
    regName = f"REGFRMSB(0x{REG_06_FRF_MSB})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRMSB:02x}")

    REGFRMID = lora._spi_read(REG_07_FRF_MID)
    regName = f"REGFRMID(0x{REG_07_FRF_MID})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRMID:02x}")

    REGFRLSB = lora._spi_read(REG_08_FRF_LSB)
    regName = f"REGFRLSB(0x{REG_08_FRF_LSB})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRLSB:02x}")

    frequency = (REGFRMSB*256*256 + REGFRMID*256 + REGFRLSB )*32e6/((1<<19))
    print(f"{'CarrierRF':>{maxRegLen}}: {frequency/1e6} MHz")

    print("-----------------------------------------------")
    REGPACONFIG = lora._spi_read(REG_09_PA_CONFIG)
    regName = f"REGPACONFIG(0x{REG_09_PA_CONFIG})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPACONFIG:02x}")
    print(f"\t{'PaSelect':>{maxFieldLen}}: 0x{(REGPACONFIG & 0x80) >> 7}")
    print(f"\t{'MaxPower':>{maxFieldLen}}: 0x{(REGPACONFIG & 0x70) >> 4}")
    print(f"\t{'OutputPower':>{maxFieldLen}}: 0x{REGPACONFIG & 0xf:02x}")

    print("-----------------------------------------------")
    REGPARAMP = lora._spi_read(0x0a)
    regName = f"REGPARAMP(0x{0xa:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPARAMP:02x}")
    print(f"\t{'PaRamp':>{maxFieldLen}}: {REGPARAMP & 0xf:04b}")

    print("-----------------------------------------------")
    REGOCP = lora._spi_read(0x0b)
    regName = f"REGOCP(0x{0xb:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGOCP:02x}")
    print(f"\t{'OcpOn':>{maxFieldLen}}: 0x{(REGOCP & 0x20) >> 5}")
    print(f"\t{'OcpTrim':>{maxFieldLen}}: 0x{REGOCP & 0x1f:02x}")

    print("-----------------------------------------------")
    REGLNA = lora._spi_read(0x0c)
    regName = f"REGLNA(0x{0xc:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGLNA:02x}")
    print(f"\t{'LnaGain':>{maxFieldLen}}: {(REGLNA & 0xe0) >> 5:03b}")
    print(f"\t{'LnaBoostLf':>{maxFieldLen}}: {(REGLNA & 0x18) >> 3:02b}")
    print(f"\t{'LnaBoostHf':>{maxFieldLen}}: {REGLNA & 0x03:02b}")

    print("-----------------------------------------------")
    REGFIFOADDRPTR = lora._spi_read(0x0d)
    regName = f"REGFIFOADDRPTR(0x{0xd:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFOADDRPTR:02x}")

    print("-----------------------------------------------")
    REGFIFOTXBASEADDR = lora._spi_read(REG_0E_FIFO_TX_BASE_ADDR)
    regName = f"REGFIFOTXBASEADDR(0x{REG_0E_FIFO_TX_BASE_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFOTXBASEADDR:02x}")

    print("-----------------------------------------------")
    REGFIFORXBASEADDR = lora._spi_read(REG_0F_FIFO_RX_BASE_ADDR)
    regName = f"REGFIFORXBASEADDR(0x{REG_0F_FIFO_RX_BASE_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBASEADDR:02x}")

    print("-----------------------------------------------")
    REGFIFORXBCURADDR = lora._spi_read(REG_10_FIFO_RX_CURRENT_ADDR)
    regName = f"REGFIFORXBASEADDR(0x{REG_10_FIFO_RX_CURRENT_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBCURADDR:02x}")

    print("-----------------------------------------------")
    REGIRQFLAGSMSK = lora._spi_read(0x11)
    regName = f"REGIRQFLAGSMSK(0x{0x11:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGIRQFLAGSMSK:02x}")
    print(f"\t{'RxTimeoutMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x80) >> 7}")
    print(f"\t{'RxDoneMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x40) >> 6}")
    print(f"\t{'PayloadCrcErrorMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x20) >> 5}")
    print(f"\t{'ValidHeaderMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x10) >> 4}")
    print(f"\t{'TxDoneMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x8) >> 3}")
    print(f"\t{'CADDoneMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x4) >> 2}")
    print(f"\t{'FhssChangeChannelMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x2) >> 1}")
    print(f"\t{'CadDetectedMask':>{maxFieldLen}}: {(REGIRQFLAGSMSK & 0x1)}")

    print("-----------------------------------------------")
    REGIRQFLAGS = lora._spi_read(0x12)
    regName = f"REGIRQFLAGS(0x{0x12:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGIRQFLAGS:02x}")
    print(f"\t{'RxTimeout':>{maxFieldLen}}: {(REGIRQFLAGS & 0x80) >> 7}")
    print(f"\t{'RxDone':>{maxFieldLen}}: {(REGIRQFLAGS & 0x40) >> 6}")
    print(f"\t{'PayloadCrcError':>{maxFieldLen}}: {(REGIRQFLAGS & 0x20) >> 5}")
    print(f"\t{'ValidHeader':>{maxFieldLen}}: {(REGIRQFLAGS & 0x10) >> 4}")
    print(f"\t{'TxDone':>{maxFieldLen}}: {(REGIRQFLAGS & 0x8) >> 3}")
    print(f"\t{'CADDone':>{maxFieldLen}}: {(REGIRQFLAGS & 0x4) >> 2}")
    print(f"\t{'FhssChangeChannel':>{maxFieldLen}}: {(REGIRQFLAGS & 0x2) >> 1}")
    print(f"\t{'CadDetected':>{maxFieldLen}}: {(REGIRQFLAGS & 0x1)}")

    print("-----------------------------------------------")
    REGRXNBBYTES = lora._spi_read(REG_13_RX_NB_BYTES)
    regName = f"REGRXNBBYTES(0x{REG_13_RX_NB_BYTES:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXNBBYTES:02x}")

    print("-----------------------------------------------")
    REGRXHDRCNTMSB = lora._spi_read(0x14)
    regName = f"REGRXHDRCNTMSB(0x{0x14:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXHDRCNTMSB:02x}")

    REGRXHDRCNTLSB = lora._spi_read(0x15)
    regName = f"REGRXHDRCNTLSB(0x{0x15:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXHDRCNTLSB:02x}")
    print(f"{'RegRxHeaderCnt':>{maxRegLen}}: {REGRXHDRCNTMSB*256 + REGRXHDRCNTLSB}")

    print("-----------------------------------------------")
    REGRXCNTMSB = lora._spi_read(0x16)
    regName = f"REGRXCNTMSB(0x{0x16:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXCNTMSB:02x}")

    REGRXCNTLSB = lora._spi_read(0x17)
    regName = f"REGRXCNTLSB(0x{0x17:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXCNTLSB:02x}")
    print(f"{'RegRxCnt':>{maxRegLen}}: {REGRXCNTMSB*256 + REGRXCNTLSB}")

    print("-----------------------------------------------")
    REGMODEMSTAT = lora._spi_read(0x18)
    regName = f"REGMODEMSTAT(0x{0x18:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMSTAT:02x}")
    print(f"\t{'RxCodingRate':>{maxFieldLen}}: 0x{(REGMODEMSTAT & 0xe) >> 5:02x}")
    print(f"\t{'Clear':>{maxFieldLen}}: {(REGMODEMSTAT & 0x10) >> 4}")
    print(f"\t{'HdrValid':>{maxFieldLen}}: {(REGMODEMSTAT & 0x8) >> 3}")
    print(f"\t{'RX on':>{maxFieldLen}}: {(REGMODEMSTAT & 0x4) >> 2}")
    print(f"\t{'Signal sync':>{maxFieldLen}}: {(REGMODEMSTAT & 0x2) >> 1}")
    print(f"\t{'Signal detect':>{maxFieldLen}}: {(REGMODEMSTAT & 0x1)}")

    print("-----------------------------------------------")
    REGPKTSNR = lora._spi_read(REG_19_PKT_SNR_VALUE)
    regName = f"REGPKTSNR(0x{REG_19_PKT_SNR_VALUE:02x})"
    if REGPKTSNR > 128:
        val = 127 - REGPKTSNR
    else:
        val = REGPKTSNR
    print(f"{regName:>{maxRegLen}}: {val/4.0}")

    print("-----------------------------------------------")
    REGPKTRSSI = lora._spi_read(REG_1A_PKT_RSSI_VALUE)
    regName = f"REGPKTRSSI(0x{REG_1A_PKT_RSSI_VALUE:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPKTRSSI:02x}")

    print("-----------------------------------------------")
    REGRSSI = lora._spi_read(0x1b)
    regName = f"REGRSSI(0x{0x1b:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRSSI:02x}")

    print("-----------------------------------------------")
    REGHOPCHAN = lora._spi_read(0x1c)
    regName = f"REGHOPCHANNEL(0x{0x1c:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGHOPCHAN:02x}")
    print(f"\t{'PllTimeout':>{maxFieldLen}}: {(REGHOPCHAN & 0x80) >> 7}")
    print(f"\t{'CrcOnPayload':>{maxFieldLen}}: {(REGHOPCHAN & 0x40) >> 6}")
    print(f"\t{'FhssPresentChannel':>{maxFieldLen}}: {(REGHOPCHAN & 0x3f)}")

    print("-----------------------------------------------")
    REGMODEMCFG1 = lora._spi_read(REG_1D_MODEM_CONFIG1)
    regName = f"REGMODEMCFG1(0x{REG_1D_MODEM_CONFIG1:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG1:02x}")
    print(f"\t{'Bw':>{maxFieldLen}}: {(REGMODEMCFG1 & 0xf0) >> 4:04b}")
    print(f"\t{'CodingRate':>{maxFieldLen}}: {(REGMODEMCFG1 & 0xe) >> 1:03b}")
    print(f"\t{'ImplicitHeaderModeOn':>{maxFieldLen}}: {REGMODEMCFG1 & 0x1}")


    print("-----------------------------------------------")
    REGMODEMCFG2 = lora._spi_read(REG_1E_MODEM_CONFIG2)
    regName = f"REGMODEMCFG2(0x{REG_1E_MODEM_CONFIG2:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG2:02x}")
    print(f"\t{'SF':>{maxFieldLen}}: {(REGMODEMCFG2 & 0xf0) >> 4}")
    print(f"\t{'TxContinuousMode':>{maxFieldLen}}: {(REGMODEMCFG2 & 0x8) >> 3}")
    print(f"\t{'RxPayloadCrcOn':>{maxFieldLen}}: {(REGMODEMCFG2 & 0x4) >> 2}")
    print(f"\t{'SymTimeoutMsb':>{maxFieldLen}}: 0x{(REGMODEMCFG2 & 0x3):02x}")

    print("-----------------------------------------------")
    REGSYMBTIMELSB = lora._spi_read(0x1f)
    regName = f"REGSYMBTIMELSB(0x{0x1f:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGSYMBTIMELSB:02x}")

    print("-----------------------------------------------")
    REGPREMSB = lora._spi_read(0x20)
    regName = f"REGPREMSB(0x{0x20:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPREMSB:02x}")

    REGPRELSB = lora._spi_read(0x21)
    regName = f"REGPRELSB(0x{0x21:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPRELSB:02x}")
    print(f"{'PreambleLen':>{maxRegLen}}: {REGPREMSB*256 + REGPRELSB}")

    print("-----------------------------------------------")
    REGPAYLEN = lora._spi_read(0x22)
    regName = f"REGPAYLEN(0x{0x22:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPAYLEN:02x}")

    print("-----------------------------------------------")
    REGHOPPERIOD = lora._spi_read(0x24)
    regName = f"REGHOPPERIOD(0x{0x24:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGHOPPERIOD:02x}")

    print("-----------------------------------------------")
    REGFIFORXBYTEADDR = lora._spi_read(0x25)
    regName = f"REGFIFORXBYTEADDR(0x{0x25:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBYTEADDR:02x}")

    print("-----------------------------------------------")
    REGMODEMCFG3 = lora._spi_read(REG_26_MODEM_CONFIG3)
    regName = f"REGMODEMCFG3(0x{REG_26_MODEM_CONFIG3:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG3:02x}")
    print(f"\t{'LowDataRateOpt':>{maxFieldLen}}: {(REGMODEMCFG3 & 0x8) >> 3}")
    print(f"\t{'AgcAutoOn':>{maxFieldLen}}: {(REGMODEMCFG3 & 0x4) >> 2}")

    print("-----------------------------------------------")
    REGPPMCORRECTION= lora._spi_read(0x27)
    regName = f"REGPPMCORRECTION(0x{0x27:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPPMCORRECTION:02x}")

    print("-----------------------------------------------")
    REGDETECTOPT = lora._spi_read(0x31)
    regName = f"REGDETECTOPT(0x{0x31:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGDETECTOPT:02x}")
    print(f"\t{'DetectionOptimize':>{maxFieldLen}}: 0x{(REGDETECTOPT & 0x7):02x}")

    print("-----------------------------------------------")
    REGINVERTIQ = lora._spi_read(0x33)
    regName = f"REGINVERTIQ(0x{0x33:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGINVERTIQ:02x}")
    print(f"\t{'InvertIQ':>{maxFieldLen}}: {(REGINVERTIQ & 0x40) >> 6}")

    print("-----------------------------------------------")
    REGDETECTTHRESH= lora._spi_read(0x37)
    regName = f"REGDETECTTHRESH(0x{0x37:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGDETECTTHRESH:02x}")

    print("-----------------------------------------------")
    REGSYNCWORD = lora._spi_read(0x39)
    regName = f"REGSYNCWORD(0x{0x39:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGSYNCWORD:02x}")

    print("-----------------------------------------------")
    REGINVERTIQ2 = lora._spi_read(0x3b)
    regName = f"REGINVERTIQ2(0x{0x3b:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGINVERTIQ2:02x}")

class ModemConfig():
    Bw125Cr45Sf128 = (0x72, 0x74, 0x04) #< Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Default medium range
    Bw500Cr45Sf128 = (0x92, 0x74, 0x04) #< Bw = 500 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on. Fast+short range
    Bw31_25Cr48Sf512 = (0x48, 0x94, 0x04) #< Bw = 31.25 kHz, Cr = 4/8, Sf = 512chips/symbol, CRC on. Slow+long range
    Bw125Cr48Sf4096 = (0x78, 0xc4, 0x0c) #/< Bw = 125 kHz, Cr = 4/8, Sf = 4096chips/symbol, low data rate, CRC on. Slow+long range
    Bw125Cr45Sf2048 = (0x72, 0xb4, 0x04) #< Bw = 125 kHz, Cr = 4/5, Sf = 2048chips/symbol, CRC on. Slow+long range
    lorawan = (0x72, 0xa4, 0x04)
    gateway = (0x92, 0xc0, 0x04)

class SPIConfig():
    # spi pin defs for various boards (channel, sck, mosi, miso)
    rp2_0 = (0, 6, 7, 4)
    rp2_1 = (1, 10, 11, 8)
    esp8286_1 = (1, 14, 13, 12)
    esp32_1 = (1, 14, 13, 12)
    esp32_2 = (2, 18, 23, 19)

class LoRa(object):
    def __init__(self, spi_channel, interrupt, this_address, cs_pin, reset_pin=None, freq=868.0, tx_power=14,
                 modem_config=ModemConfig.Bw125Cr45Sf128, receive_all=False, acks=False, crypto=None):
        """
        Lora(channel, interrupt, this_address, cs_pin, reset_pin=None, freq=868.0, tx_power=14,
                 modem_config=ModemConfig.Bw125Cr45Sf128, receive_all=False, acks=False, crypto=None)
        channel: SPI channel, check SPIConfig for preconfigured names
        interrupt: GPIO interrupt pin
        this_address: set address for this device [0-254]
        cs_pin: chip select pin from microcontroller 
        reset_pin: the GPIO used to reset the RFM9x if connected
        freq: frequency in MHz
        tx_power: transmit power in dBm
        modem_config: Check ModemConfig. Default is compatible with the Radiohead library
        receive_all: if True, don't filter packets on address
        acks: if True, request acknowledgments
        crypto: if desired, an instance of ucrypto AES (https://docs.pycom.io/firmwareapi/micropython/ucrypto/) - not tested
        """
        
        self._spi_channel = spi_channel
        self._interrupt = interrupt
        self._cs_pin = cs_pin

        self._mode = None
        self._cad = None
        self._freq = freq
        self._tx_power = tx_power
        self._modem_config = modem_config
        self._receive_all = receive_all
        self._acks = acks

        self._this_address = this_address
        self._last_header_id = 0

        self._last_payload = None
        self.crypto = crypto

        self.cad_timeout = 0
        self.send_retries = 2
        self.wait_packet_sent_timeout = 0.2
        self.retry_timeout = 0.2
        
        # Setup the module
#        gpio_interrupt = Pin(self._interrupt, Pin.IN, Pin.PULL_DOWN)
        gpio_interrupt = Pin(self._interrupt, Pin.IN)
        gpio_interrupt.irq(trigger=Pin.IRQ_RISING, handler=self._handle_interrupt)
        
        # reset the board
        if reset_pin:
            gpio_reset = Pin(reset_pin, Pin.OUT)
            gpio_reset.value(0)
            time.sleep(0.01)
            gpio_reset.value(1)
            time.sleep(0.01)

        # baud rate to 5MHz
        self.spi = SPI(self._spi_channel[0], 5000000,
                       sck=Pin(self._spi_channel[1]), mosi=Pin(self._spi_channel[2]), miso=Pin(self._spi_channel[3]))

        # cs gpio pin
        self.cs = Pin(self._cs_pin, Pin.OUT)
        self.cs.value(1)
        
        # set mode
        self._spi_write(REG_01_OP_MODE, MODE_SLEEP | LONG_RANGE_MODE)
        time.sleep(0.1)
        
        # check if mode is set
        assert self._spi_read(REG_01_OP_MODE) == (MODE_SLEEP | LONG_RANGE_MODE), \
            "LoRa initialization failed"

        self._spi_write(REG_0E_FIFO_TX_BASE_ADDR, 0)
        self._spi_write(REG_0F_FIFO_RX_BASE_ADDR, 0)
        
        self.set_mode_idle()

        # set modem config (Bw125Cr45Sf128)
        self._spi_write(REG_1D_MODEM_CONFIG1, self._modem_config[0])
        self._spi_write(REG_1E_MODEM_CONFIG2, self._modem_config[1])
        self._spi_write(REG_26_MODEM_CONFIG3, self._modem_config[2])

        # set preamble length (8)
        self._spi_write(REG_20_PREAMBLE_MSB, 0)
        self._spi_write(REG_21_PREAMBLE_LSB, 8)

        # set frequency
        frf = int((self._freq * 1000000.0) / FSTEP)
        self.fReg = ((frf >> 16) & 0xff, (frf >> 8) & 0xff, frf & 0xff)
        self._spi_write(REG_06_FRF_MSB, self.fReg[0])
        self._spi_write(REG_07_FRF_MID, self.fReg[1])
        self._spi_write(REG_08_FRF_LSB, self.fReg[2])
        
        # Set tx power
        if self._tx_power < 5:
            self._tx_power = 5
        if self._tx_power > 23:
            self._tx_power = 23

        if self._tx_power < 20:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_ENABLE)
            self._tx_power -= 3
        else:
            self._spi_write(REG_4D_PA_DAC, PA_DAC_DISABLE)

        self._spi_write(REG_09_PA_CONFIG, PA_SELECT | (self._tx_power - 5))

        # Make it more like lorawan
        self._spi_write(0x39, 0x34)

        self.sendsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.gwStatus = {'time': '',
                         'lati': 46.24,
                         'long': 3.2523,
                         'alti': 145,
                         'rxnb': 0,
                         'rxok': 0,
                         'rxfw': 0,
                         'ackr': 100.0,
                         'dwnb': 0,
                         'txnb': 0}
        self.id = mm_wlan.getmac() + 2*b'\xff'
        #self.recvsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.recvsock.bind(('0.0.0.0', server_port_down))


    def process(self):
        #(data, address) = self.recvsock.recvfrom(4096)

        print("Received a packet of size {len(data)} . . . now what?")

        
    def on_recv(self, message):
        # This should be overridden by the user
        pass

    def sleep(self):
        if self._mode != MODE_SLEEP:
            self._spi_write(REG_01_OP_MODE, MODE_SLEEP)
            self._mode = MODE_SLEEP

    def set_mode_tx(self):
        if self._mode != MODE_TX:
            self._spi_write(0x33, 0x26)   
            self._spi_write(0x3b, 0x19)
            self._spi_write(REG_01_OP_MODE, MODE_TX)
            self._spi_write(REG_40_DIO_MAPPING1, 0x40)  # Interrupt on TxDone
            self._mode = MODE_TX

    def set_mode_rx(self):
        if self._mode != MODE_RXCONTINUOUS:
            self._spi_write(0x33, 0x27)   
            self._spi_write(0x3b, 0x1d)
            self._spi_write(REG_01_OP_MODE, MODE_RXCONTINUOUS)
            self._spi_write(REG_40_DIO_MAPPING1, 0x00)  # Interrupt on RxDone
            self._mode = MODE_RXCONTINUOUS
            
    def set_mode_cad(self):
        if self._mode != MODE_CAD:
            self._spi_write(REG_01_OP_MODE, MODE_CAD)
            self._spi_write(REG_40_DIO_MAPPING1, 0x80)  # Interrupt on CadDone
            self._mode = MODE_CAD

    def _is_channel_active(self):
        self.set_mode_cad()

        while self._mode == MODE_CAD:
            yield

        return self._cad
    
    def wait_cad(self):
        if not self.cad_timeout:
            return True

        start = time.time()
        for status in self._is_channel_active():
            if time.time() - start < self.cad_timeout:
                return False

            if status is None:
                time.sleep(0.1)
                continue
            else:
                return status

    def wait_packet_sent(self):
        # wait for `_handle_interrupt` to switch the mode back
        start = time.time()
        while time.time() - start < self.wait_packet_sent_timeout:
            if self._mode != MODE_TX:
                return True

        return False

    def set_mode_idle(self):
        if self._mode != MODE_STDBY:
            self._spi_write(REG_01_OP_MODE, MODE_STDBY)
            self._mode = MODE_STDBY

    def send(self, data, header_to, header_id=0, header_flags=0):
        self.wait_packet_sent()
        self.set_mode_idle()
        self.wait_cad()

        header = [header_to, self._this_address, header_id, header_flags]
        if type(data) == int:
            data = [data]
        elif type(data) == bytes:
            data = [p for p in data]
        elif type(data) == str:
            data = [ord(s) for s in data]

        if self.crypto:
            data = [b for b in self._encrypt(bytes(data))]

        payload = header + data
        self._spi_write(REG_0D_FIFO_ADDR_PTR, 0)
        self._spi_write(REG_00_FIFO, payload)
        self._spi_write(REG_22_PAYLOAD_LENGTH, len(payload))

        self.set_mode_tx()
        return True

    def send_to_wait(self, data, header_to, header_flags=0, retries=3):
        self._last_header_id += 1

        for _ in range(retries + 1):
            self.send(data, header_to, header_id=self._last_header_id, header_flags=header_flags)
            self.set_mode_rx()

            if header_to == BROADCAST_ADDRESS:  # Don't wait for acks from a broadcast message
                return True

            start = time.time()
            while time.time() - start < self.retry_timeout + (self.retry_timeout * (getrandbits(16) / (2**16 - 1))):
                if self._last_payload:
                    if self._last_payload.header_to == self._this_address and \
                            self._last_payload.header_flags & FLAGS_ACK and \
                            self._last_payload.header_id == self._last_header_id:

                        # We got an ACK
                        return True
        return False

    def send_ack(self, header_to, header_id):
        self.send(b'!', header_to, header_id, FLAGS_ACK)
        self.wait_packet_sent()

    def _spi_write(self, register, payload):
        if type(payload) == int:
            payload = [payload]
        elif type(payload) == bytes:
            payload = [p for p in payload]
        elif type(payload) == str:
            payload = [ord(s) for s in payload]
        self.cs.value(0)
        self.spi.write(bytearray([register | 0x80] + payload))
        self.cs.value(1)

    def _spi_read(self, register, length=1):
        self.cs.value(0)
        if length == 1:
            data = self.spi.read(length + 1, register)[1]
        else:
            data = self.spi.read(length + 1, register)[1:]
        self.cs.value(1)
        return data
        
    def _decrypt(self, message):
        decrypted_msg = self.crypto.decrypt(message)
        msg_length = decrypted_msg[0]
        return decrypted_msg[1:msg_length + 1]

    def _encrypt(self, message):
        msg_length = len(message)
        padding = bytes(((math.ceil((msg_length + 1) / 16) * 16) - (msg_length + 1)) * [0])
        msg_bytes = bytes([msg_length]) + message + padding
        encrypted_msg = self.crypto.encrypt(msg_bytes)
        return encrypted_msg

    def _handle_interrupt(self, channel):
        now = time.ticks_ms()
        irq_flags = self._spi_read(REG_12_IRQ_FLAGS)

        print(f"Handling interrupt with 0x{irq_flags:02x}")

        if self._mode == MODE_RXCONTINUOUS and (irq_flags & RX_DONE):
            self.gwStatus['rxnb'] += 1
            if not (irq_flags & CRC_ERROR):
                self.gwStatus['rxok'] += 1
            packet_len = self._spi_read(REG_13_RX_NB_BYTES)
            self._spi_write(REG_0D_FIFO_ADDR_PTR, self._spi_read(REG_10_FIFO_RX_CURRENT_ADDR))

            packet = self._spi_read(REG_00_FIFO, packet_len)
            self._spi_write(REG_12_IRQ_FLAGS, 0xff)  # Clear all IRQ flags

            snr = self._spi_read(REG_19_PKT_SNR_VALUE) / 4
            rssi = self._spi_read(REG_1A_PKT_RSSI_VALUE)

            if snr < 0:
                rssi = snr + rssi
            else:
                rssi = rssi * 16 / 15

            if self._freq >= 779:
                rssi = round(rssi - 157, 2)
            else:
                rssi = round(rssi - 164, 2)

            hexpacket = ''.join(f'0x{x:02x} ' for x in packet)
            print(f"rssi={rssi}, snr={snr}")
            print(f"Received a packet of size {packet_len}:{hexpacket}")
            rawts = rtc.datetime()
            ts = getFormattedTime(rawts)            
            self.gwStatus['time'] = getFormattedTime(rawts, stat=True)
            self.lastRx = now
            fwd = {"rxpk": [{
                   'time': ts,
                   'tmms': (time.time() -  time.mktime((1980, 1, 6, 0, 0, 0, 0, 5)))*1000,
                   'tmst': now,
                   'freq': self._freq,
                   'chan': 0,
                   'rfch': 0,
                   'stat': 1,
                   'modu': 'LORA',
                   'datr': 'SF10BW125',
                   'codr': '4/5',
                   'rssi': int(rssi),
                   'lsnr': snr,
                   'size': packet_len,
                   'data': ubinascii.b2a_base64(packet).strip()
                   }],
                   "stat": self.gwStatus}
            
            gwPacket = struct.pack('!BHB8s', 2, self.gwStatus['rxfw'], 0, self.id)
            gwPacket += bytes(json.dumps(fwd), 'utf-8')
            self.sendsock.sendto(gwPacket, (serverIp, server_port_up))
            print("Waiting for response from server . . .")
            (response, address) = self.sendsock.recvfrom(4)
            print(f"Received {response}")
            self.gwStatus['rxok'] += 1            
            self.gwStatus['rxfw'] += 1
        
            if packet_len >= 4:
                header_to = packet[0]
                header_from = packet[1]
                header_id = packet[2]
                header_flags = packet[3]
                message = bytes(packet[4:]) if packet_len > 4 else b''


                if (self._this_address != header_to) and ((header_to != BROADCAST_ADDRESS) or (self._receive_all is False)):
                    return

                if self.crypto and len(message) % 16 == 0:
                    message = self._decrypt(message)

                if self._acks and header_to == self._this_address and not header_flags & FLAGS_ACK:
                    self.send_ack(header_from, header_id)

                self.set_mode_rx()

                self._last_payload = namedtuple(
                    "Payload",
                    ['message', 'header_to', 'header_from', 'header_id', 'header_flags', 'rssi', 'snr']
                )(message, header_to, header_from, header_id, header_flags, rssi, snr)

                if not header_flags & FLAGS_ACK:
                    self.on_recv(self._last_payload)

        elif self._mode == MODE_TX and (irq_flags & TX_DONE):
            # self.set_mode_idle()

            # Restore settings
            self._spi_write(REG_1D_MODEM_CONFIG1, self._modem_config[0])
            self._spi_write(REG_1E_MODEM_CONFIG2, self._modem_config[1])
            #self._spi_write(REG_26_MODEM_CONFIG3, self._modem_config[2])
            self._spi_write(REG_06_FRF_MSB, self.fReg[0])
            self._spi_write(REG_07_FRF_MID, self.fReg[1])
            self._spi_write(REG_08_FRF_LSB, self.fReg[2])

            self.set_mode_rx()

        elif self._mode == MODE_CAD and (irq_flags & CAD_DONE):
            self._cad = irq_flags & CAD_DETECTED
            self.set_mode_idle()

        self._spi_write(REG_12_IRQ_FLAGS, 0xff)

    def close(self):
        self.spi.deinit()

    def sendMessage(self, message):
        print(f'Sending {message}')
        modulation = message['txInfo']['modulation']
        frequency = message['txInfo']['frequency']

        # Compute new configuration register values
        Bw = 0x90                              # 500kHz          
        CodingRate = 0x2                       # 4/5
        RegModemConfig1 = Bw | CodingRate

        SpreadingFactor = modulation['lora']['spreadingFactor'] << 4
        RxPayloadCrcOn = 0x4
        RegModemConfig2 = SpreadingFactor | RxPayloadCrcOn

        # Assuming RegModemConfig3 does not change

        frf = int((frequency) / FSTEP)

        # Setup for transmit
        self._spi_write(REG_1D_MODEM_CONFIG1, RegModemConfig1)
        self._spi_write(REG_1E_MODEM_CONFIG2, RegModemConfig2)
        self._spi_write(REG_06_FRF_MSB, (frf >> 16) & 0xff)
        self._spi_write(REG_07_FRF_MID, (frf >> 8) & 0xff)
        self._spi_write(REG_08_FRF_LSB, frf & 0xff)

        # Send packet
        self.wait_packet_sent()
        self.set_mode_idle()
        self.wait_cad()
        payload =  ubinascii.a2b_base64(message['phyPayload'])
        self._spi_write(REG_0D_FIFO_ADDR_PTR, 0)
        self._spi_write(REG_00_FIFO, payload)
        self._spi_write(REG_22_PAYLOAD_LENGTH, len(payload))
        self.set_mode_tx()

        dumpCfg(self)



