# MicroPython lora simple_rxtx example - synchronous API version
# MIT license; Copyright (c) 2023 Angus Gratton
import time
from machine import Pin, SPI

maxRegLen = 23
maxFieldLen = maxRegLen
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
REG_09_PA_CONFIG = 0x09

def dumpCfg(lora):
    print("-----------------------------------------------")
    OP_MODE = lora._reg_read(REG_01_OP_MODE)
    regName = f"OPMODE(0x{REG_01_OP_MODE})"
    print(f"{regName:>{maxRegLen}}: 0x{OP_MODE:02x}")
    print(f"\t{'LongRangeMode':>{maxFieldLen}}: 0x{(OP_MODE & 0x80) >> 7}")
    print(f"\t{'AccessSharedReg':>{maxFieldLen}}: 0x{(OP_MODE & 0x40) >> 6}")
    print(f"\t{'LowFrequencyModeOn':>{maxFieldLen}}: 0x{(OP_MODE & 0x08) >> 3}")
    print(f"\t{'Mode':>{maxFieldLen}}: {OP_MODE & 0x07:03b}")

    print("-----------------------------------------------")
    REGFRMSB = lora._reg_read(REG_06_FRF_MSB)
    regName = f"REGFRMSB(0x{REG_06_FRF_MSB})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRMSB:02x}")

    REGFRMID = lora._reg_read(REG_07_FRF_MID)
    regName = f"REGFRMID(0x{REG_07_FRF_MID})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRMID:02x}")

    REGFRLSB = lora._reg_read(REG_08_FRF_LSB)
    regName = f"REGFRLSB(0x{REG_08_FRF_LSB})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFRLSB:02x}")

    frequency = (REGFRMSB*256*256 + REGFRMID*256 + REGFRLSB )*32e6/((1<<19))
    print(f"{'CarrierRF':>{maxRegLen}}: {frequency/1e6} MHz")

    print("-----------------------------------------------")
    REGPACONFIG = lora._reg_read(REG_09_PA_CONFIG)
    regName = f"REGPACONFIG(0x{REG_09_PA_CONFIG})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPACONFIG:02x}")
    print(f"\t{'PaSelect':>{maxFieldLen}}: 0x{(REGPACONFIG & 0x80) >> 7}")
    print(f"\t{'MaxPower':>{maxFieldLen}}: 0x{(REGPACONFIG & 0x70) >> 4}")
    print(f"\t{'OutputPower':>{maxFieldLen}}: 0x{REGPACONFIG & 0xf:02x}")

    print("-----------------------------------------------")
    REGPARAMP = lora._reg_read(0x0a)
    regName = f"REGPARAMP(0x{0xa:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPARAMP:02x}")
    print(f"\t{'PaRamp':>{maxFieldLen}}: {REGPARAMP & 0xf:04b}")

    print("-----------------------------------------------")
    REGOCP = lora._reg_read(0x0b)
    regName = f"REGOCP(0x{0xb:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGOCP:02x}")
    print(f"\t{'OcpOn':>{maxFieldLen}}: 0x{(REGOCP & 0x20) >> 5}")
    print(f"\t{'OcpTrim':>{maxFieldLen}}: 0x{REGOCP & 0x1f:02x}")

    print("-----------------------------------------------")
    REGLNA = lora._reg_read(0x0c)
    regName = f"REGLNA(0x{0xc:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGLNA:02x}")
    print(f"\t{'LnaGain':>{maxFieldLen}}: {(REGLNA & 0xe0) >> 5:03b}")
    print(f"\t{'LnaBoostLf':>{maxFieldLen}}: {(REGLNA & 0x18) >> 3:02b}")
    print(f"\t{'LnaBoostHf':>{maxFieldLen}}: {REGLNA & 0x03:02b}")

    print("-----------------------------------------------")
    REGFIFOADDRPTR = lora._reg_read(0x0d)
    regName = f"REGFIFOADDRPTR(0x{0xd:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFOADDRPTR:02x}")

    print("-----------------------------------------------")
    REGFIFOTXBASEADDR = lora._reg_read(REG_0E_FIFO_TX_BASE_ADDR)
    regName = f"REGFIFOTXBASEADDR(0x{REG_0E_FIFO_TX_BASE_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFOTXBASEADDR:02x}")

    print("-----------------------------------------------")
    REGFIFORXBASEADDR = lora._reg_read(REG_0F_FIFO_RX_BASE_ADDR)
    regName = f"REGFIFORXBASEADDR(0x{REG_0F_FIFO_RX_BASE_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBASEADDR:02x}")

    print("-----------------------------------------------")
    REGFIFORXBCURADDR = lora._reg_read(REG_10_FIFO_RX_CURRENT_ADDR)
    regName = f"REGFIFORXBASEADDR(0x{REG_10_FIFO_RX_CURRENT_ADDR:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBCURADDR:02x}")

    print("-----------------------------------------------")
    REGIRQFLAGSMSK = lora._reg_read(0x11)
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
    REGIRQFLAGS = lora._reg_read(0x12)
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
    REGRXNBBYTES = lora._reg_read(REG_13_RX_NB_BYTES)
    regName = f"REGRXNBBYTES(0x{REG_13_RX_NB_BYTES:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXNBBYTES:02x}")

    print("-----------------------------------------------")
    REGRXHDRCNTMSB = lora._reg_read(0x14)
    regName = f"REGRXHDRCNTMSB(0x{0x14:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXHDRCNTMSB:02x}")

    REGRXHDRCNTLSB = lora._reg_read(0x15)
    regName = f"REGRXHDRCNTLSB(0x{0x15:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXHDRCNTLSB:02x}")
    print(f"{'RegRxHeaderCnt':>{maxRegLen}}: {REGRXHDRCNTMSB*256 + REGRXHDRCNTLSB}")

    print("-----------------------------------------------")
    REGRXCNTMSB = lora._reg_read(0x16)
    regName = f"REGRXCNTMSB(0x{0x16:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXCNTMSB:02x}")

    REGRXCNTLSB = lora._reg_read(0x17)
    regName = f"REGRXCNTLSB(0x{0x17:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRXCNTLSB:02x}")
    print(f"{'RegRxCnt':>{maxRegLen}}: {REGRXCNTMSB*256 + REGRXCNTLSB}")

    print("-----------------------------------------------")
    REGMODEMSTAT = lora._reg_read(0x18)
    regName = f"REGMODEMSTAT(0x{0x18:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMSTAT:02x}")
    print(f"\t{'RxCodingRate':>{maxFieldLen}}: 0x{(REGMODEMSTAT & 0xe) >> 5:02x}")
    print(f"\t{'Clear':>{maxFieldLen}}: {(REGMODEMSTAT & 0x10) >> 4}")
    print(f"\t{'HdrValid':>{maxFieldLen}}: {(REGMODEMSTAT & 0x8) >> 3}")
    print(f"\t{'RX on':>{maxFieldLen}}: {(REGMODEMSTAT & 0x4) >> 2}")
    print(f"\t{'Signal sync':>{maxFieldLen}}: {(REGMODEMSTAT & 0x2) >> 1}")
    print(f"\t{'Signal detect':>{maxFieldLen}}: {(REGMODEMSTAT & 0x1)}")

    print("-----------------------------------------------")
    REGPKTSNR = lora._reg_read(REG_19_PKT_SNR_VALUE)
    regName = f"REGPKTSNR(0x{REG_19_PKT_SNR_VALUE:02x})"
    if REGPKTSNR > 128:
        val = 127 - REGPKTSNR
    else:
        val = REGPKTSNR
    print(f"{regName:>{maxRegLen}}: {val/4.0}")

    print("-----------------------------------------------")
    REGPKTRSSI = lora._reg_read(REG_1A_PKT_RSSI_VALUE)
    regName = f"REGPKTRSSI(0x{REG_1A_PKT_RSSI_VALUE:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPKTRSSI:02x}")

    print("-----------------------------------------------")
    REGRSSI = lora._reg_read(0x1b)
    regName = f"REGRSSI(0x{0x1b:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGRSSI:02x}")

    print("-----------------------------------------------")
    REGHOPCHAN = lora._reg_read(0x1c)
    regName = f"REGHOPCHANNEL(0x{0x1c:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGHOPCHAN:02x}")
    print(f"\t{'PllTimeout':>{maxFieldLen}}: {(REGHOPCHAN & 0x80) >> 7}")
    print(f"\t{'CrcOnPayload':>{maxFieldLen}}: {(REGHOPCHAN & 0x40) >> 6}")
    print(f"\t{'FhssPresentChannel':>{maxFieldLen}}: {(REGHOPCHAN & 0x3f)}")

    print("-----------------------------------------------")
    REGMODEMCFG1 = lora._reg_read(REG_1D_MODEM_CONFIG1)
    regName = f"REGMODEMCFG1(0x{REG_1D_MODEM_CONFIG1:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG1:02x}")
    print(f"\t{'Bw':>{maxFieldLen}}: {(REGMODEMCFG1 & 0xf0) >> 4:04b}")
    print(f"\t{'CodingRate':>{maxFieldLen}}: {(REGMODEMCFG1 & 0xe) >> 1:03b}")
    print(f"\t{'ImplicitHeaderModeOn':>{maxFieldLen}}: {REGMODEMCFG1 & 0x1}")


    print("-----------------------------------------------")
    REGMODEMCFG2 = lora._reg_read(REG_1E_MODEM_CONFIG2)
    regName = f"REGMODEMCFG2(0x{REG_1E_MODEM_CONFIG2:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG2:02x}")
    print(f"\t{'SF':>{maxFieldLen}}: {(REGMODEMCFG2 & 0xf0) >> 4}")
    print(f"\t{'TxContinuousMode':>{maxFieldLen}}: {(REGMODEMCFG2 & 0x8) >> 3}")
    print(f"\t{'RxPayloadCrcOn':>{maxFieldLen}}: {(REGMODEMCFG2 & 0x4) >> 2}")
    print(f"\t{'SymTimeoutMsb':>{maxFieldLen}}: 0x{(REGMODEMCFG2 & 0x3):02x}")

    print("-----------------------------------------------")
    REGSYMBTIMELSB = lora._reg_read(0x1f)
    regName = f"REGSYMBTIMELSB(0x{0x1f:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGSYMBTIMELSB:02x}")

    print("-----------------------------------------------")
    REGPREMSB = lora._reg_read(0x20)
    regName = f"REGPREMSB(0x{0x20:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPREMSB:02x}")

    REGPRELSB = lora._reg_read(0x21)
    regName = f"REGPRELSB(0x{0x21:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPRELSB:02x}")
    print(f"{'PreambleLen':>{maxRegLen}}: {REGPREMSB*256 + REGPRELSB}")

    print("-----------------------------------------------")
    REGPAYLEN = lora._reg_read(0x22)
    regName = f"REGPAYLEN(0x{0x22:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPAYLEN:02x}")

    print("-----------------------------------------------")
    REGHOPPERIOD = lora._reg_read(0x24)
    regName = f"REGHOPPERIOD(0x{0x24:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGHOPPERIOD:02x}")

    print("-----------------------------------------------")
    REGFIFORXBYTEADDR = lora._reg_read(0x25)
    regName = f"REGFIFORXBYTEADDR(0x{0x25:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGFIFORXBYTEADDR:02x}")

    print("-----------------------------------------------")
    REGMODEMCFG3 = lora._reg_read(REG_26_MODEM_CONFIG3)
    regName = f"REGMODEMCFG3(0x{REG_26_MODEM_CONFIG3:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGMODEMCFG3:02x}")
    print(f"\t{'LowDataRateOpt':>{maxFieldLen}}: {(REGMODEMCFG3 & 0x8) >> 3}")
    print(f"\t{'AgcAutoOn':>{maxFieldLen}}: {(REGMODEMCFG3 & 0x4) >> 2}")

    print("-----------------------------------------------")
    REGPPMCORRECTION= lora._reg_read(0x27)
    regName = f"REGPPMCORRECTION(0x{0x27:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGPPMCORRECTION:02x}")

    print("-----------------------------------------------")
    REGDETECTOPT = lora._reg_read(0x31)
    regName = f"REGDETECTOPT(0x{0x31:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGDETECTOPT:02x}")
    print(f"\t{'DetectionOptimize':>{maxFieldLen}}: 0x{(REGDETECTOPT & 0x7):02x}")

    print("-----------------------------------------------")
    REGINVERTIQ = lora._reg_read(0x33)
    regName = f"REGINVERTIQ(0x{0x33:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGINVERTIQ:02x}")
    print(f"\t{'InvertIQ':>{maxFieldLen}}: {(REGINVERTIQ & 0x40) >> 6}")

    print("-----------------------------------------------")
    REGDETECTTHRESH= lora._reg_read(0x37)
    regName = f"REGDETECTTHRESH(0x{0x37:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGDETECTTHRESH:02x}")

    print("-----------------------------------------------")
    REGSYNCWORD = lora._reg_read(0x39)
    regName = f"REGSYNCWORD(0x{0x39:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGSYNCWORD:02x}")

    print("-----------------------------------------------")
    REGINVERTIQ2 = lora._reg_read(0x3b)
    regName = f"REGINVERTIQ2(0x{0x3b:02x})"
    print(f"{regName:>{maxRegLen}}: 0x{REGINVERTIQ2:02x}")


def get_modem():
    from lora import SX1276
    
    lora_cfg = {
       "freq_khz": 923300,
       "sf": 10,
       "bw": "500",  # kHz
       "coding_rate": 8,
       "preamble_len": 12,
       "output_power": 0,  # dBm
       "invert_iq_tx": False,
       "invert_iq_rx": True
    }
    
    return SX1276(
        spi=SPI(0, baudrate=2000_000, polarity=0, phase=0,
                miso=Pin(4), mosi=Pin(7), sck=Pin(6)),
        cs=Pin(5),
        dio0=Pin(28),
        dio1=Pin(10),
        reset=Pin(27),
        lora_cfg=lora_cfg,
    )


def main():
    print("Initializing...")
    modem = get_modem()
    modem._reg_write(0x39, 0x34)

    counter = 0
    while True:
        print("Sending...")
        modem.send(f"Hello world from MicroPython side a #{counter}".encode())

        print("Receiving...")
        modem._reg_write(0x9, 0xcf)
        rx = modem.recv(timeout_ms=5000)
        if rx:
            dumpCfg(modem)
            print(f"Received: {rx!r}")
        else:
            print("Timeout!")
        time.sleep(2)
        counter += 1


if __name__ == "__main__":
    main()
