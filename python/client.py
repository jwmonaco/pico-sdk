from time import sleep
import ulora

# Lora Parameters
RFM95_RST = 27
RFM95_SPIBUS = ulora.SPIConfig.rp2_0
RFM95_CS = 5
RFM95_INT = 28
RF95_FREQ = 902.3
RF95_POW = 20
CLIENT_ADDRESS = 1
SERVER_ADDRESS = 2

# initialise radio
lora = ulora.LoRa(RFM95_SPIBUS, RFM95_INT, CLIENT_ADDRESS, RFM95_CS, reset_pin=RFM95_RST, freq=RF95_FREQ, tx_power=RF95_POW, acks=True)


# loop and send data
while True:
    lora.send_to_wait("This is a test message", SERVER_ADDRESS)
    print("sent")

    ulora.dumpCfg(lora)
    
    sleep(10)
