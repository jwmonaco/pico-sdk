from time import sleep
import ulora
import ntptime
import mm_wlan
import time
import json
from umqtt import MQTTClient
from machine import Timer


sendFifo = []

def sendNextOnFifo(arg):
    global sendFifo
    message = sendFifo.pop(0)
    lora.sendMessage(message)


def mqtt_sub_callback(topic, message):
    global sendFifo
    now = time.ticks_ms()
    elapsed = now - lora.lastRx
    jsonMessage = json.loads(message.decode('utf-8'))
    for item in jsonMessage['items']:
        sendFifo.append(item)
        delay = int(item['txInfo']['timing']['delay']['delay'][0])
        Timer(mode=Timer.ONE_SHOT, period=delay*1000 - elapsed, callback=sendNextOnFifo)
    #print(f'Topic {topic} received after {elapsed/1000}s: {jsonMessage}')

# This is our callback function that runs when a message is received
def on_recv(payload):
    print("From:", payload.header_from)
    print("Received:", payload.message)
    print("RSSI: {}; SNR: {}".format(payload.rssi, payload.snr))

# Lora Parameters
RFM95_RST = 27
RFM95_SPIBUS = ulora.SPIConfig.rp2_0
RFM95_CS = 5
RFM95_INT = 28
RF95_FREQ = 923.3
RF95_POW = 20
CLIENT_ADDRESS = 1
SERVER_ADDRESS = 2
RF95CFG = ulora.ModemConfig.gateway

# Connect to network and initialize ntp
mm_wlan.connect_to_network('JWMFIBER', 'gopack00')
print("Trying to set time via NTP")
while True:
    try:
        ntptime.settime()
        print(f"Current time: {ulora.getFormattedTime(ulora.rtc.datetime())}")
        break
    except Exception as e:
        print(f"NTP failed with {e}")


# initialise radio
lora = ulora.LoRa(RFM95_SPIBUS, RFM95_INT, SERVER_ADDRESS, RFM95_CS, reset_pin=RFM95_RST, freq=RF95_FREQ, tx_power=RF95_POW, acks=True, modem_config=RF95CFG, inverted=True)

# set callback
lora.on_recv = on_recv

# set to listen continuous
lora.set_mode_rx()

ulora.dumpCfg(lora)

# loop and wait for data
mqtt_client = MQTTClient(server='192.168.87.57',
                         client_id='picoGateway')
mqtt_client.set_callback(mqtt_sub_callback)
mqtt_client.connect()
mqtt_client.subscribe('us915_0/gateway/+/command/down')

while True:
#    lora.process()
    mqtt_client.wait_msg()
    sleep(0.1)
