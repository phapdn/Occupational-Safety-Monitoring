# -*- coding: utf-8 -*-
import lgpio
import spidev
import struct
import time
import json
import ssl
from datetime import datetime
import paho.mqtt.client as mqtt

# ==== CAU HINH HIVEMQ CLOUD ====
MQTT_BROKER = "d0a82f39864c4e86a0551feaed97f7c5.s1.eu.hivemq.cloud"
MQTT_PORT = 8883
MQTT_USERNAME = "truong123"
MQTT_PASSWORD = "Truong123"
MQTT_TOPIC_BASE = "helmet"

# ==== MQTT SETUP ====
client = mqtt.Client()
client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
client.tls_set(cert_reqs=ssl.CERT_NONE)
client.tls_insecure_set(True)

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    print("Ket noi HiveMQ Cloud thanh cong toi {}:{}".format(MQTT_BROKER, MQTT_PORT))
    print("")
except Exception as e:
    print("Loi ket noi HiveMQ Cloud: {}".format(e))
    exit(1)

# ==== GPIO ====
CHIP = lgpio.gpiochip_open(0)
PIN_RST = 17
lgpio.gpio_claim_output(CHIP, PIN_RST)
lgpio.gpio_write(CHIP, PIN_RST, 1)

# ==== SPI ====
spi = spidev.SpiDev()
spi.open(0, 0)
spi.max_speed_hz = 5000000
spi.mode = 0

# ==== Thanh ghi SX1278 ====
REG_FIFO = 0x00
REG_OP_MODE = 0x01
REG_FIFO_ADDR_PTR = 0x0D
REG_FIFO_RX_BASE_ADDR = 0x0F
REG_FIFO_RX_CURRENT_ADDR = 0x10
REG_IRQ_FLAGS = 0x12
REG_RX_NB_BYTES = 0x13
REG_MODEM_CONFIG_1 = 0x1D
REG_MODEM_CONFIG_2 = 0x1E
REG_MODEM_CONFIG_3 = 0x26
REG_SYNC_WORD = 0x39
REG_FREQ_MSB = 0x06
REG_FREQ_MID = 0x07
REG_FREQ_LSB = 0x08

# ==== SPI utils ====
def write_reg(addr, val):
    spi.xfer2([addr | 0x80, val])

def read_reg(addr):
    return spi.xfer2([addr & 0x7F, 0x00])[1]

# ==== Tan so 433 MHz (nhan truc tiep tu Tag) ====
def set_frequency(freq=433E6):
    frf = int((freq / 32000000) * (1 << 19))
    spi.xfer2([REG_FREQ_MSB | 0x80, (frf >> 16) & 0xFF])
    spi.xfer2([REG_FREQ_MID | 0x80, (frf >> 8) & 0xFF])
    spi.xfer2([REG_FREQ_LSB | 0x80, frf & 0xFF])

# ==== Khoi tao SX1278 ====
def init_lora():
    write_reg(REG_OP_MODE, 0x80)
    time.sleep(0.05)
    set_frequency(433E6)
    write_reg(REG_MODEM_CONFIG_1, 0x72)  # BW=125kHz, CR=4/5
    write_reg(REG_MODEM_CONFIG_2, 0x94)  # SF=9
    write_reg(REG_MODEM_CONFIG_3, 0x04)
    write_reg(REG_SYNC_WORD, 0x12)  # SyncWord=0x12 (nhan truc tiep tu Tag)
    write_reg(REG_FIFO_RX_BASE_ADDR, 0x00)
    write_reg(REG_IRQ_FLAGS, 0xFF)  # Clear IRQ
    write_reg(REG_OP_MODE, 0x85)  # RX Continuous mode
    
    print("="*80)
    print("GATEWAY RASPBERRY PI - NHAN TRUC TIEP TU TAG")
    print("="*80)
    print("Frequency  : 433 MHz (nhan truc tiep tu Tag)")
    print("Bandwidth  : 125 kHz")
    print("SF         : 9")
    print("SyncWord   : 0x12")
    print("Packet Size: 74 bytes (Full_System_DualCore structure)")
    print("TAG -> Gateway (BO QUA RELAY)")
    print("="*80)
    print("")

# ==== Doc goi LoRa ====
def read_packet():
    irq = read_reg(REG_IRQ_FLAGS)
    if irq & 0x40:
        write_reg(REG_IRQ_FLAGS, 0xFF)
        nb = read_reg(REG_RX_NB_BYTES)
        spi.xfer2([REG_FIFO_ADDR_PTR | 0x80, read_reg(REG_FIFO_RX_CURRENT_ADDR)])
        data = [read_reg(REG_FIFO) for _ in range(nb)]
        return bytes(data)
    return None

# ==== Giai ma packet (74 bytes - Full_System_DualCore structure) ====
def parse_packet(raw):
    # DataPacket ESP32: mac(8) + bodyTemp(4) + busVoltage(4) + current_mA(4) + batteryPercent(4) +
    #                   latitude(8) + longitude(8) + heartRate(4) + spo2(4) +
    #                   uwb_A0(4) + uwb_A1(4) + uwb_A2(4) + uwb_A3(4) + uwb_Tag3(4) + uwb_Tag4(4) +
    #                   fallDetected(1) + helpRequest(1) = 74 bytes
    # Format: Q=uint64, f=float(4), d=double(8), B=uint8
    if len(raw) < 74:
        return None
    
    try:
        # DUNG: mac(Q) + 4 float + 2 double + 2 float + 6 float + 2 uint8
        fields = struct.unpack('<QffffddffffffffBB', raw[:74])
        return {
            "mac": fields[0],
            "bodyTemp": fields[1],
            "busVoltage": fields[2],
            "current_mA": fields[3],
            "batteryPercent": fields[4],
            "latitude": fields[5],
            "longitude": fields[6],
            "heartRate": fields[7],
            "spo2": fields[8],
            "uwb_A0": fields[9],
            "uwb_A1": fields[10],
            "uwb_A2": fields[11],
            "uwb_A3": fields[12],
            "uwb_Tag3": fields[13],
            "uwb_Tag4": fields[14],
            "fallDetected": fields[15],
            "helpRequest": fields[16]
        }
    except Exception as e:
        print("    >>> Parse error: {}".format(e))
        return None

# ==== Quan ly node ====
devices = {}

def update_device(data):
    mac = data["mac"]
    now = time.time()
    devices[mac] = {
        "last_seen": now,
        **data
    }

def count_online_nodes(timeout=5):
    now = time.time()
    return [mac for mac, info in devices.items() if now - info["last_seen"] < timeout]

# ==== MAIN LOOP ====
init_lora()
print("Dang lang nghe LoRa truc tiep tu Tag...")
print("")

packet_count = 0

while True:
    pkt = read_packet()
    if pkt:
        packet_count += 1
        print(">>> Nhan duoc packet #{}, size: {} bytes".format(packet_count, len(pkt)))
        
        data = parse_packet(pkt)
        if data:
            update_device(data)
            mac_hex = "{:012X}".format(data['mac'])
            
            # In ra console day du
            print("="*80)
            print("[{}] MAC: {}".format(datetime.now().strftime("%H:%M:%S"), mac_hex))
            print("GPS       : Lat={:.6f}, Lon={:.6f}".format(data['latitude'], data['longitude']))
            print("Body Temp : {:.2f}C".format(data['bodyTemp']))
            print("Battery   : Voltage={:.2f}V, Current={:.0f}mA, Percent={:.0f}%".format(
                data['busVoltage'], data['current_mA'], data['batteryPercent']))
            print("Heart Rate: HR={:.0f} BPM, SpO2={:.0f}%".format(data['heartRate'], data['spo2']))
            print("UWB       : A0={:.2f}m, A1={:.2f}m, A2={:.2f}m, A3={:.2f}m, Tag3={:.2f}m, Tag4={:.2f}m".format(
                data['uwb_A0'], data['uwb_A1'], data['uwb_A2'], data['uwb_A3'], data['uwb_Tag3'], data['uwb_Tag4']))
            print("Fall      : {}".format("YES - DETECTED!" if data['fallDetected'] == 1 else "No"))
            print("Help Btn  : {}".format("PRESSED!" if data['helpRequest'] == 1 else "Not pressed"))
            print("="*80)
            print("")
            
            # Gui du lieu len MQTT
            payload = {
                "mac": mac_hex,
                "temp": round(data["bodyTemp"], 2),
                "voltage": round(data["busVoltage"], 2),
                "current": round(data["current_mA"], 1),
                "battery": round(data["batteryPercent"], 1),
                "lat": round(data["latitude"], 6),
                "lon": round(data["longitude"], 6),
                "hr": round(data["heartRate"], 0),
                "spo2": round(data["spo2"], 0),
                "uwb": {
                    "A0": round(data["uwb_A0"], 2),
                    "A1": round(data["uwb_A1"], 2),
                    "A2": round(data["uwb_A2"], 2),
                    "A3": round(data["uwb_A3"], 2),
                    "Tag3": round(data["uwb_Tag3"], 2),
                    "Tag4": round(data["uwb_Tag4"], 2)
                },
                "fallDetected": data["fallDetected"],
                "helpRequest": data["helpRequest"],
                "timestamp": datetime.now().isoformat()
            }
            
            topic = "{}/{}".format(MQTT_TOPIC_BASE, mac_hex)
            client.publish(topic, json.dumps(payload))
        else:
            print("    >>> Parse FAILED - Invalid packet structure")
    
    time.sleep(0.05)
