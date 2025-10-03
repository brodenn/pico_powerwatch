# ================== Pico 2 W — Project IO + INA219 + Rotary (io_test UI) ==================
# Projektets pinout (oförändrat):
#   OLED via SPI0:  SCK=GP18, MOSI=GP19, MISO=None, CS=GP17, DC=GP20, RST=GP21
#   INA219 via I2C0: SDA=GP4,  SCL=GP5  (addr 0x40)
#   ROTARY:          A=GP14,   B=GP15,  SW=GP13 (aktiv låg)
#
# Displayen visar EXAKT samma text/layout som originalets io_test:
#   "I/O Test"
#   "Button GP18: ...", "EncBtn GP13: ...", "Encoder: N"
#   "Green GP16: ...", "Red GP17: ..."  (röd visas men styrs ej, pga CS på GP17)
#
# Rotary-kod: io_test-stil (IRQ på A, 3 ms spärr, läser B för riktning).
# MQTT: publicerar ENBART Volt till TOPIC_BASE var 500 ms (retain).

import time, network
from machine import Pin, SPI, I2C
import ssd1306
from umqtt.simple import MQTTClient

# ---------- Wi-Fi / MQTT ----------
SSID = "GoteborgPSK"
PASSWORD = "UfEQNec36A"
MQTT_SERVER = "100.82.0.4"
MQTT_PORT = 1883
MQTT_USER = "elektronik"
MQTT_PASSWORD = "elektronik"

TOPIC_BASE = b"pico17/sensor/1"   # publicerar float Volt (retain)
PUBLISH_MS = 500

# ---------- OLED (SPI0, projektets pinnar) ----------
spi = SPI(0, baudrate=10_000_000, polarity=0, phase=0,
          sck=Pin(18), mosi=Pin(19), miso=None)
dc, res, cs = Pin(20), Pin(21), Pin(17)
display = ssd1306.SSD1306_SPI(128, 64, spi, dc, res, cs)
display.contrast(255)

# ---------- INA219 ----------
class INA219:
    REG_CFG=0x00; REG_BUS=0x02; REG_CUR=0x04; REG_CAL=0x05
    def __init__(self, i2c, addr=0x40, shunt=0.1, maxA=1.0):
        self.i2c=i2c; self.addr=addr
        self.current_lsb = maxA/32767.0
        cal = int(0.04096/(self.current_lsb*shunt))
        self._w16(self.REG_CAL, cal)
        self._w16(self.REG_CFG, 0x019F)   # continuous shunt+bus, 12-bit
    def _r16(self, reg):
        self.i2c.writeto(self.addr, bytes([reg]))
        d=self.i2c.readfrom(self.addr, 2)
        return (d[0]<<8) | d[1]
    def _w16(self, reg, val):
        self.i2c.writeto(self.addr, bytes([reg, (val>>8)&0xFF, val&0xFF]))
    def voltage(self):
        raw=self._r16(self.REG_BUS)      # 13-bit (bits 15..3), LSB=4 mV
        return ((raw >> 3) * 4) / 1000.0
    def current(self):
        raw=self._r16(self.REG_CUR)
        if raw & 0x8000: raw -= 1<<16
        return raw * self.current_lsb

i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400_000)
try:
    ina = INA219(i2c, 0x40)
except Exception:
    ina = None  # kör vidare, visar 0.000 V

# ---------- IO (matchar io_test-funktionalitet) ----------
green_led = Pin(16, Pin.OUT)             # Green LED (OK, ledig)
# Red LED på GP17 kan ej användas här (CS för OLED). Vi emulerar bara status på displayen.
button   = Pin(18, Pin.IN, Pin.PULL_UP)  # Tactile (som i io_test)
enc_a    = Pin(14, Pin.IN, Pin.PULL_UP)  # Encoder A (projektets A)
enc_b    = Pin(15, Pin.IN, Pin.PULL_UP)  # Encoder B (projektets B)
enc_btn  = Pin(13, Pin.IN, Pin.PULL_UP)  # Encoder SW (projektet) — labeln visar GP13

# ---------- Rotary: io_test-stil ----------
encoder_value = 0
last_tick = 0
debounce_ms = 3

def encoder_callback(pin):
    global encoder_value, last_tick
    now = time.ticks_ms()
    if time.ticks_diff(now, last_tick) > debounce_ms:
        a_val = enc_a.value()
        b_val = enc_b.value()
        if a_val == 1:            # rising edge av A
            if b_val == 0:
                encoder_value += 1   # medurs
            else:
                encoder_value -= 1   # moturs
        last_tick = now

# IRQ på rising edge av A (som i io_test)
enc_a.irq(trigger=Pin.IRQ_RISING, handler=encoder_callback)

# ---------- Nät / MQTT ----------
def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        wlan.connect(SSID, PASSWORD)
        while not wlan.isconnected():
            time.sleep_ms(200)
    return wlan.ifconfig()[0]

def mqtt_connect():
    c = MQTTClient(
        client_id=b"pico2w",
        server=MQTT_SERVER,
        port=MQTT_PORT,
        user=MQTT_USER,
        password=MQTT_PASSWORD,
        keepalive=30,
    )
    c.connect()
    return c

def safe_publish(client, topic: bytes, payload: bytes, retain=False):
    try:
        client.publish(topic, payload, retain=retain)
        return True
    except Exception:
        try:
            client.disconnect()
        except:
            pass
        try:
            client = mqtt_connect()
            client.publish(topic, payload, retain=retain)
            return True
        except:
            return False

# ---------- Main ----------
def main():
    ip = connect_wifi()
    client = mqtt_connect()

    last_pub = time.ticks_ms()
    last_draw = 0

    while True:
        # --- Mät INA219 (eller 0.0 om ej ansluten) ---
        if ina:
            try:
                v = ina.voltage()
                i = ina.current()
            except Exception:
                v = 0.0; i = 0.0
        else:
            v = 0.0; i = 0.0
        p = v * i

        # --- LED-mönster (som original: gröna blinkar varje steg; röd "emuleras") ---
        green_led.value((encoder_value % 2) == 0)
        red_state = ((encoder_value % 4) < 2)   # visas bara på displayen

        # --- MQTT publish var PUBLISH_MS ---
        now = time.ticks_ms()
        if time.ticks_diff(now, last_pub) >= PUBLISH_MS:
            safe_publish(client, TOPIC_BASE, b"%.3f" % v, retain=True)
            last_pub = now

        # --- Display (EXAKT originalupplägg) ~20 Hz ---
        if time.ticks_diff(now, last_draw) >= 50:
            btn_state     = "ON " if button.value() == 0 else "OFF"
            enc_btn_state = "ON " if enc_btn.value() == 0 else "OFF"
            display.fill(0)
            display.text("I/O Test", 0, 0, 1)
            display.text(f"Button GP18: {btn_state}",        0, 14, 1)
            display.text(f"EncBtn GP13: {enc_btn_state}",    0, 24, 1)  # label uppdaterad till projektets SW-pin
            display.text(f"Encoder: {encoder_value}",         0, 34, 1)
            display.text(f"Green GP16: {'ON' if green_led.value() else 'OFF'}", 0, 44, 1)
            display.text(f"Red GP17:   {'ON' if red_state else 'OFF'}",         0, 54, 1)
            display.show()
            last_draw = now

        time.sleep_ms(10)

# Start
if __name__ == "__main__":
    main()
