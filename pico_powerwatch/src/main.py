# Pico 2 W + SSD1309 (SPI) + INA219 (I2C) + encoder/knapp + enkel meny
# RAW encoder (no debounce at all) + single-schedule drain (prevents "schedule queue full")
# Robust MQTT. Bas-topic pico17/sensor/1 (Volt, retain) + separata topics.

import uasyncio as aio
from machine import Pin, SPI, I2C
import machine
import time, micropython
from micropython import const

micropython.alloc_emergency_exception_buf(128)

# ---------- Wi-Fi / MQTT ----------
WIFI_SSID   = "GoteborgPSK"
WIFI_PASS   = "UfEQNec36A"

MQTT_BROKER   = "100.82.0.4"
MQTT_PORT     = 1883
MQTT_USER     = "elektronik"
MQTT_PASSWORD = "elektronik"

# Topics (bas + derivat)
TOPIC_BASE = b"pico17/sensor/1"    # Dashboard läser denna (ren float)
TOPIC_SUB  = TOPIC_BASE + b"/cmd"

TOPIC_V    = TOPIC_BASE + b"/voltage"
TOPIC_I    = TOPIC_BASE + b"/current"
TOPIC_P    = TOPIC_BASE + b"/power"
TOPIC_WH   = TOPIC_BASE + b"/wh"
TOPIC_MAH  = TOPIC_BASE + b"/mah"
TOPIC_T    = TOPIC_BASE + b"/t"
TOPIC_PAGE = TOPIC_BASE + b"/page"
TOPIC_ON   = TOPIC_BASE + b"/online"

PUBLISH_MS = 500   # skicka var 0.5 s

# ---------- Pinnar ----------
SCK, MOSI, CS = 18, 19, 17
DC,  RST      = 20, 21
SDA, SCL      = 4, 5
ENC_A, ENC_B, BTN = 14, 15, 13

# ---------- OLED ----------
spi = SPI(0, baudrate=10_000_000, polarity=0, phase=0,
          sck=Pin(SCK), mosi=Pin(MOSI), miso=None)
dc  = Pin(DC,  Pin.OUT); rst = Pin(RST, Pin.OUT); cs = Pin(CS, Pin.OUT)
import ssd1306
oled = ssd1306.SSD1306_SPI(128, 64, spi, dc, rst, cs)

# ---------- INA219 (minimal) ----------
class INA219:
    REG_CFG=0x00; REG_BUS=0x02; REG_CUR=0x04; REG_CAL=0x05
    def __init__(self, i2c, addr=0x40, shunt=0.1, maxA=1.0):
        self.i2c=i2c; self.addr=addr
        self.current_lsb = maxA/32767.0
        cal = int(0.04096/(self.current_lsb*shunt))
        self._w16(self.REG_CAL, cal)
        self._w16(self.REG_CFG, 0x019F)   # continuous shunt+bus, 12-bit
    def _r16(self, reg):
        self.i2c.writeto(self.addr, bytes([reg])); d=self.i2c.readfrom(self.addr,2)
        return (d[0]<<8)|d[1]
    def _w16(self, reg, val):
        self.i2c.writeto(self.addr, bytes([reg,(val>>8)&0xFF,val&0xFF]))
    def voltage(self):
        raw=self._r16(self.REG_BUS); return ((raw>>3)*4)/1000.0
    def current(self):
        raw=self._r16(self.REG_CUR)
        if raw & 0x8000: raw -= 1<<16
        return raw*self.current_lsb

i2c = I2C(0, sda=Pin(SDA), scl=Pin(SCL), freq=400_000)
ina = INA219(i2c, 0x40)

# ---------- RAW Rotary (NO DEBOUNCE) ----------
class RotaryRaw:
    """No debouncing. Every edge is processed. Uses single scheduled drain to avoid queue overflow."""
    ROT_CW      = const(1)
    ROT_CCW     = const(2)
    SW_PRESS    = const(4)
    SW_RELEASE  = const(8)

    _CW  = (0b0001,0b0111,0b1110,0b1000)
    _CCW = (0b0010,0b0100,0b1101,0b1011)

    def __init__(self, a, b, sw, pull=Pin.PULL_UP):
        self.a = Pin(a, Pin.IN, pull)
        self.b = Pin(b, Pin.IN, pull)
        self.sw = Pin(sw, Pin.IN, pull)

        self._last_ab = (self.a.value()<<1) | self.b.value()
        self._last_sw = self.sw.value()

        self._steps = 0            # +N/-N (no filtering)
        self._btn_evt = 0          # PRESS/RELEASE (raw)
        self._scheduled = False
        self._handlers = []

        # both edges, no debounce:
        self.a.irq(self._rot_isr, Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.b.irq(self._rot_isr, Pin.IRQ_FALLING | Pin.IRQ_RISING)
        self.sw.irq(self._sw_isr,  Pin.IRQ_FALLING | Pin.IRQ_RISING)

    def add_handler(self, fn): self._handlers.append(fn)

    def _try_schedule(self):
        if not self._scheduled:
            self._scheduled = True
            try:
                micropython.schedule(self._drain, 0)
            except RuntimeError:
                self._scheduled = False  # queue full → try again on next IRQ

    def _rot_isr(self, _pin):
        ab = (self.a.value()<<1) | self.b.value()
        if ab != self._last_ab:
            tr = (self._last_ab<<2) | ab
            self._last_ab = ab
            if tr in self._CW:
                self._steps += 1
                self._try_schedule()
            elif tr in self._CCW:
                self._steps -= 1
                self._try_schedule()

    def _sw_isr(self, _pin):
        v = self.sw.value()
        if v != self._last_sw:
            self._last_sw = v
            self._btn_evt = self.SW_RELEASE if v else self.SW_PRESS
            self._try_schedule()

    def _drain(self, _):
        state = machine.disable_irq()
        steps = self._steps; self._steps = 0
        btn   = self._btn_evt; self._btn_evt = 0
        self._scheduled = False
        machine.enable_irq(state)

        if steps > 0:
            for _ in range(steps):
                for fn in self._handlers:
                    try: fn(self.ROT_CW)
                    except: pass
        elif steps < 0:
            for _ in range(-steps):
                for fn in self._handlers:
                    try: fn(self.ROT_CCW)
                    except: pass

        if btn:
            for fn in self._handlers:
                try: fn(btn)
                except: pass

rot = RotaryRaw(ENC_A, ENC_B, BTN)

# ---------- Trip ----------
ws=0.0; As=0.0; t0=time.ticks_ms()
def trip_reset():
    global ws, As, t0
    ws=0.0; As=0.0; t0=time.ticks_ms()
def trip_update(v,i,dt):
    global ws, As
    ws += (v*i)*dt
    As += i*dt
def trip_wh():  return ws/3600.0
def trip_mah(): return (As/3600.0)*1000.0
def trip_elapsed():
    return time.ticks_diff(time.ticks_ms(), t0)/1000.0

# ---------- UI ----------
PAGE_LIVE, PAGE_TRIP, PAGE_MENU = 0,1,2
page = PAGE_LIVE
menu = ["Live", "Trip", "Reset Trip"]
menu_idx=0

def clamp(x,a,b): return a if x<a else b if x>b else x
def fmt_hms(sec):
    s=max(0,int(sec)); h=s//3600; s%=3600; m=s//60; s%=60
    return "%02d:%02d:%02d"%(h,m,s)

def draw_live(v,i,p):
    oled.fill(0)
    oled.text("LIVE",0,0,1)
    oled.text("V:{:.3f} I:{:.3f}".format(v,i),0,14,1)
    oled.text("P:{:.3f}W".format(p),0,26,1)
    oled.text("Press: Menu",0,54,1)
    oled.show()

def draw_trip():
    oled.fill(0)
    oled.text("TRIP",0,0,1)
    oled.text("Wh : {:.3f}".format(trip_wh()),0,14,1)
    oled.text("mAh: {:.1f}".format(trip_mah()),0,26,1)
    oled.text("t  : {}".format(fmt_hms(trip_elapsed())),0,38,1)
    oled.text("Long: Reset",0,54,1)
    oled.show()

def draw_menu():
    oled.fill(0)
    oled.text("MENU",0,0,1)
    for i,title in enumerate(menu):
        oled.text((">" if i==menu_idx else " ")+title,0,14+i*12,1)
    oled.text("Press: Select",0,54,1)
    oled.show()

def do_menu(i):
    global page
    if i==0: page=PAGE_LIVE
    elif i==1: page=PAGE_TRIP
    elif i==2:
        trip_reset()
        oled.fill(0); oled.text("Trip reset!",16,28,1); oled.show()
        time.sleep_ms(500); page=PAGE_TRIP

# ----- Rotary event handler -----
_press_t = 0
def rotary_event(code):
    global menu_idx, page, _press_t
    if code == RotaryRaw.ROT_CW:
        if page == PAGE_MENU:
            menu_idx = clamp(menu_idx + 1, 0, len(menu)-1)
    elif code == RotaryRaw.ROT_CCW:
        if page == PAGE_MENU:
            menu_idx = clamp(menu_idx - 1, 0, len(menu)-1)
    elif code == RotaryRaw.SW_PRESS:
        _press_t = time.ticks_ms()
    elif code == RotaryRaw.SW_RELEASE:
        dur = time.ticks_diff(time.ticks_ms(), _press_t)
        if page == PAGE_TRIP and dur > 700:
            trip_reset()
        else:
            if page == PAGE_MENU:
                do_menu(menu_idx)
            else:
                page = PAGE_MENU

rot.add_handler(rotary_event)

# ---------- State ----------
v=i=p=0.0
tprev = time.ticks_ms()

# ---------- Tasks ----------
async def task_measure(ms=100):
    global v,i,p,tprev
    while True:
        now=time.ticks_ms()
        dt=max(0.001, time.ticks_diff(now,tprev)/1000.0)
        tprev=now
        v = ina.voltage()
        i = ina.current()
        p = v*i
        trip_update(v,i,dt)
        await aio.sleep_ms(ms)

async def task_oled(ms=100):
    while True:
        if page==PAGE_LIVE: draw_live(v,i,p)
        elif page==PAGE_TRIP: draw_trip()
        else: draw_menu()
        await aio.sleep_ms(ms)

# ---------- MQTT (robust, auto-reconnect + ping) ----------
async def task_mqtt():
    import network
    from umqtt.simple import MQTTClient

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)

    def wifi_ok(): return wlan.isconnected()

    async def ensure_wifi():
        if wifi_ok(): return
        try: wlan.connect(WIFI_SSID, WIFI_PASS)
        except: pass
        t0 = time.ticks_ms()
        while not wifi_ok() and time.ticks_diff(time.ticks_ms(), t0) < 15000:
            await aio.sleep_ms(200)

    client = None
    last_pub = 0
    last_ping = 0
    backoff_ms = 500  # ökar upp till 5 s

    def on_msg(topic, msg):
        try:
            s = msg.decode().strip().lower()
            global page
            if s == "menu": page = PAGE_MENU
            elif s == "live": page = PAGE_LIVE
            elif s == "trip": page = PAGE_TRIP
            elif s == "reset": trip_reset()
        except: pass

    while True:
        await ensure_wifi()
        if not wlan.isconnected():
            await aio.sleep_ms(backoff_ms)
            backoff_ms = min(backoff_ms * 2, 5000)
            continue

        if client is None:
            try:
                client = MQTTClient(
                    client_id="pico2w",
                    server=MQTT_BROKER, port=MQTT_PORT,
                    user=MQTT_USER, password=MQTT_PASSWORD,
                    keepalive=30,
                )
                try: client.set_last_will(TOPIC_ON, b"0", retain=True)
                except: pass
                client.connect()
                try: client.publish(TOPIC_ON, b"1", retain=True)
                except: pass
                try:
                    client.set_callback(on_msg)
                    client.subscribe(TOPIC_SUB)
                except: pass
                backoff_ms = 500
                last_ping = time.ticks_ms()
                last_pub  = time.ticks_ms()
            except:
                client = None
                await aio.sleep_ms(backoff_ms)
                backoff_ms = min(backoff_ms * 2, 5000)
                continue

        try:
            client.check_msg()
        except:
            try: client.disconnect()
            except: pass
            client = None
            continue

        now = time.ticks_ms()

        if time.ticks_diff(now, last_ping) >= 10000:
            try: client.ping()
            except:
                try: client.disconnect()
                except: pass
                client = None
                continue
            last_ping = now

        if time.ticks_diff(now, last_pub) >= PUBLISH_MS:
            try:
                client.publish(TOPIC_BASE, b"%.3f" % v, retain=True)
                client.publish(TOPIC_V,   b"%.3f" % v)
                client.publish(TOPIC_I,   b"%.4f" % i)
                client.publish(TOPIC_P,   b"%.3f" % (v*i))
                client.publish(TOPIC_WH,  b"%.5f" % trip_wh())
                client.publish(TOPIC_MAH, b"%.1f"  % trip_mah())
                client.publish(TOPIC_T,   str(int(trip_elapsed())).encode())
                client.publish(TOPIC_PAGE,str(int(page)).encode())
            except:
                try: client.disconnect()
                except: pass
                client = None
                continue
            last_pub = now

        await aio.sleep_ms(50)

# ---------- Main ----------
async def main():
    oled.fill(0); oled.text("Boot...",0,28,1); oled.show()
    await aio.sleep_ms(300)
    aio.create_task(task_measure(100))
    aio.create_task(task_oled(100))
    aio.create_task(task_mqtt())
    while True:
        await aio.sleep(1)

aio.run(main())
