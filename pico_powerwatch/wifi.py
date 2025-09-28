import network, time, mip

# ======= Ändra till ditt nätverk =======
SSID = "goteborgfree"
PASSWORD = ""   # om nätet är öppet, lämna tomt

# ======= Anslut WiFi =======
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(SSID, PASSWORD)

print("Försöker ansluta till:", SSID)
while not wlan.isconnected():
    print("  ... ansluter")
    time.sleep(1)

print("\n✅ Ansluten!\n")
print("IP-adress   :", wlan.ifconfig()[0])
print("Subnätmask  :", wlan.ifconfig()[1])
print("Gateway     :", wlan.ifconfig()[2])
print("DNS-server  :", wlan.ifconfig()[3])

print("\nNu kan du installera bibliotek med mip, t.ex.:")
mip.install('github:micropython/micropython/drivers/display/ssd1306.py')
