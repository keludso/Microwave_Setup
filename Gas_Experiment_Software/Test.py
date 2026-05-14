import u12
d = u12.U12()
for channel in range(3):
    reading = d.eAnalogIn(channel)
    print(f"AI{channel}: {reading['voltage']} V")