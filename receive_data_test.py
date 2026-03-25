import socket
import struct

fmt = ">Q i I I H"

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5005))

print("Listening on UDP 5005...")

while True:
    data, addr = sock.recvfrom(1024)

    if len(data) != struct.calcsize(fmt):
        print("Unexpected size:", len(data))
        continue

    ts, temp, hum, press, crc = struct.unpack(fmt, data)

    print("Timestamp:", ts)
    print("Temp (C):", temp / 100.0)
    print("Humidity:", hum)
    print("Pressure:", press)
    print("------")