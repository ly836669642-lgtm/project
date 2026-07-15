import socket
import struct

def decode(data):
    if len(data) == 16:
        values = struct.unpack("ffff", data)
        return values
    else:
        return None

port = 6657
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", port))

print("Listening on UDP port " + str(port) + "...")

while True:
    data, addr = sock.recvfrom(1024)
    values = decode(data = data)
    if values is not None:
        print("Address: " + str(addr))
        print(f"Control values: throttle={values[0]:.3f}, steering={values[1]:.3f}, brake={values[2]:.3f}, dummy={values[3]:.3f}")
