# ROV (4‑Thruster)

## Layout
- 2 horizontal thrusters for surge + yaw.
- 2 vertical thrusters for heave.

## Channel Mapping
- `ch0`: surge (forward/back)
- `ch1`: yaw (turn)
- `ch2`: heave (up/down)

## Mixer Output
- Motor 0: horizontal left  = surge + yaw
- Motor 1: horizontal right = surge − yaw
- Motor 2: vertical front   = heave
- Motor 3: vertical rear    = heave

## UDP Packet Format
```cpp
struct UdpPacket {
  uint32_t magic;      // 0x4D465454 ("MFTT")
  uint8_t version;     // 1
  uint8_t channel_count;
  float channels[16];
};
```

## UDP Sender Example (Python)
```python
import socket
import struct

MAGIC = 0x4D465454
VERSION = 1

def send_frame(sock, addr, channels):
  channel_count = min(len(channels), 16)
  payload = struct.pack("<Ibb", MAGIC, VERSION, channel_count)
  payload += struct.pack("<16f", *(channels + [0.0] * (16 - channel_count)))
  sock.sendto(payload, addr)

if __name__ == "__main__":
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  addr = ("127.0.0.1", 14550)
  send_frame(sock, addr, [0.2, 0.1, -0.3])
```

## Biheli PWM
Normalized command range is `[-1, 1]` mapped to `1100us..1900us` with neutral at `1500us`.
