"""sensor_msgs/Image (rgb8 or bgr8) -> PNG bytes without OpenCV or Pillow. Downscaled by an integer factor."""
import struct
import zlib


def image_to_png(msg, max_width=320):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f'unsupported encoding {msg.encoding}')
    step = max(1, msg.width // max_width)
    w, h = msg.width // step, msg.height // step
    data = bytes(msg.data)
    swap = msg.encoding == 'bgr8'
    rows = bytearray()
    for y in range(h):
        base = y * step * msg.step
        rows.append(0)  # filter: none
        for x in range(w):
            i = base + x * step * 3
            px = data[i:i + 3]
            rows += (px[2:3] + px[1:2] + px[0:1]) if swap else px
    def chunk(tag, body):
        return struct.pack('>I', len(body)) + tag + body + struct.pack('>I', zlib.crc32(tag + body) & 0xffffffff)
    ihdr = struct.pack('>IIBBBBB', w, h, 8, 2, 0, 0, 0)
    return b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', ihdr) + chunk(b'IDAT', zlib.compress(bytes(rows), 6)) + chunk(b'IEND', b''), w, h
