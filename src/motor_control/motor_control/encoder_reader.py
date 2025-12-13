import rclpy
from rclpy.node import Node
import serial
import struct

class EncoderReader(Node):
    def __init__(self):
        super().__init__("encoder_reader")

        # Serial port
        self.ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.01)

        self.buffer = []
        self.packet_len = 14  # 2A + 4 bytes ID + 8 bytes data + 23

        self.timer = self.create_timer(0.01, self.read_serial)  # 100 Hz

    def read_serial(self):
        while self.ser.in_waiting:
            byte = self.ser.read(1)[0]

            # Wait for start byte
            if len(self.buffer) == 0:
                if byte == 0x2A:
                    self.buffer.append(byte)
                continue

            self.buffer.append(byte)

            # Nếu đã đủ 14 byte → xử lý
            if len(self.buffer) == self.packet_len:

                if self.buffer[-1] == 0x23:

                    # ID = 4 bytes (little-endian)
                    id_bytes = bytes(self.buffer[1:5])
                    frame_id = struct.unpack("<I", id_bytes)[0]

                    # Decoder 8 byte encoder
                    d = self.buffer[5:13]

                    tickLF = struct.unpack(">h", bytes(d[0:2]))[0]
                    tickRF = struct.unpack(">h", bytes(d[2:4]))[0]
                    tickLB = struct.unpack(">h", bytes(d[4:6]))[0]
                    tickRB = struct.unpack(">h", bytes(d[6:8]))[0]

                    self.get_logger().info(
                        f"ID={frame_id}  Encoders [LF={tickLF}, RF={tickRF}, LB={tickLB}, RB={tickRB}]"
                    )

                # Reset buffer
                self.buffer = []


def main(args=None):
    rclpy.init(args=args)
    node = EncoderReader()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
