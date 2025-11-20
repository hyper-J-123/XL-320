import serial
import struct

def send_servo_command(ser):
    # [(id, position), (id, position), ...]
    servos = [(1, 500), (2, 500), (3, 500), (4, 500), (5, 500), (6, 500), (7, 500), (8, 500), (9, 500)]
    # servos = []
    # for i in range(1, 10):
    #     servos.append((i, 700))  # ID=i, Position=512
    # 每个舵机占3字节：1字节ID + 2字节位置
    payload = b''
    for servo_id, pos in servos:
        # struct.pack格式: 'B'是unsigned char(1字节), 'H'是unsigned short(2字节)
        # '<' 代表小端模式
        payload += struct.pack('<BH', servo_id, pos)

    frame_head = 0xA8
    func_code = 0x01
    data_len = len(payload)

    # Head(1) + Func(1) + Len(2, 小端) + Payload(27)
    packet_no_sum = struct.pack('B', frame_head) + \
                    struct.pack('B', func_code) + \
                    struct.pack('<H', data_len) + \
                    payload

    checksum = sum(packet_no_sum) & 0xFF
    final_packet = packet_no_sum + struct.pack('B', checksum)

    print(f"发送的数据长度: {len(final_packet)} bytes")
    print("Hex:", final_packet.hex().upper())
    ser.write(final_packet)

if __name__ == '__main__':
    try:
        ser = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        send_servo_command(ser)
        ser.close()
        print("发送成功")
    except Exception as e:
        print(f"串口错误: {e}")