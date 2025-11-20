import serial
import struct
import time

# 配置串口参数
SERIAL_PORT = '/dev/ttyUSB1'  # 请修改为你的实际串口号 (Windows: COMx, Linux: /dev/ttyUSBx)
BAUD_RATE = 115200  # 确保与STM32设置一致


def parse_servo_data(payload):
    print("-" * 40)
    print(f"收到数据包 (Payload长度: {len(payload)})")

    # 确保负载长度正确 (9个舵机 * 3字节 = 27字节)
    if len(payload) != 27:
        print("数据长度错误！")
        return

    # 遍历9个舵机的数据
    for i in range(9):
        base_idx = i * 3
        # 提取ID (1个字节)
        servo_id = payload[base_idx]
        # 提取位置 (2个字节，小端模式 '<H')
        # payload[base_idx+1 : base_idx+3] 取出位置的高低字节
        servo_pos = struct.unpack('<H', payload[base_idx + 1: base_idx + 3])[0]

        print(f"舵机 ID: {servo_id} | 位置: {servo_pos}")
    print("-" * 40)


def main():
    try:
        # 打开串口
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"串口 {SERIAL_PORT} 已打开，等待数据...")

        buffer = b''  # 用于缓存接收到的数据流

        while True:
            # 读取串口缓存中的所有数据
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                buffer += chunk

            # --- 协议解析状态机 ---

            # 1. 寻找帧头 0xA8
            # 如果缓冲区里甚至没有由头+功能+长度组成的最小字节数(4字节)，就继续读
            while len(buffer) >= 5:

                # 检查第一个字节是否是帧头
                if buffer[0] != 0xA8:
                    # 不是帧头，丢弃第一个字节，左移缓冲区
                    buffer = buffer[1:]
                    continue

                # 2. 解析数据长度 (第2、3字节是Length)
                # buffer[0]=Head, buffer[1]=Func, buffer[2]=LenLow, buffer[3]=LenHigh
                # 解析长度 (小端)
                data_len = buffer[2] + (buffer[3] << 8)

                # 计算整帧需要的长度 = Head(1) + Func(1) + Len(2) + Data(data_len) + Sum(1)
                frame_overhead = 5
                total_frame_len = frame_overhead + data_len

                # 3. 检查缓冲区数据是否足够构成一整帧
                if len(buffer) < total_frame_len:
                    # 数据不够，跳出内层循环，继续等待串口接收更多数据
                    break

                # 4. 提取完整的一帧数据
                current_frame = buffer[:total_frame_len]

                # 5. 校验和验证
                # 规则：累加 (Head ... DataLastByte) & 0xFF == Checksum
                received_sum = current_frame[-1]  # 最后一个字节
                calc_sum = sum(current_frame[:-1]) & 0xFF  # 计算除了最后字节之外的所有和

                if calc_sum == received_sum:
                    # --- 校验通过，进行解析 ---
                    func_code = current_frame[1]
                    payload = current_frame[4:-1]  # 数据段：从索引4开始，到倒数第2个字节

                    if func_code == 0x81:  # 假设 STM32 发送的功能码是 0x81
                        parse_servo_data(payload)
                    else:
                        print(f"未知功能码: {func_code:02X}")
                else:
                    print(f"校验失败! 计算值: {calc_sum:02X}, 接收值: {received_sum:02X}")

                # 6. 处理完毕，从缓冲区移除这一帧数据
                buffer = buffer[total_frame_len:]

            # 稍微休息一下，避免CPU占用过高
            time.sleep(1)

    except serial.SerialException as e:
        print(f"串口错误: {e}")
    except KeyboardInterrupt:
        if ser.is_open:
            ser.close()
        print("\n程序已停止，串口关闭。")


if __name__ == '__main__':
    main()