import serial
import time

# --- 配置区 ---
# 如果你的上位机连接了 COM2，这里就填 'COM3'
# 如果你的上位机连接了 COM3，这里就填 'COM2'
TARGET_PORT = 'COM3' 
BAUD_RATE = 115200
TIMEOUT = 1

def start_serial_service():
    try:
        # 初始化串口
        ser = serial.Serial(TARGET_PORT, BAUD_RATE, timeout=TIMEOUT)
        print(f"--- 串口响应程序已启动 ---")
        print(f"当前监听端口: {TARGET_PORT}")
        print(f"请确保上位机已连接配对的另一个端口")
        print("等待接收指令... (按 Ctrl+C 退出)")

        while True:
            # 检查缓冲区是否有数据
            if ser.in_waiting > 0:
                # 读取接收到的所有字节
                raw_data = ser.read(ser.in_waiting)
                
                # 尝试以 UTF-8 解码显示，如果失败则显示 Hex
                try:
                    display_data = raw_data.decode('utf-8').strip()
                except UnicodeDecodeError:
                    display_data = raw_data.hex(' ')

                print(f"\n[收到指令]: {display_data}")

                # --- 自动回传逻辑 ---
                # 逻辑1：原样返回 (Echo)
                # response = raw_data 
                
                # 逻辑2：自定义回复字符串
                response_str = f"Server Received: {display_data}\n"
                ser.write(response_str.encode('utf-8'))
                
                print(f"[已回传]: {response_str.strip()}")

            time.sleep(0.1) # 降低CPU占用

    except serial.SerialException as e:
        print(f"无法打开串口 {TARGET_PORT}: {e}")
        print("请检查该端口是否被其他程序占用，或端口号是否正确。")
    except KeyboardInterrupt:
        print("\n程序已手动停止")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()
            print("串口已释放")

if __name__ == "__main__":
    start_serial_service()