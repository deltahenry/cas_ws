from pymodbus.client import ModbusTcpClient
import time
import sys
# 設定從端設備的 IP 和埠號
slave_ip = '192.168.1.10'  # 換成你的從端 IP
slave_port = 502            # 默認 Modbus TCP 埠

# 建立 Modbus TCP 客戶端（Master）
client = ModbusTcpClient(slave_ip, port=slave_port)
def checkservo():
    connection = client.connect()
    response = client.read_holding_registers(address=8, count=2, slave=2)

    if not response.isError():
        print("讀取到的寄存器值:", response.registers)
    else:
        print("讀取錯誤:", response)
    if response.registers==[0,0]:
        client.close()
        return 1
        
    else:
        client.close()
        return 0
    
def Jointsethome(n):
    joint_addr=[0,0x1210,0x2210,0x3210]
    connection = client.connect()
    if connection:
        print("連接成功")
        values_to_write = [271]  # 要寫入的值

        # 發送寫入多寄存器指令（0x10）
        result = client.write_registers(address=joint_addr[n], values=values_to_write, slave=251)
        if not result.isError():
            print("寫入成功")
        else:
            print("寫入錯誤:", result)
        values_to_write = [1]
        result = client.write_registers(address=joint_addr[n]+0x7e, values=values_to_write, slave=251)
        if not result.isError():
            print("寫入成功")
        else:
            print("寫入錯誤:", result)
            time.sleep(0.5)
        values_to_write = [0]
        result = client.write_registers(address=joint_addr[n], values=values_to_write, slave=251)

        if not result.isError():
            print("寫入成功")
        else:
            print("寫入錯誤:", result)
        result = client.write_registers(address=joint_addr[n]+0x7e, values=values_to_write, slave=251)
        if not result.isError():
            print("寫入成功")
        else:
            print("寫入錯誤:", result)
        # 關閉連接
        client.close()
    else:
        print("連接失敗")

def MODBUS_write(addr,IO_value):
    connection = client.connect()
    if connection:
        print("連接成功")
        values_to_write = [IO_value]  # 要寫入的值

        # 發送寫入多寄存器指令（0x10）
        result = client.write_registers(address=addr, values=values_to_write, slave=2)
        if not result.isError():
            print("寫入成功")
        else:
            print("寫入錯誤:", result)
        # 關閉連接
        client.close()
    else:
        print("連接失敗")


def main():
    MODBUS_write(0x1000,0x55)
    time.sleep(0.5)
    MODBUS_write(0x1000,0)
    MODBUS_write(0x1000,0)
    MODBUS_write(0x1000,0)

if __name__ == '__main__':
    sys.exit(main())

