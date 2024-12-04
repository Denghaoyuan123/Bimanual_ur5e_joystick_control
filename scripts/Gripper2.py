from pymodbus.client.sync import ModbusTcpClient
import time

# 连接到夹爪的Modbus服务器
client = ModbusTcpClient('192.168.0.144', port=54321)  # IP地址和端口根据实际情况修改
client.connect()

# 定义Robotiq 2F-85的一些基本Modbus寄存器地址
ACTIVATE_REGISTER = 0x03E8  # 激活夹爪的寄存器
GRIPPER_STATUS_REGISTER = 0x07D0  # 读取夹爪状态的寄存器
GRIPPER_CONTROL_REGISTER = 0x07D3  # 控制夹爪开闭的寄存器

# 激活夹爪
client.write_register(ACTIVATE_REGISTER, 1)
time.sleep(1)  # 等待夹爪激活

# 检查夹爪是否激活
status = client.read_holding_registers(GRIPPER_STATUS_REGISTER, 1)
print("Gripper Status:", status.registers[0])

# 控制夹爪关闭
client.write_register(GRIPPER_CONTROL_REGISTER, 0xFF)
time.sleep(1)  # 等待夹爪操作完成

# 控制夹爪开启（完全开启位置）
client.write_register(GRIPPER_CONTROL_REGISTER, 0x00)
time.sleep(1)  # 等待夹爪操作完成

# 断开连接
client.close()