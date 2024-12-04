import socket

def send_urscript(ip, port, script):
    """发送URScript到UR机器人控制器"""
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((ip, port))
        s.sendall(script.encode('utf-8'))

def main():
    robot_ip = '192.168.0.144'  # UR机器人的IP地址
    port = 63352  # URScript的端口

    # URScript命令打开夹爪
    open_gripper_script = """
    def open_gripper():
        rq_activate()
        sleep(2)  # 等待夹爪激活
        rq_open()
        sleep(2)  # 等待夹爪动作完成
    end
    open_gripper()
    """
    
    # URScript命令关闭夹爪
    close_gripper_script = """
    def close_gripper():
        rq_activate()
        sleep(2)  # 等待夹爪激活
        rq_close()
        sleep(2)  # 等待夹爪动作完成
    end
    close_gripper()
    """
    
    # 根据需要发送打开或关闭命令
    send_urscript(robot_ip, port, open_gripper_script)
    # send_urscript(robot_ip, port, close_gripper_script)

if __name__ == "__main__":
    main()
