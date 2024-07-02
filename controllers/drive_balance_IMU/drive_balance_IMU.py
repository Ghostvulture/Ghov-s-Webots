"""drive_balance_IMU controller."""

import sys
sys.path.append("$WEBOTS_HOME/lib/controller/python3.12/")


from controller import Robot
import numpy as np
from math import pi
import time
from controller import display

robot = Robot()
TIME_STEP = 32
MAX_Motor_Speed = 40

'''初始化设置和时钟使能'''
# 电机设置
# 左轮 电机速度为正 顺时针旋转 车体后退
# 右轮 电机速度为正 逆时针旋转 车体后退
MotorNames = ['motor1', 'motor2']
motor = [] 
for i in range(len(MotorNames)):
    motor.append(robot.getDevice(MotorNames[i]))
    motor[i].setPosition(float('inf'))  # 电机位置设置
    motor[i].setVelocity(0)             # 电机速度设置
# 电机编码器设置（position sensor)
left_encoder = robot.getDevice('ps_1')
right_encoder = robot.getDevice('ps_2')
left_encoder.enable(TIME_STEP)                         # 使能
right_encoder.enable(TIME_STEP)                        # 使能

'''设置电机转速'''
def SetMotorSpeed(LeftMotorSpeed, RightMotorSpeed):
    if abs(LeftMotorSpeed) > MAX_Motor_Speed:
        LeftMotorSpeed = LeftMotorSpeed / abs(LeftMotorSpeed) * MAX_Motor_Speed
    if abs(RightMotorSpeed) > MAX_Motor_Speed:
        RightMotorSpeed = RightMotorSpeed / abs(RightMotorSpeed) * MAX_Motor_Speed
    motor[0].setVelocity(LeftMotorSpeed)
    motor[1].setVelocity(RightMotorSpeed)
    print(f"LeftSpeed = {round(LeftMotorSpeed, 3)}, RightSpeed = {round(RightMotorSpeed, 3)}")

left_velocity = 0
right_velocity = 0
velocity = 0
def GetMotorVelocity():
    global left_velocity, right_velocity, velocity
    left_velocity = motor[0].getVelocity()                      # 左电机速度
    right_velocity = motor[1].getVelocity()                     # 右电机速度
    velocity = (left_velocity + right_velocity) / 2             # 平均速度
    print(f"left_velocity = {round(left_velocity, 3)}, right_velocity = {round(right_velocity, 3)}")
    return velocity


# IMU传感器设置
IMU = robot.getDevice('IMU')
IMU.enable(TIME_STEP)  # 确保IMU设备已经被启用

# 读取IMU模块值
# ENU东北天坐标系
# xAxis = roll; yAxis = pitch; zAxis = yaw  当前坐标系
# NUE坐标系
# xAxis = roll; yAxis = yaw; zAxis = pitch
def GetIMUValue():
    roll, pitch, yaw = IMU.getRollPitchYaw()
    Rotation_Matrix = np.array([[0, 1, 0], [1, 0, 0], [0, 0, 1]])
    [[roll], [pitch], [yaw]] = Rotation_Matrix.dot(np.array([[roll], [pitch], [yaw]]))
    # 弧度制转化为角度制
    roll = round(roll * 180 / pi, 3)          # 滚转角  绕X轴旋转
    pitch = round(pitch * 180 / pi, 3)        # 俯仰角  绕Y轴旋转
    yaw = round(yaw * 180 / pi, 3)            # 航向角  绕Z轴旋转      逆时针旋转角度增大 [0 --> 180 --> -180 --> 0]
    print(f"roll = {roll},Pitch = {pitch} ,Yaw = {yaw} ")
    return roll, pitch, yaw

# 编码器设置
left_position = 0
right_position = 0
position = 0
def GetPosition():
    global left_position, right_position, position
    left_position = left_encoder.getValue()                  # 左电机编码器
    right_position = right_encoder.getValue()                # 右电机编码器
    position = (left_position + right_position) / 2          # 左右电机平均位置
    print(f"position = {round(position, 3)}")
    return position

'''pid控制算法'''
# 平衡PID控制算法    直立环     PD控制
latest_pitch_error = 0
pitch_integral = 0
Balance_Kp = 2# 2.5
Balance_Ki = 0
Balance_Kd = 2
pitch_radio = 1

def BalancePIDControl(current_pitch):
    global latest_pitch_error, pitch_integral                   # 全局变量
    pitch_error = current_pitch - 0.0                           # 偏差，期望值ref为0.0（中立位）
    pitch_derivative = pitch_error - latest_pitch_error         # 差分
    pitch_integral += pitch_error                               # 积分
    pitch_control = pitch_radio * (Balance_Kp * pitch_error + Balance_Ki * pitch_integral + Balance_Kd * pitch_derivative)
    print(f"control = {round(pitch_control, 3)}, derivative = {round(pitch_derivative, 3)}")
    latest_pitch_error = pitch_error
    return pitch_control

# 速度PID控制算法    速度环        PI控制
latest_speed_error = 0
speed_integral = 0
# 2023年5月14日 0:34 Speed_Kp = 0.9
Speed_Kp = 2#1.2
Speed_Ki = Speed_Kp / 200
Speed_Kd = 0
pos_integral = 0
def VelocityPIDControl(current_speed):
    global latest_speed_error, speed_integral
    global pos_integral
    # 当前速度减目标速度
    speed_error = current_speed - speed                      # 偏差
    speed_derivative = speed_error - latest_speed_error      # 差分
    # 一阶低通滤波器
    speed_integral *= 0.7
    speed_integral += speed_error * 0.3                      # 积分
    pos_integral += speed_integral                           # 位置积分
    # 积分幅限
    if speed_integral > 40:
        speed_integral = 40
    if speed_integral < -40:
        speed_integral = -40
    speed_control = Speed_Kp * speed_error + Speed_Ki * speed_integral + Speed_Kd * speed_derivative
    # speed_control = Speed_Kp * speed_error + Speed_Ki * (pos_integral - pos) + Speed_Kd * speed_derivative
    latest_speed_error = speed_error
    return speed_control

latest_yaw_error = 0
Yaw_Kp = 1#0.5
Yaw_Ki = 0
Yaw_Kd = 0.2
yaw_integral = 0
# 转向PID控制算法  转向环      PD控制
def SteerPIDControl(current_yaw):
    global yaw_integral
    yaw_error = current_yaw - angle
    yaw_derivative = yaw_error - latest_yaw_error
    yaw_integral += yaw_error
    yaw_control = Yaw_Kp * yaw_error + Yaw_Ki * yaw_integral + Yaw_Kd * yaw_derivative
    print("yaw control: {}".format(yaw_control))
    return yaw_control

# 键盘控制
robot.keyboard.enable(TIME_STEP)
pos = 0
speed = 0
angle = 0
def GetKeyboardValue():
    global pos, angle, speed
    key = robot.keyboard.getKey()
    if key == robot.keyboard.UP:                  # 上箭头键    前进
        pos += 0.1
        speed += 0.01
        print(f"pos = {pos}")
    elif key == robot.keyboard.DOWN:              # 下箭头键    后退
        pos -= 1
        speed -= 0.5
        print(f"pos = {pos}")
    elif key == robot.keyboard.LEFT:              # 左箭头键    向左旋转
        angle += 0.001
        print(f"angle = {angle}")
    elif key == robot.keyboard.RIGHT:             # 右箭头键    向右旋转
        angle -= 5
        print(f"angle = {angle}")
    elif key == 32:                               # 空格键键值
        speed = 0                                 # 设定速度为0

    if key != -1:
        print(f"Key Code = {key}")

'''display窗口,详见https://www.cyberbotics.com/doc/reference/display?version=master&tab-language=python#display'''
#display窗口初始化
balance_display = robot.getDevice('balance_display')
width = balance_display.getWidth()
height = balance_display.getHeight()

max_data_point = 70
data_pts = []

t = 0

'''主执行函数'''
while robot.step(TIME_STEP) != -1:
    print("-------------------------")
    [roll, pitch, yaw] = GetIMUValue()
    Pitch_Speed = BalancePIDControl(pitch)  # 直立环控制
    Yaw_Speed = SteerPIDControl(yaw)        # 转向环控制
    # SetMotorSpeed(Pitch_Speed, Pitch_Speed)
    car_pos = GetPosition()
    print(f"car_pos = {round(car_pos, 3)}")
    motor_speed = GetMotorVelocity()
    Vel_Speed = VelocityPIDControl(motor_speed)  # 速度环控制
    print(f"Vel_Speed = {Vel_Speed}")
    # SetMotorSpeed(Vel_Speed, Vel_Speed)
    Left_Speed = Pitch_Speed + Vel_Speed - Yaw_Speed             # 左轮电机速度设置
    Right_Speed = Pitch_Speed + Vel_Speed + Yaw_Speed            # 右轮电机速度设置
    print(f"Left_Speed = {round(Left_Speed, 3)}, Right_Speed = {round(Right_Speed, 3)}, speed = {speed}")
    SetMotorSpeed(Left_Speed, Right_Speed)

    balance_display.setColor(0x000000)
    balance_display.fillRectangle(0, 0, width, height)
    # balance_display.setColor(0xFF00FF)                 #目标pitch值
    # balance_display.drawPixel(t*10, 0)
    balance_display.setColor(0xAA00FF)                 

    data_pts.append(pitch*20 + 25)
    if len(data_pts) > max_data_point:
        data_pts.pop(0)

    for i in range(len(data_pts)):
        balance_display.drawPixel(i, data_pts[i])

    t += 1

    GetKeyboardValue()                                 # 键盘控制
    time.sleep(0.02)

