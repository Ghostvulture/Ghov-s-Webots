"""inv_pendulum_ctrl controller."""

from controller import Robot
import math



'''pid类'''
class PID_control:
    def __init__(self, kp, ki, kd, ref):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ref = ref

        #增量式参数
        self.computed_input= 0.0
        self.last_error = 0.0
        self.last_last_error = 0.0

        #位置式参数
        self.last_time = 0.0
        self.sum_err = 0.0
        self.theta_last = 0.0

    def increment_pid(self, fdb):
        pres_err = self.ref - fdb
        self.computed_input += self.kp * (pres_err - self.last_error) + self.ki * pres_err + self.kd * (pres_err - 2 * self.last_error + self.last_last_error)
        self.last_last_error = self.last_error
        self.last_error = pres_err

        return  self.computed_input

    def position_pid(self, fdb, current_time):
        #single step duration
        dt = current_time - self.last_time
        #Error
        pres_err = self.ref - fdb
        #Error accumulation
        self.sum_err += pres_err * dt
        #Derivative of error
        d_err = (fdb - self.theta_last) / dt
        #PID control signal
        u = self.kp * pres_err + self.ki * self.sum_err + self.kd * d_err
        #update theta_last and last_time
        self.theta_last = fdb
        self.last_time = current_time

        return u



#main函数
if __name__ == "__main__":
    #执行初始化

    # 创建robot实例
    robot = Robot()

    # 每隔16ms程序执行一次
    TIME_STEP = 16
    mx_motor_spd = 40


    #电机初始化
    MotorNames = ['Lm_1', 'Lm_2', 'Rm_1', 'Rm_2']
    motor = []
    for i in range(len(MotorNames)):
        motor.append(robot.getDevice(MotorNames[i]))
        motor[i].setPosition(float('inf'))  # 电机位置设置
        motor[i].setVelocity(0)             # 电机速度设置

    #电机角度传感器初始化
    Lps_1 = robot.getDevice('Lps_1')
    Lps_2 = robot.getDevice('Lps_2')
    Rps_1 = robot.getDevice('Rps_1')
    Rps_2 = robot.getDevice('Rps_2')
    rod_ps = robot.getDevice('rod_ps')
    Lps_1.enable(TIME_STEP)  # 使能
    Lps_2.enable(TIME_STEP)
    Rps_1.enable(TIME_STEP)
    Rps_2.enable(TIME_STEP)
    rod_ps.enable(TIME_STEP)


    radius = 0.02
    length = 0.12 / 2
    g = 9.81

    theta_Kp = 5
    theta_Ki = 1
    theta_Kd = 0.1
    theta_ctrl = PID_control(theta_Kp, theta_Ki, theta_Kd, ref = 0)
    dis_kp = 1
    dis_ki = 0
    dis_kd = 0.1
    displacement_ctrl = PID_control(dis_kp, dis_ki, dis_kd, ref = 0)

    i = 0

    while robot.step(TIME_STEP) != -1:
        if i == 0:
            ps_rod_zero = rod_ps.getValue()
            ps_l1_zero = Lps_1.getValue()
            ps_l2_zero = Lps_2.getValue()
            ps_r1_zero = Rps_1.getValue()
            ps_r2_zero = Rps_2.getValue()
            current_time = 0.0
            i = 1

        else:
            #theta_rod: 杆子和垂直竖线的夹角，角度制
            theta_rod = (rod_ps.getValue() - ps_rod_zero) * 180 / math.pi
            #四个轮子分别的路程
            ps_l1_val = (Lps_1.getValue() - ps_l1_zero) * radius
            ps_l2_val = (Lps_2.getValue() - ps_l2_zero) * radius
            ps_r1_val = (Rps_1.getValue() - ps_r1_zero) * radius
            ps_r2_val = (Rps_2.getValue() - ps_r2_zero) * radius
            #小车线性位移
            x_pres = (ps_l1_val + ps_r2_val + ps_r1_val + ps_l2_val) / 4

            #并行双环pid
            current_time = robot.getTime()
            input_value_theta = theta_ctrl.increment_pid(theta_rod)
            input_value_displacement = displacement_ctrl.increment_pid(x_pres)
            input_value = input_value_theta + input_value_displacement

            for i in range(4):
                motor[i].setVelocity(-input_value)

            print("-----------------------")
            print("linear movement: " + str(x_pres))
            print("rod angle: " + str(theta_rod) + " degree")
            print("input_value_theta: " + str(input_value_theta))
            print("input_value_displacement: " + str(input_value_displacement))
            print("input_value: " + str(input_value))
































