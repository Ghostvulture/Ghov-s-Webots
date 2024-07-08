"""tool_box"""

from controller import Robot

#创建robot实例
robot = Robot()

#每隔16ms程序执行一次
timestep = 16

#main函数
if __name__ == "__main__":
    #while前执行初始化（速度角度，使能）
    while robot.step(timestep) != -1:
        #程序执行逻辑
        pass


'''调参用显示窗口类，Robot实例名称设置为robot，display设置名称为display_window'''
class DisplayWindow:
    def __init__(self, robot, mx_data_point = 70):
        self.display_window = robot.getDevice('display_window')
        self.width = self.display_window.getWidth()
        self.height = self.display_window.getHeight()
        self.mx_data_point = mx_data_point
        self.data_pts_ref = []
        self.data_pts_fdb = []
        self.t = 0

    def update_display(self, ref, fdb):
        #清空显示屏
        self.display_window.setColor(0x000000)
        self.display_window.fillRectangle(0, 0, self.width, self.height)

        #更新数据ref
        self.data_pts_ref.append(ref * 20 + 25)
        if len(self.data_pts_ref) > self.mx_data_point:
            self.data_pts_ref.pop(0)

        # 更新数据fdb
        self.data_pts_fdb.append(fdb * 20 + 25)
        if len(self.data_pts_fdb) > self.mx_data_point:
            self.data_pts_fdb.pop(0)

        #绘制数据ref
        self.display_window.setColor(0xAA00FF)
        for i in range(len(self.data_pts_ref)):
            self.display_window.drawPixel(i, self.data_pts_ref[i])


        #绘制数据fdb
        self.display_window.setColor(0x00FF00)
        for i in range(len(self.data_pts_fdb)):
            self.display_window.drawPixel(i, self.data_pts_fdb[i])

        self.t += 1

'''pid类'''
class PID_control:
    def _init__(self, kp, ki, kd, ref):
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






















