from simple_pid import PID
import time

class MotorController:

    def __init__(self):
      self.TICKS_PER_REVOLUTION = 1000 # Calculate this value manually
      self.TICKS_PER_METER = 3100 # Number of ticks a wheel makes moving a linear distance of 1 meter
      self.prevLeftCount = 0
      self.prevTimeLeft = 0
      self.prevRightCount = 0
      self.prevTimeRight = 0

    def calculate_vel_left_wheel(self, left_ticks):
      numOfTicks = (65535 + left_ticks - self.prevLeftCount) % 65535
      if (numOfTicks > 10000): # Whenever the counter pass from positive to negative values
        numOfTicks = 0 - (65535 - numOfTicks)
      self.velLeftWheel = numOfTicks / self.TICKS_PER_METER / (time.time() - self.prevTimeLeft)
      self.prevLeftCount = left_ticks
      self.prevTimeLeft = time.time()

    def calculate_vel_right_wheel(self, right_ticks):
      numOfTicks = (65535 + right_ticks - self.prevRightCount) % 65535
      if (numOfTicks > 10000):
        numOfTicks = 0 - (65535 - numOfTicks)
      self.velRightWheel = numOfTicks / self.TICKS_PER_METER / (time.time() - self.prevTimeRight)
      self.prevRightCount = right_ticks
      self.prevTimeRight = time.time()

    def calculate_pwm_values(self, desiredSpeed_left, desiredSpeed_right, left_ticks, right_ticks):
      self.calculate_vel_left_wheel(left_ticks, right_ticks)
      Kp_left = 1; Ki_left = 1; Kd_left = 1
      Kp_right = 1; Ki_right = 1; Kd_right = 1
      pid_left = PID(Kp_left, Ki_left, Kd_left, setpoint = desiredSpeed_left)
      pid_right = PID(Kp_right, Ki_right, Kd_right, setpoint = desiredSpeed_right)
      pid_left.output_limits = (-255, 255); pid_right.output_limits = (-255, 255)
      pid_left.sample_time = 0.001; pid_right.sample_time = 0.001

      PWM_Output_left = pid_left(self.velLeftWheel)
      PWM_Output_right = pid_right(self.velRightWheel)
      return PWM_Output_left, PWM_Output_right
