from machine import Pin, PWM
import time
import math
from mpu6050 import MPU6050

import gc

#pin declorations
m1 = PWM(Pin(0)) #front left, clockwise
m2 =  PWM(Pin(2)) #front right, counter clockwise
m3 = PWM(Pin(4)) #rear left, counter clockwise
m4 =  PWM(Pin(6)) #rear right, clockwise

m1.freq(1000)
m2.freq(1000)
m3.freq(1000)
m4.freq(1000)

#setup mpu6050 i2c connection
i2c = machine.I2C(0, scl=machine.Pin(9), sda=machine.Pin(8))
mpu = MPU6050(i2c)
mpu.wake()

#overclock
machine.freq(250000000)

#constants
roll_kP = 1
roll_kI = 0
roll_kD = 0

pitch_kP = 1
pitch_kI = 0
pitch_kD = 0

yaw_kP = 1
yaw_kI = 0
yaw_kD = 0

roll_rate_kP = 1
pitch_rate_kP = 0
yaw_rate_kP = 0

throttle_kP = 1
throttle_kI = 0
throttle_kP = 0

integral_limit = 150

max_roll = 20
max_pitch = 20

alpha = 0.98

target_hz = 500



def read_accel_and_gyro():
    """Function reads accelerometer and calculates pitch and roll angles based on this, and also reads gyro."""

    Ax, Ay, Az = mpu.read_accel_data()
    Gx, Gy, Gz = mpu.read_gyro_data()
    
    #correct for gyroscope bias
    Gx -= gyro_bias_x
    Gy -= gyro_bias_y
    Gz -= gyro_bias_z
    
    pitch = math.atan2(Ay, Az) * (180 / math.pi)
    roll = math.atan2(Ax, Az) * (180 / math.pi)
    
    return(roll,pitch,Gy,Gx,Gz)
    
    
    
def find_gyro_bias(calibration_time):
    """Function takes calibration time in seconds and returns gyro_bias_x, y and z"""
    x_list = []
    y_list = []
    z_list = []
    
    start_time = time.ticks_ms()
    
    while ((time.ticks_ms() - start_time) / 1000) < calibration_time: 
        gyro_x, gyro_y, gyro_z = mpu.read_gyro_data()
        x_list.append(gyro_x)
        y_list.append(gyro_y)
        z_list.append(gyro_z)
        time.sleep(0.025)
        
    gyro_bias_x = sum(x_list) / len(x_list)
    gyro_bias_y = sum(y_list) / len(y_list)
    gyro_bias_z = sum(z_list) / len(z_list)
    
    return(gyro_bias_x, gyro_bias_y, gyro_bias_z)
    
    
def apply_throttle(motor, throttle):
    """Function takes motor name and a throttle, and applys the appropriate pwm signal to that motors pin"""
    throttle = min(max(throttle, 0), 1)
    duty_cycle = int(throttle * 65535)
    motor.duty_u16(duty_cycle)
    
#calculate time per cycle
delta_t = 1/target_hz

#get gyro bias values
gyro_bias_x, gyro_bias_y, gyro_bias_z = find_gyro_bias(5)

#initialise
roll_last_error = 0
pitch_last_error = 0
yaw_last_error = 0
throttle_last_error = 0
    
roll_last_integral = 0
pitch_last_integral = 0
yaw_last_integral = 0
throttle_last_integral =0
    
prev_comp_roll = 0
prev_comp_pitch = 0


#just so the code runs, these would come from outer loop
yaw_reading = roll_input = pitch_input = vertical_offset = 0


while True:
    #cascading PID control loop for controlling the drone based on inputs from the outer loop
    loop_start_time = time.ticks_us()
    
    #get angles using complimentary filtering
    roll,pitch,roll_rate,pitch_rate,yaw_rate = read_accel_and_gyro()
    comp_roll = alpha * (prev_comp_roll + roll_rate * delta_t) + (1 - alpha) * roll
    comp_pitch = alpha * (prev_comp_pitch + pitch_rate * delta_t) + (1 - alpha) * pitch
    yaw = yaw_reading #yaw is read in the outer loop based on april tag shape
    
    #calculate desired angles 
    desired_roll = roll_input*max_roll #roll input from outer loop, -1 to 1
    desired_pitch = pitch_input*max_pitch  #pitch input from outer loop, -1 to 1
    desired_yaw = 0 #we always want drone facing tag
    
    #calculate angle errors 
    roll_error = desired_roll - comp_roll
    pitch_error = desired_pitch - comp_pitch
    yaw_error = desired_yaw - yaw
    
    #calculate desired angle rates based on angle errors 
    desired_rate_roll = roll_error * roll_rate_kP
    desired_rate_pitch = pitch_error * pitch_rate_kP
    desired_rate_yaw = yaw_error * yaw_rate_kP
    
    #calculate angle rate errors 
    error_rate_roll = desired_rate_roll - roll_rate
    error_rate_pitch = desired_rate_pitch - pitch_rate 
    error_rate_yaw = desired_rate_yaw - yaw_rate
    
    #PID rate controllers
    P_roll = roll_kP * error_rate_roll
    I_roll = roll_last_integral + roll_kI * (error_rate_roll * delta_t)
    I_roll = min(max(I_roll, -integral_limit), integral_limit) #clamp to prevent integral windup
    D_roll = (error_rate_roll - roll_last_error) / delta_t
    PID_roll = P_roll + I_roll + D_roll
    
    P_pitch = pitch_kP * error_rate_pitch
    I_pitch = pitch_last_integral + pitch_kI * (error_rate_pitch * delta_t)
    I_pitch = min(max(I_pitch, -integral_limit), integral_limit)
    D_pitch = (error_rate_pitch - pitch_last_error) / delta_t
    PID_pitch = P_pitch + I_pitch + D_pitch
    
    P_yaw = yaw_kP * error_rate_yaw
    I_yaw = yaw_last_integral + yaw_kI * (error_rate_yaw * delta_t)
    I_yaw = min(max(I_yaw, -integral_limit), integral_limit) 
    D_yaw = (error_rate_yaw - yaw_last_error) / delta_t
    PID_yaw = P_yaw + I_yaw + D_yaw
    
    #calculate the appropriate base throttle needed for vertical alignment with april tag - this is not a cascading PID loop, as there is no easy way to find the drones vertical velocity
    throttle_error = vertical_offset #vertical offset based on april tag position from outerloop
    
    #throttle PID controller based on altitude error
    P_throttle = throttle_kP * throttle_error
    I_throttle = throttle_last_integral + throttle_kI * (throttle_error * delta_t)
    I_throttle = min(max(I_throttle, -integral_limit), integral_limit)
    D_throttle = (throttle_error - throttle_last_error) / delta_t
    PID_throttle = P_throttle + I_throttle + D_throttle
    
    #throttle mixing
    t1 = PID_throttle + PID_pitch + PID_roll - PID_yaw
    t2 = PID_throttle + PID_pitch - PID_roll + PID_yaw
    t3 = PID_throttle - PID_pitch + PID_roll + PID_yaw
    t4 = PID_throttle - PID_pitch - PID_roll - PID_yaw
    
    #apply throttles to motors
    apply_throttle(m1, t1)
    apply_throttle(m2, t2)
    apply_throttle(m3, t3)
    apply_throttle(m4, t4)
    
    #update for next loop
    roll_last_error = error_rate_roll
    pitch_last_error = error_rate_pitch
    yaw_last_error = error_rate_yaw
    throttle_last_error = throttle_error
    
    roll_last_integral = I_roll
    pitch_last_integral = I_pitch
    yaw_last_integral = I_yaw
    throttle_last_integral = I_throttle
    
    prev_comp_roll = comp_roll
    prev_comp_pitch = comp_pitch
    
    #try to run at constant frequency
    loop_end_time = time.ticks_us()
    elapsed_time = time.ticks_diff(loop_end_time, loop_start_time) 
    if elapsed_time < delta_t * 1000000:
        time.sleep_us(int(delta_t * 1000000) - elapsed_time) #since delta_t is in seconds

    


    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
