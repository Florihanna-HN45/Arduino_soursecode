import pygame
import time
import numpy as np
import utils
from classes import Robot, Graphics

# =============================================================================
# CẤU HÌNH THÔNG SỐ PID (Giống code Arduino)
# =============================================================================
kp = 35
ki = 0.002
kd = 5

last_timer = 0
last_error = 0
tichphan = 0

# Tốc độ cơ bản (Map từ 0-255 của Arduino sang RPM của Robot ảo)
# Bạn có thể tăng giảm BASE_SPEED_FACTOR để xe chạy nhanh/chậm
BASE_SPEED_ARDUINO = 165 
MAX_SPEED_ARDUINO = 255

values = [0, 0, 0, 0, 0] 
read_last_value = 0.0
dem_ngaba = 0
dem_nga4 = 0

# =============================================================================
# CÁC HÀM HỖ TRỢ
# =============================================================================

def map_speed(arduino_pwm, max_rpm):
    """Chuyển PWM (0-255) sang RPM thực tế"""
    pwm = max(-255, min(arduino_pwm, 255)) 
    return (pwm / 255.0) * max_rpm

def doc_cambien(robot, gfx):
    global values
    for i, sensor in enumerate(robot.sensors):
        sensor.read_data(gfx.map_image)
        values[i] = sensor.data

def tinh_toan_loi(robot, gfx):
    global read_last_value
    doc_cambien(robot, gfx)
    
    error_sum = 0
    active_sensors = 0
    
    # Logic trọng số: Trái(-2, -1) --- Giữa(0) --- Phải(1, 2)
    if values[0] == 0: error_sum -= 2; active_sensors += 1
    if values[1] == 0: error_sum -= 1; active_sensors += 1
    if values[2] == 0: error_sum += 0; active_sensors += 1
    if values[3] == 0: error_sum += 1; active_sensors += 1
    if values[4] == 0: error_sum += 2; active_sensors += 1
    
    if active_sensors > 0:
        read_last_value = error_sum / active_sensors

def controller_pid(setvalue, readvalue, dt):
    global tichphan, last_error
    
    if dt <= 0: dt = 0.001
    
    error = setvalue - readvalue
    tichphan += error * dt
    daoham = (error - last_error) / dt
        
    output = (kp * error) + (ki * tichphan) + (kd * daoham)
    
    last_error = error
    return output

# --- ĐIỀU KHIỂN MOTOR ---

def set_motor(robot, l_pwm, r_pwm):
    robot.left_motor.set_speed(map_speed(l_pwm, robot.left_motor.max_motor_speed))
    robot.right_motor.set_speed(map_speed(r_pwm, robot.right_motor.max_motor_speed))

def tien(robot, l, r): set_motor(robot, l, r)
def stop(robot): set_motor(robot, 0, 0)
def trai(robot, speed): set_motor(robot, -speed, speed) # Quay trái tại chỗ
def phai(robot, speed): set_motor(robot, speed, -speed) # Quay phải tại chỗ

# --- LOGIC ĐẶC BIỆT (Mô phỏng blocking của Arduino) ---

def re_trai_90_do_sim(robot, gfx, dt_step):
    # 1. Quay mù 0.3s
    timer = 0
    while timer < 0.3:
        trai(robot, 180)
        robot.update_position(dt_step)
        gfx.map.blit(gfx.map_image, (0, 0))
        gfx.draw_robot(robot.x, robot.y, robot.heading)
        pygame.display.update()
        timer += dt_step

    # 2. Quay đến khi mắt giữa thấy đen
    doc_cambien(robot, gfx)
    while values[2] == 1: # Đang trắng
        trai(robot, 180)
        robot.update_position(dt_step)
        doc_cambien(robot, gfx)
        
        gfx.map.blit(gfx.map_image, (0, 0))
        gfx.draw_robot(robot.x, robot.y, robot.heading)
        pygame.display.update()
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return
    
    stop(robot)

# =============================================================================
# MAIN
# =============================================================================

def main():
    global dem_nga4, dem_ngaba
    
    # 1. Đọc file setup (Đã sửa utils để trả về Dictionary)
    setup_data = utils.read_setup_file()
    if not setup_data:
        return

    # 2. Khởi tạo Robot
    robot = Robot(setup_data['robot_start'], 
                  setup_data['robot_width'], 
                  setup_data['initial_motor_speed'], 
                  setup_data['max_motor_speed'], 
                  setup_data['wheel_radius'])
    
    for pos in setup_data['sensors_positions']:
        robot.add_sensor(pos, setup_data['robot_start'])

    gfx = Graphics(setup_data['map_dimensions'], './images/agv2.png', './images/blue.png')

    running = True
    clock = pygame.time.Clock()
    
    while running:
        dt = clock.tick(60) / 1000.0 
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT: running = False

        # --- LOGIC XE CHẠY ---
        tinh_toan_loi(robot, gfx)

        # 1. NGÃ TƯ (5 mắt đen)
        if all(v == 0 for v in values):
            dem_nga4 += 1
            print(f"Ngã 4: {dem_nga4}")
            if dem_nga4 == 4:
                re_trai_90_do_sim(robot, gfx, dt)
            else:
                # Đi thẳng thoát ngã tư
                tout = 0
                while values[2] == 0 and tout < 50:
                    tien(robot, 180, 180)
                    robot.update_position(0.01) # bước nhỏ
                    doc_cambien(robot, gfx)
                    tout += 1
            continue

        # 2. NGÃ 3 (T-Trái)
        elif values[0]==0 and values[1]==0 and values[2]==0 and values[4]==1:
            dem_ngaba += 1
            if dem_ngaba == 3:
                re_trai_90_do_sim(robot, gfx, dt)
            else:
                # Đi thẳng
                tien(robot, 180, 180)
            continue

        # 3. NGÃ 3 (T-Phải) - Code Arduino của bạn dùng trai(200)? Mình giữ nguyên logic.
        elif values[4]==0 and values[3]==0 and values[2]==0 and values[0]==1:
            dem_ngaba += 1
            if dem_ngaba == 4:
                # Mô phỏng delay(450) và quay
                t_delay = 0
                while t_delay < 0.45:
                    trai(robot, 200) # Theo code gốc của bạn là Trai?
                    robot.update_position(dt)
                    t_delay += dt
            else:
                tien(robot, 180, 180)
            continue

        # 4. PID
        output = controller_pid(0, read_last_value, dt)
        l_motor = BASE_SPEED_ARDUINO - output
        r_motor = BASE_SPEED_ARDUINO + output
        
        # 5. Mất Line
        if all(v == 1 for v in values):
            if read_last_value < 0: trai(robot, 160)
            else: phai(robot, 160)
        else:
            tien(robot, l_motor, r_motor)

        # Update Vật lý & Hình ảnh
        robot.update_position(dt)
        
        gfx.map.blit(gfx.map_image, (0, 0))
        gfx.draw_robot(robot.x, robot.y, robot.heading)
        for i, s in enumerate(robot.sensors):
            gfx.draw_sensor(s, setup_data['sensor_colors'][i])
            
        pygame.display.update()

if __name__ == "__main__":
    main()