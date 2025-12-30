import numpy as np
import pygame
import utils

# +===========================================================================+
# |                     Robot class (and related classes)                     |
# +===========================================================================+

class Motor:
    def __init__(self, max_motor_speed, wheel_radius):
        self.max_motor_speed = max_motor_speed
        self.wheel_radius = wheel_radius
        self.speed = 0
    
    def set_speed(self, speed):
        self.speed = speed

class Sensor:
    def __init__(self, sensor_relative_position, robot_initial_position):
        sensor_position_rotated = utils.rotate_vector(sensor_relative_position, robot_initial_position[2])
        self.x = robot_initial_position[0] + sensor_position_rotated[0] 
        self.y = robot_initial_position[1] - sensor_position_rotated[1] 
        self.data = 0
        
    def update_position(self, sensor_relative_position, robot_position):
        sensor_position_rotated = utils.rotate_vector(sensor_relative_position, robot_position[2])
        self.x = robot_position[0] + sensor_position_rotated[0] 
        self.y = robot_position[1] - sensor_position_rotated[1] 
    
    def read_data(self, map_image):
        try:
            # Check bounds
            if 0 <= self.x < map_image.get_width() and 0 <= self.y < map_image.get_height():
                color = map_image.get_at((int(self.x), int(self.y)))[:-1]
                self.data = 0 if utils.is_darker(color, (128, 128, 128)) else 1
            else:
                self.data = 1 # Out of bounds is white
        except:
            self.data = 1
  
class Robot:
    def __init__(self, initial_position, width, 
                 initial_motor_speed=500, max_motor_speed=1000, wheel_radius=0.04):
        self.sensors = []
        self.width = width
        self.x = initial_position[0]
        self.y = initial_position[1]
        self.heading = initial_position[2]
        
        self.left_motor = Motor(max_motor_speed, wheel_radius)
        self.right_motor = Motor(max_motor_speed, wheel_radius)
        self.left_motor.set_speed(initial_motor_speed)
        self.right_motor.set_speed(initial_motor_speed)
        
    def update_position(self, dt):
        left_wheel_linear_speed = 2*np.pi*self.left_motor.wheel_radius*self.left_motor.speed/60
        right_wheel_linear_speed = 2*np.pi*self.right_motor.wheel_radius*self.right_motor.speed/60
        
        x_speed = (left_wheel_linear_speed + right_wheel_linear_speed)*np.cos(self.heading)/2
        y_speed = (left_wheel_linear_speed + right_wheel_linear_speed)*np.sin(self.heading)/2
        heading_speed = (right_wheel_linear_speed - left_wheel_linear_speed)/self.width
        
        self.x += x_speed*dt
        self.y -= y_speed*dt 
        self.heading += heading_speed*dt
        
        if (self.heading > 2*np.pi) or (self.heading < -2*np.pi):
            self.heading = 0
    
    def stop(self):
        self.left_motor.set_speed(0)
        self.right_motor.set_speed(0)
    
    def add_sensor(self, sensor_relative_position, robot_initial_position):
        self.sensors.append(Sensor(sensor_relative_position, robot_initial_position))

# +===========================================================================+
# |                               Graphics class                              |
# +===========================================================================+

class Graphics:
    def __init__(self, screen_dimensions, robot_image_path, map_imape_path):
        pygame.init()
        self.robot_image = pygame.image.load(robot_image_path)
        self.map_image = pygame.transform.scale(pygame.image.load(map_imape_path), screen_dimensions)
        pygame.display.set_caption("Line Follower Simulator")
        self.map = pygame.display.set_mode(screen_dimensions)
        self.map.blit(self.map_image, (0, 0))
    
    def robot_positioning(self):
        running = True
        closed = False
        robot_start_heading = np.pi/2
        xy_positioned = False
        heading_positioned = False
        robot_start_x, robot_start_y = 0, 0 # Fix UnboundLocalError
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    running = False
                    closed = True

                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: 
                    robot_start_x, robot_start_y = pygame.mouse.get_pos()
                    xy_positioned = True
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.button in [4, 5]:
                    direction = 1 if event.button == 4 else -1
                    robot_start_heading += direction*np.pi/6
                    robot_start_heading %= (2*np.pi)
                    heading_positioned = True
                    
                if (event.type == pygame.KEYDOWN and xy_positioned and heading_positioned):
                    running = False
                    
            self.map.blit(self.map_image, (0, 0))
            
            # Instructions
            if not xy_positioned:
                 mx, my = pygame.mouse.get_pos()
                 self.draw_robot(mx, my, robot_start_heading)
            else:
                 self.draw_robot(robot_start_x, robot_start_y, robot_start_heading)
                 
            self.show_important_message("Click to Place Robot. Scroll to Rotate. Press Key to Confirm.")
            pygame.display.update()
            
        return (robot_start_x, robot_start_y, robot_start_heading), closed
    
    def sensors_positioning(self, number_of_sensors, robot_start, closed):
        running = True
        sensors_positions = []
        sensors_relative_positions = []
        counter = 0
        sensor_colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255)] # Top 5 colors

        while counter < number_of_sensors and running and not closed:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: 
                    running = False
                    closed = True
                
                if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1: 
                    sx, sy = pygame.mouse.get_pos()
                    sensors_positions.append((sx, sy))
                    
                    # Calculate relative position
                    rel_x = sx - robot_start[0]
                    rel_y = robot_start[1] - sy
                    rel_pos = utils.rotate_vector((rel_x, rel_y), -robot_start[2])
                    sensors_relative_positions.append(list(rel_pos))
                    counter += 1
            
            self.map.blit(self.map_image, (0, 0))
            self.draw_robot(robot_start[0], robot_start[1], robot_start[2])
            
            for i, pos in enumerate(sensors_positions):
                self.draw_sensor_symbol(pos, sensor_colors[i % len(sensor_colors)])
            
            # Draw cursor sensor
            if counter < number_of_sensors:
                mx, my = pygame.mouse.get_pos()
                self.draw_sensor_symbol((mx, my), sensor_colors[counter % len(sensor_colors)])
                
            self.show_important_message(f"Place Sensor {counter+1}/{number_of_sensors}")
            pygame.display.update()
            
        return sensors_relative_positions, closed
    
    def draw_robot(self, x, y, heading):
        rotated_robot = pygame.transform.rotozoom(self.robot_image, np.degrees(heading), 1)
        rect = rotated_robot.get_rect(center=(x, y))
        self.map.blit(rotated_robot, rect)
        
    def draw_sensor(self, sensor, color=(255, 0, 0)):
        self.draw_sensor_symbol((int(sensor.x), int(sensor.y)), color)
        
    def draw_sensor_symbol(self, position, color):
        pygame.draw.circle(self.map, (0, 0, 0), position, 4) # Border
        pygame.draw.circle(self.map, color, position, 3) # Inner
        
    def show_sensors_data(self, sensors, sensor_colors):
        # Simplified display
        font = pygame.font.SysFont("Arial", 20)
        y = 50
        for i, s in enumerate(sensors):
            col = sensor_colors[i % len(sensor_colors)]
            txt = font.render(f"S{i}: {s.data}", True, col)
            self.map.blit(txt, (10, y))
            y += 25
            
    def show_important_message(self, message):
        font = pygame.font.SysFont("Arial", 20)
        text = font.render(message, True, (0, 0, 0))
        # Draw background box for text
        bg = pygame.Rect(10, 10, text.get_width() + 10, text.get_height() + 10)
        pygame.draw.rect(self.map, (255, 255, 255), bg)
        self.map.blit(text, (15, 15))

    def show_text(self, text, position, fontsize=30, color=(0, 0, 0)):
        pass # Not used heavily anymore