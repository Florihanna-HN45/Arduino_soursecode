import pygame
import os
import sys
from classes import Graphics
import utils

# +=====================================================================+
# |                   CẤU HÌNH THÔNG SỐ ROBOT                           |
# +=====================================================================+
ROBOT_WIDTH = 0.1         # Chiều rộng xe (m)
INITIAL_MOTOR_SPEED = 200 # Tốc độ ban đầu
MAX_MOTOR_SPEED = 255     # Tốc độ tối đa
WHEEL_RADIUS = 0.04       # Bán kính bánh xe (m)
SENSORS_NUMBER = 5        # Số lượng cảm biến (BẮT BUỘC LÀ 5 ĐỂ KHỚP CODE CHẠY)
SENSOR_COLORS = [(255, 0, 0), (0, 255, 0), (0, 0, 255),
                (255, 255, 0), (0, 255, 255), (255, 0, 255),
                (255, 255, 255), (128, 0, 0), (0, 128, 0),
                (0, 0, 128)]

# +=====================================================================+
# |                   CHƯƠNG TRÌNH SETUP                                |
# +=====================================================================+

def run_setup():
    # 1. Khởi tạo Pygame và Màn hình
    pygame.init()
    
    # Lấy kích thước màn hình hiện tại
    infoObject = pygame.display.Info()
    MAP_DIMENSIONS = (1200, 600) # Cố định kích thước cho ổn định, hoặc dùng (infoObject.current_w - 50, infoObject.current_h - 100)
    
    # Kiểm tra đường dẫn ảnh
    if not os.path.exists('./images/agv2.png') or not os.path.exists('./images/map2.png'):
        print("LỖI: Không tìm thấy thư mục 'images' hoặc file ảnh (agv2.png, map2.png)!")
        print("Hãy đảm bảo thư mục 'images' nằm cùng chỗ với file setup.py")
        return

    # Tạo đối tượng đồ họa
    gfx = Graphics(MAP_DIMENSIONS, './images/agv2.png', './images/map2.png')

    # 2. Đặt vị trí Robot
    # Hàm này trả về vị trí (x,y,heading) và trạng thái đóng cửa sổ
    ROBOT_START, closed = gfx.robot_positioning()

    if closed:
        print("Đã đóng cửa sổ khi đang đặt Robot.")
        return

    # 3. Đặt vị trí Cảm biến
    # Lưu ý: Cần đặt 5 cảm biến theo thứ tự TRÁI -> PHẢI
    SENSORS_POSITIONS, closed = gfx.sensors_positioning(SENSORS_NUMBER, ROBOT_START, closed)

    if closed:
        print("Đã đóng cửa sổ khi đang đặt Cảm biến.")
        return

    # 4. Lưu thông tin
    if not closed:
        gfx.show_important_message("Setup complete! Saving...")
        pygame.display.update()
        pygame.time.wait(1000)

        # Tạo nội dung file string theo đúng thứ tự mà utils.py sẽ đọc
        setup_info = f"""{ROBOT_WIDTH}
{INITIAL_MOTOR_SPEED}
{MAX_MOTOR_SPEED}
{WHEEL_RADIUS}
{SENSORS_NUMBER}
{MAP_DIMENSIONS}
{ROBOT_START}
{SENSORS_POSITIONS}
{SENSOR_COLORS}"""

        # Ghi vào file
        utils.write_setup_file(setup_info)
        print(">>> Đã lưu file setup.txt thành công!")
        pygame.quit()

if __name__ == "__main__":
    # Kiểm tra nếu file đã tồn tại
    if os.path.isfile('setup.txt'):
        print('\nFile setup.txt đã tồn tại.')
        ans = input('Bạn có muốn cài đặt lại không? (y/n): ')
        if ans.lower() == 'y':
            run_setup()
        else:
            print("Giữ nguyên cấu hình cũ.")
    else:
        run_setup()