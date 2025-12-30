import numpy as np
import os

# --- HÀM TÍNH TOÁN CƠ BẢN ---

def rotate_vector(vector, angle):
    """Xoay vector theo góc."""
    rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                [np.sin(angle), np.cos(angle)]])
    return rotation_matrix.dot(vector)

def is_darker(color1, color2):
    """So sánh độ tối màu."""
    # Tính trung bình cộng RGB (grayscale)
    gray1 = sum(color1) / len(color1)
    gray2 = sum(color2) / len(color2)
    return gray1 < gray2

def write_setup_file(setup_info):
    """Ghi file setup."""
    with open('setup.txt', 'w') as file:
        file.write(setup_info)

def read_setup_file():
    """
    Đọc file setup.txt.
    Hàm này dùng eval() để xử lý được cả các ký tự 'np.float64' trong file text.
    """
    if not os.path.isfile('setup.txt'):
        print("Lỗi: Không tìm thấy file setup.txt")
        return None

    with open('setup.txt', 'r') as file:
        content = file.read()
        lines = content.split('\n')
        
        # Lọc bỏ dòng trống
        lines = [l for l in lines if l.strip()]

        try:
            # Đọc dữ liệu và đóng gói vào DICTIONARY (Từ điển)
            # eval() sẽ tự động hiểu np.float64 nhờ import numpy as np ở đầu
            data = {
                'robot_width': float(lines[0]),
                'initial_motor_speed': float(lines[1]),
                'max_motor_speed': float(lines[2]),
                'wheel_radius': float(lines[3]),
                'sensors_number': int(lines[4]),
                'map_dimensions': eval(lines[5]),
                'robot_start': eval(lines[6]),
                'sensors_positions': eval(lines[7]),
                'sensor_colors': eval(lines[8])
            }
            return data
        except Exception as e:
            print(f"Lỗi đọc file: {e}")
            return None