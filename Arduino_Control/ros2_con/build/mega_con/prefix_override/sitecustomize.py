import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/antel/2025IEVE_1of5/2025IEVE/Arduino_Control/ros2_con/install/mega_con'
