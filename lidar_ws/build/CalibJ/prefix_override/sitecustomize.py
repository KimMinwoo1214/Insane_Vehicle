import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kmw/2025IEVE/lidar_ws/install/CalibJ'
