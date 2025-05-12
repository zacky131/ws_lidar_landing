import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/zacky/ws_lidar_landing/install/px4_keyboard_control'
