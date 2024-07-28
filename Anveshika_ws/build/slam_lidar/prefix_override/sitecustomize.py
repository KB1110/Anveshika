import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kb/Anveshika/Anveshika_ws/install/slam_lidar'
