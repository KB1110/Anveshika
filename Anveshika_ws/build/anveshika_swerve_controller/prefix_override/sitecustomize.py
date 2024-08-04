import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pi/Anveshika/Anveshika_ws/install/anveshika_swerve_controller'
