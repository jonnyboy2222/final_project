import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/addinedu/dev_ws/vic_pinky/install/vic_pinky'
