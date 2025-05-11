import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kishore/ur5e_velocity_ws/install/ur5e_velocity_description'
