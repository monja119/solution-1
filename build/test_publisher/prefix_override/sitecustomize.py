import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/monja/projects/python/robotics/challengeParc/ros2_ws/src/install/test_publisher'
