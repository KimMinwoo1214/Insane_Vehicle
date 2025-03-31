import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/chaewan/다운로드/ieve_git/path_ws/install/waypoint_parser'
