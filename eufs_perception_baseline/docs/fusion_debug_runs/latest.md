# Fusion Debug Run - 2026-06-22 16:35:26 KST

## Runtime Accessibility

Traceback (most recent call last):
  File "/opt/ros/galactic/bin/ros2", line 11, in <module>
    load_entry_point('ros2cli==0.13.5', 'console_scripts', 'ros2')()
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/cli.py", line 67, in main
    rc = extension.main(parser=parser, args=args)
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2topic/command/topic.py", line 41, in main
    return extension.main(args=args)
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2topic/verb/list.py", line 55, in main
    with NodeStrategy(args) as node:
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/node/strategy.py", line 27, in __init__
    if use_daemon and is_daemon_running(args):
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/node/daemon.py", line 32, in is_daemon_running
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  File "/usr/lib/python3.8/socket.py", line 231, in __init__
    _socket.socket.__init__(self, family, type, proto, fileno)
PermissionError: [Errno 1] Operation not permitted
Error in sys.excepthook:
Traceback (most recent call last):
  File "/usr/lib/python3/dist-packages/apport_python_hook.py", line 153, in apport_excepthook
    with os.fdopen(os.open(pr_filename,
OSError: [Errno 30] Read-only file system: '/var/crash/_opt_ros_galactic_bin_ros2.1000.crash'

Original exception was:
Traceback (most recent call last):
  File "/opt/ros/galactic/bin/ros2", line 11, in <module>
    load_entry_point('ros2cli==0.13.5', 'console_scripts', 'ros2')()
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/cli.py", line 67, in main
    rc = extension.main(parser=parser, args=args)
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2topic/command/topic.py", line 41, in main
    return extension.main(args=args)
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2topic/verb/list.py", line 55, in main
    with NodeStrategy(args) as node:
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/node/strategy.py", line 27, in __init__
    if use_daemon and is_daemon_running(args):
  File "/opt/ros/galactic/lib/python3.8/site-packages/ros2cli/node/daemon.py", line 32, in is_daemon_running
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  File "/usr/lib/python3.8/socket.py", line 231, in __init__
    _socket.socket.__init__(self, family, type, proto, fileno)
PermissionError: [Errno 1] Operation not permitted
