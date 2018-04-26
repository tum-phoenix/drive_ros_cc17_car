from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['drive_ros_cc2017_car'],
    scripts=['scripts/heartbeat_visualization'],
    package_dir={'': 'src'}
)

setup(**d)
