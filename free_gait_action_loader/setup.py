from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['free_gait_action_loader'],
    scripts=['bin/free_gait_action_loader'],
    package_dir={'': 'src'}
)

setup(**d)
