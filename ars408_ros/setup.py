## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['ars408_ros', 'config'],
    package_dir={'': 'include', '': ''},
    scripts=['bin/default.py', 'bin/rps.py'],
)

setup(**setup_args)
