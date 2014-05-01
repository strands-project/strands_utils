
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['check_something'],
    scripts=['scripts/check_door.py'],
    package_dir={'': 'src'}
)

setup(**d)