# Author: ML lab continuum-actuation team 
# University of California, San Diego

## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(packages=['actuation_control'],
                                      package_dir={'': 'src'},
                                     include_package_data=True,
                                      )

setup(**setup_args)
