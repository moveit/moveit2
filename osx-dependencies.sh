#!/bin/bash

#FROM https://index.ros.org/doc/ros2/Installation/OSX-Development-Setup/

brew doctor
brew install cppcheck pcre poco tinyxml openssl
brew install asio tinyxml2
brew install log4cxx
brew install assimp
brew install qhull
brew install boost
python3 -m pip install argcomplete catkin_pkg colcon-common-extensions coverage empy flake8 flake8-blind-except \
flake8-builtins flake8-class-newline flake8-comprehensions flake8-deprecated flake8-docstrings flake8-import-order flake8-quotes \
git+https://github.com/lark-parser/lark.git@0.7d mock nose pep8 pydocstyle pyparsing setuptools vcstool
