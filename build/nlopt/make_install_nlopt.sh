#!/bin/sh

DESTDIR=/home/arob/catkin_ws/build/nlopt/nlopt_install make install

cp -r /home/arob/catkin_ws/build/nlopt/nlopt_install//home/arob/catkin_ws/install/* /home/arob/catkin_ws/devel/.private/nlopt/
