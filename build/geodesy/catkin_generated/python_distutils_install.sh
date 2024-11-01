#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/arob/catkin_ws/src/geographic_info/geodesy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arob/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arob/catkin_ws/install/lib/python3/dist-packages:/home/arob/catkin_ws/build/geodesy/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arob/catkin_ws/build/geodesy" \
    "/usr/bin/python3" \
    "/home/arob/catkin_ws/src/geographic_info/geodesy/setup.py" \
    egg_info --egg-base /home/arob/catkin_ws/build/geodesy \
    build --build-base "/home/arob/catkin_ws/build/geodesy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/arob/catkin_ws/install" --install-scripts="/home/arob/catkin_ws/install/bin"
