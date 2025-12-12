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

echo_and_run cd "/home/pjw/turtlebot_ws/src/robot_simulations/turtlebot/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/pjw/turtlebot_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/pjw/turtlebot_ws/install/lib/python3/dist-packages:/home/pjw/turtlebot_ws/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/pjw/turtlebot_ws/build" \
    "/usr/bin/python3" \
    "/home/pjw/turtlebot_ws/src/robot_simulations/turtlebot/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/pjw/turtlebot_ws/build/robot_simulations/turtlebot/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/pjw/turtlebot_ws/install" --install-scripts="/home/pjw/turtlebot_ws/install/bin"
