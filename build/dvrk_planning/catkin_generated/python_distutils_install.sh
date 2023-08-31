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

echo_and_run cd "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/arionlaw/Documents/ContinuumRobotModel/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/arionlaw/Documents/ContinuumRobotModel/install/lib/python3/dist-packages:/home/arionlaw/Documents/ContinuumRobotModel/build/dvrk_planning/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/arionlaw/Documents/ContinuumRobotModel/build/dvrk_planning" \
    "/usr/bin/python3" \
    "/home/arionlaw/Documents/ContinuumRobotModel/src/dvrk_planning/dvrk_planning/setup.py" \
     \
    build --build-base "/home/arionlaw/Documents/ContinuumRobotModel/build/dvrk_planning" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/arionlaw/Documents/ContinuumRobotModel/install" --install-scripts="/home/arionlaw/Documents/ContinuumRobotModel/install/bin"
