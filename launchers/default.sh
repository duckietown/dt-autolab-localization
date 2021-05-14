#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# YOUR CODE BELOW THIS LINE
# ----------------------------------------------------------------------------


# NOTE: Use the variable DT_REPO_PATH to know the absolute path to your code
# NOTE: Use `dt-exec COMMAND` to run the main process (blocking process)

# make the `aruco_python.so` library discoverable
export PYTHONPATH=${PYTHONPATH}:${CATKIN_WS_DIR}/devel/lib

# launching app
dt-exec roslaunch --wait autolab-localization default.launch


# ----------------------------------------------------------------------------
# YOUR CODE ABOVE THIS LINE

# wait for app to end
dt-launchfile-join
