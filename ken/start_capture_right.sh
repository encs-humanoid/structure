#!/bin/bash
# This is a kickstart file to automatically start capturing from the
# camera when launching the raspicam node.
# See http://answers.ros.org/question/11271/call-a-service-at-startup-with-a-launch-file/
rosservice call --wait /stereo/right/camera/start_capture &
exec "$@"
