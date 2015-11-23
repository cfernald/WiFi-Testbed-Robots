#!/bin/sh
for var in "$@"
do
echo $var
    if [ "$var" -lt 60 ]; then
        expect _send_waypoints $USER $var ut longhorn
    else
        expect _send_waypoints $USER $var siavash 123456
    fi
done
