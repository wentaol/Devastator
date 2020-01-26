function cleanup {
	echo Killing camera stream...
    screen -S visdom -X quit
    kill -9 $STREAM_PID
    kill -9 $DIST_PID
}

screen -S visdom -dm visdom
# Give visdom some time to start up
sleep 2
python cam_stream.py &
STREAM_PID=$!
echo "Camera stream started at PID:$STREAM_PID"
python dist_stream.py &
DIST_PID=$!
echo "Distance sensor stream started at PID:$DIST_PID"

sleep 2
#python joypad_control.py
while [ true ]
do
    sleep 2
done

trap cleanup EXIT
