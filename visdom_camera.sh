function cleanup {
	echo Killing camera stream...
    screen -S visdom -X quit
    kill -9 $STREAM_PID
}

screen -S visdom -dm visdom
sleep 2 
python cam_stream.py
STREAM_PID=$!
echo "Camera stream started at PID:$STREAM_PID"

trap cleanup EXIT
