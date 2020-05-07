function cleanup {
    screen -S visdom -X quit
}

screen -S visdom -dm visdom
# Give visdom some time to start up
sleep 2
python3 main.py

trap cleanup EXIT
