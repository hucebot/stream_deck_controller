# For graphics

isRunning=`docker ps -f name=stream_deck | grep -c "stream_deck"`;

if [ $isRunning -eq 0 ]; then
    xhost +local:docker
    docker rm stream_deck_container
    docker run \
        --name stream_deck_container \
        -it \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="PACKAGES_DIR=`pwd`/../" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --net host \
        --privileged \
        -v /dev:/dev \
        -v `pwd`/../:/home/forest_ws/src/stream_deck_controller \
        -v `pwd`/../../demo_record_utils/postprocess:/home/forest_ws/src/stream_deck_controller/src/utils/postprocess \
        -v /var/run/docker.sock:/var/run/docker.sock \
        -w /home/forest_ws \
        stream_deck:latest

else
    echo "Stream Deck container is already running"
    docker exec -it stream_deck_container /bin/bash
fi

