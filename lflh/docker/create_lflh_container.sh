docker run -it -d --rm --name="barn_lflh" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${HOME}/Repo/inventec-team-barn-challenge-2023/lflh:/root/jackal_ws/src/lflh/:rw" \
    --volume="${HOME}/Repo/inventec-team-barn-challenge-2023/nav-competition-icra2023:/root/jackal_ws/src/nav-competition-icra2023/:rw" \
    ros/melodic:barn-lflh
