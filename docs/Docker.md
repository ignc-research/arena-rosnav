## Introduction
There are two containers in one docker network. One of them contains the ros melodic environment and all dependencies of arena-rosnav. The other one is used to forward the graphic siganl of the previous container to make GUI such as rviz visible in the browser.

## Install docker
Ubuntu:
https://cnvrg.io/how-to-setup-docker-and-nvidia-docker-2-0-on-ubuntu-18-04/

Mac:
https://docs.docker.com/docker-for-mac/install/

Windows:
https://docs.docker.com/docker-for-windows/install/


## Create containers
1. Clone the repository
```
git clone https://github.com/ignc-research/arena-rosnav
```
2. Go into the arena-rosnav folder
```
cd arena-rosnav
```
3. Create network for two containers
```
docker network create -d bridge x11
```
4. Run the first container for ros
```
docker run -d \
--name ros \
--network=x11 \
-e DISPLAY=novnc:0.0 \
-v $PWD:/root/catkin_ws/src/arena-rosnav \
llalal/docker:arena-rosnav \
tail -f /dev/null
```
5. Run the second container for display
```
docker run -d \
--name novnc \
--network=x11 \
-e DISPLAY_WIDTH=1920 \
-e DISPLAY_HEIGHT=1080 \
-e RUN_XTERM=no \
-p 8080:8080 \
theasp/novnc:latest
```
6. Run `docker ps` to check if two containers are already running

## Enter and exit ros container
1. Enter the ros container
```
docker exec -it ros /bin/zsh
```
For cases where multiple terminals need to be opened for testing, open multiple local terminals and enter the container with the same command (It's better to use VS Code, which is more convenient. The instruction is at the end)

2. Exit the container
Input `exit` in the command line
1. Stop the container (exit the container firstly)
```
docker stop ros
```
4. Restart the container if it exists already
```
docker restart ros
```

## Show GUI in browser
1. Open http://localhost:8080/vnc.html (If you run docker on the remote host, open http://YOUR_HOST_IP_ADDRESS:8080/vnc.html )
2. You can adjust the window size to automatic local scale or set fullscreen through the side bar on the left


## Develop in VS Code
1. Necessary extensions in VS Code  
   + Remote Development  
   + Docker  
2. After executing steps in "Create containers" part, enter the container by VS Code  
    a) Open Command Palette by `F1` or `CMD + Shift + p` (windows `Ctrl + Shift + p`)  
    b) Input `Attach to Running Container` and select Remote-Containers: Attach to Running Container  
    c) Then select container named /ros  
   Now the VS Code will open a new window  
    d) Select and open the folder you want by `CMD + o` (windows `Ctrl + o`)

