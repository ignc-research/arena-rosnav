# Install docker
https://docs.docker.com/docker-for-mac/install/

## Build image and setup container
1. Download the files in this folder to local
2. Open local terminal at the path where you download this files
3. Run `docker-compose up --build`

## Start/Enter/Stop the container
1. Check if the arena-rosnav and novnc container is already started
```
docker ps
```
2. Start container (run in the path containing docker-compose.yml)
```
docker-compose start
```
2. Enter the ros contaier 
```
docker exec -it arena-rosnav bash
```
4. Stop the container (run in the path containing docker-compose.yml)cd 
```
docker-compose stop 
```

## Show the GUI app in ROS by browser
1. Open 'http://localhost:8080/vnc.html'

## Adjust the size of window shown on browser
1. If you need change the size of window in browser, you can change parameters in novnc.env

## Test
1. After entering the container by `docker exec -it arena-rosnav bash` activate the venv by
```
workon rosnav
```
2. Roslaunch in venv
```
roslaunch arena_bringup start_arena_flatland.launch train_mode:=false use_viz:=true local_planner:=dwa map_file:=map1 obs_vel:=0.3

```
3. Open http://localhost:8080/vnc.html you will see the rviz window in browser
3. By setting a 2D Nav Goal mannualy on white goal point you will see the robot move to the goal point automaticlly

## Develop in VS Code
1. Install Plugin Remote - Containers
2. Click the green icon in the lower left corner (or type `F1`) and input `Remote-Containers: Reopen in Container`
3. Select Docker from Docker Compose 