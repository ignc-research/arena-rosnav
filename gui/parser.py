import os

# IDEA: from 6 txt files make 1 json file
# parse a txt file: https://www.vipinajayakumar.com/parsing-text-with-python/; https://www.pythontutorial.net/python-basics/python-read-text-file/ (from here is the idea with the lines taken)

# check if all 6 files exist and are not empty! (to not trow an error, but print a message if that is the case)
my_path = ['output/data.txt', 'output/obstacle.txt', 'output/watcher.txt', 'output/vector.txt', 'output/robot.txt', 'output/motion.txt']
for my_file in my_path:
    if not (os.path.exists(my_file) and os.path.getsize(my_file) > 0):
        print('File ' + my_file + ' does not exist or is empty!')
        os._exit(0)

print('/**************** parsing data.txt *******************/') # done!
with open('output/data.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines = []
with open('output/data.txt') as file:
    lines = file.readlines()

# Attention: the form of data.txt was slightly changed to make parsing easier
# data from data.txt
image_corners = [] # form: [(x,y), (x,y), (x,y), (x,y)] with left_bottom -> left_up -> right_up -> right_bottom # since paint.py was adjusted, this value is maybe not needed any more :)
pos_scale = [] # form: [pos_scale1_x, pos_scale1_y,pos_scale2_x, pos_scale2_y,pos_scale3_x, pos_scale3_y]
# INFO: pos_scale1 = image_size_before/image_size_after; pos_scale2 = image_size_after/window_size; pos_scale3 = image_size_before/window_size
map_res = 0.0
map_origin = [0.0, 0.0, 0.0] # form: [x,y,z]
obstacle_vel = [] # old form [(obstacle_num, obstacle_vel), ...] # new form [obstacle_0_vel, ...]
obstacle_watcher_connections = [] # idea 1 - not used # form [(obstacle_num, (watcher_num_1, ...), ...)] # OR for example: [(obstacle_num_1, watcher_num_1), (obstacle_num_1, watcher_num_2), (obstacle_num_2, watcher_num_3) ...)]
obstacle_watcher_connections_2 = [] # idea 2 - used # form [(obst0_watch_1, obst0_watch_2, ...),(obst1_watch_1, obst1_watch_2, ...), ...]

# parse the contents into a useful format
count = 0
count_temp = 0
for line in lines:
    count += 1
    #print(f'line {count}: {line}')
    if count > 1 and count < 6:
        image_corners.append((float(line.split(',')[0]), float(line.split(',')[1])))
    if count == 7:
        pos_scale.append(float(line.split(',')[0]))
        pos_scale.append(float(line.split(',')[1]))
        pos_scale.append(float(line.split(',')[2]))
        pos_scale.append(float(line.split(',')[3]))
        pos_scale.append(float(line.split(',')[4]))
        pos_scale.append(float(line.split(',')[5]))
    if count == 9:
        map_res = float(line)
    if count == 11:
        map_origin[0] = float(line.split(',')[0])
        map_origin[1] = float(line.split(',')[1])
        map_origin[2] = float(line.split(',')[2]) # should be 0.0
    if count > 12:
        if line.split('Obstacle-watchers connections:')[0] == '':
            count_temp = count
            break
        else:
            # obstacle_vel.append((float(line.split(':')[0]), float(line.split(':')[1]))) # old version
            obstacle_vel.append(float(line))

count = 0
for line in lines:
    count += 1
    if count > count_temp:
        ## obstacle_watcher_connections.append((float(line.split(':')[0]), (float(line.split(':')[1].split(',')[0]), float(line.split(':')[1].split(',')[1]))))
        temp_array = []
        #for value in line.split(':')[1].split(','): # old version
        #    obstacle_watcher_connections.append((float(line.split(':')[0]), float(value)))
        #    temp_array.append((float(value)))
        for value in line.split(','):
            temp_array.append((int(value)))
        obstacle_watcher_connections_2.append(temp_array) # assume the ordering is right starting from 0
        print('Single obstacle-watchers connections: ' + str(line))

# DEBUGGING
print('Image corners:' + str(image_corners))
#print('First value:' + str(image_corners[0]))
#print('Second value:' + str(image_corners[1]))
print('Position scale:' + str(pos_scale))
#print('First value:' + str(pos_scale[0]))
#print('Second value:' + str(pos_scale[1]))
print('Map resolution:' + str(map_res))
print('Map origin:' + str(map_origin))
print('Obstacle velocities:' + str(obstacle_vel))
#print('First value:' + str(obstacle_vel[0]))
#print('Second value:' + str(obstacle_vel[1]))
print('Obstacle-watchers connections:' + str(obstacle_watcher_connections))
print('Obstacle-watchers connections:' + str(obstacle_watcher_connections_2))

print('/**************** parsing obstacle.txt ***************/') # done!
with open('output/obstacle.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_obstacles = []
with open('output/obstacle.txt') as file:
    lines_obstacles = file.readlines()

# data from obstacle.txt
obstacles = [] # form: [(x_center, y_center, radius, type), ...]

# parse the contents into a useful format
count = 0
for line in lines_obstacles:
    count += 1
    #print(f'line {count}: {line}')
    obstacles.append((float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[0]), float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[1]), float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[2]), line.split('obstacle (x,y,radius,type): ')[1].split(',')[3].split('\n')[0]))

# TODO: what to do with the obstacle type?

# DEBUGGING
print('Obstacles:' + str(obstacles))

print('/**************** parsing watcher.txt ****************/') # done!
with open('output/watcher.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_watchers = []
with open('output/watcher.txt') as file:
    lines_watchers = file.readlines()

# data from watcher.txt
watchers = [] # form: [(x_center, y_center, radius), ...]

# parse the contents into a useful format
count = 0
for line in lines_watchers:
    count += 1
    #print(f'line {count}: {line}')
    watchers.append((float(line.split('watcher (x,y,radius): ')[1].split(',')[0]), float(line.split('watcher (x,y,radius): ')[1].split(',')[1]), float(line.split('watcher (x,y,radius): ')[1].split(',')[2])))

# DEBUGGING
print('Watchers:' + str(watchers))

print('/**************** parsing vector.txt *****************/') # done!
with open('output/vector.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_vectors = []
with open('output/vector.txt') as file:
    lines_vectors = file.readlines()

# data from vector.txt
start_pos = [] # form: [(x, y), ...]
end_pos = [] # form: [(x, y), ...]
waypoints = [] # form: [(x, y), ...] # OR [((x_start, y_start),(x_end, y_end)), ...]

# parse the contents into a useful format
count = 0
for line in lines_vectors:
    count += 1
    #print(f'line {count}: {line}')
    if count % 2 != 0:
        start_pos.append((float(line.split('vector start (x,y): ')[1].split(',')[0]), float(line.split('vector start (x,y): ')[1].split(',')[1])))
    else:
        end_pos.append((float(line.split('vector end (x,y): ')[1].split(',')[0]), float(line.split('vector end (x,y): ')[1].split(',')[1])))

# make each element from start_pos[] and end_pos[] relative to the corresponding element of obstacles[]
i = 0
for obstacle in obstacles: # all three arrays (obstacles[], start_pos[] and end_pos[]) should have the same length (and be in the right order)!
    start_rel_x = start_pos[i][0] - obstacle[0]
    start_rel_y = start_pos[i][1] - obstacle[1]

    end_rel_x = end_pos[i][0] - obstacle[0]
    end_rel_y = end_pos[i][1] - obstacle[1]

    waypoints.append(((start_rel_x, start_rel_y), (end_rel_x, end_rel_y)))
    i += 1

# Transform start and end position to start and waypoints (use relative distances!) -> two ideas! In the json file as start position is meant the obstacle position at the beginning and not the start of the drawn line
# multiple waypoints -> each of them should be relative to the obstacle position

# IDEA 1 (used!): no matter where the line on the window starts, set the start position = obstacle position and set only one waypoint = the relative position of the end of the line in the window to the start position of the obstacle
# => do not use "vector start (x,y)", only "vector end (x,y)"
# => from waypoints = [] with the form [((x_start, y_start),(x_end, y_end)), ...] use only the end points, so waypoints[i][1][0] for x and waypoints[i][1][1] for y

# IDEA 2: the start position = where the obstacle appears before running the code; the waypoint consists of two points, the first is "vector start (x,y)", but converted to be relative to the start position and second is "vector end (x,y)", but again converted
# => both "vector start (x,y)" and "vector end (x,y)" are considered
# => from waypoints = [] with the form [((x_start, y_start),(x_end, y_end)), ...] use everything

# DEBUGGING
print('Start positions:' + str(start_pos))
print('End positions:' + str(end_pos))
print('Waypoints:' + str(waypoints))

print('/**************** parsing robot.txt ****************/') # done!
with open('output/robot.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_robot = []
with open('output/robot.txt') as file:
    lines_robot = file.readlines()

# data from robot.txt
robot = [] # form: [(x_start, y_start),(x_end, y_end)]

# parse the contents into a useful format
count = 0
for line in lines_robot:
    count += 1
    #print(f'line {count}: {line}')
    if count == 1:
        robot.append((float(line.split('robot start position (x,y): ')[1].split(',')[0]), float(line.split('robot start position (x,y): ')[1].split(',')[1])))
    if count == 2:
        robot.append((float(line.split('robot end position (x,y): ')[1].split(',')[0]), float(line.split('robot end position (x,y): ')[1].split(',')[1])))

# DEBUGGING
print('Robot start and end position:' + str(robot))

print('/**************** parsing motion.txt ****************/') # done!
with open('output/motion.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_motion = []
with open('output/motion.txt') as file:
    lines_motion = file.readlines()

# data from robot.txt
motion = [] # form: [motion_obstacle_0, motion-obstacle_1, ...]

# parse the contents into a useful format
count = 0
for line in lines_motion:
    count += 1
    #print(f'line {count}: {line}')
    motion.append(line.split('\n')[0])

# DEBUGGING
print('Motions:' + str(motion))

print('/*********** writing to new_scenario.json ************/')
fob = open('output/new_scenario.json','w') # 'w'/'a' # create a json file
final_string = ''

# TEST!: Before writing the data in the json file, all positions should be modified (scaled/shifted/...)
# consider the position of the image in the window -> make the (0,0) position of the shown image the (0,0) position of the image -> shift!
new_start_point_x = image_corners[0][0]
new_start_point_y = image_corners[0][1]
# scale all positions and all radiuses
# -> use the scale between the image before uploading to the window and the image after that
# -> and the given map resolution scale (so the scale between the image before uploading and the actual 'real/simulation space')
# --> resolution = 0.05 means that 0.05 px is 1 m => ca. 600 pixel -> 600*0.05=30 meter
scale_x = pos_scale[0]*map_res
scale_y = pos_scale[1]*map_res
# check where is the (0,0) point in rviz; for the kivy app was in the bottom left corner!
# TODO NEXT: flip_y_axis or flip_y_axis = -1 (if in rviz the start is in another corner)
flip_x_axis = 1
flip_y_axis = 1
# check if the start point should be actually (0,0) or some other point -> shift all positions by that value (after all of the scaling has already been done!)
shift_x = map_origin[0]
shift_y = map_origin[1]
# Info: map_small for example has a border on all sides -> the beginning of the border will be then the origin!
# Info: the center of the obstacle reaches the goal position (the waypoint), not the nearest part of the obstacle!

obstacles_json = ''
i = 0
for obstacle in obstacles:
    obstacle_num = str(i)
    # TODO NEXT: should the radius be scaled by scale_x or scale_y -> it should be a circle after all !?
    obstacle_radius = str(obstacle[2]*scale_x) # good with obstacle[2]*scale_x (should be right!, since the radius is a distace and it was made according to the image size after, but taken as a image size before!)
    lin_velocity = str(obstacle_vel[i]) # before was obstacle_vel[i][1]
    start_pos_x = str((obstacle[0]-new_start_point_x)*scale_x*flip_x_axis+shift_x) # good with (obstacle[0]-new_start_point_x)*scale_x*flip_x_axis+shift_x
    start_pos_y = str((obstacle[1]-new_start_point_y)*scale_y*flip_y_axis+shift_y)
    waypoint_x = str((waypoints[i][1][0])*scale_x*flip_x_axis) # the waypoints are relative to the obstacle position! -> new_start_point and shift should be irrelevant!
    waypoint_y = str((waypoints[i][1][1])*scale_y*flip_y_axis) # scale and flip? are only relevant!
    watcher_num = str(i)
    move = str(motion[i])

    # since the order of the obstacles is given in the window, it is here assumed that the obstacle velocities and the obstacle-watchers connections are written in the right order
    obstacles_json += '\n\t\t\t\t"dynamic_obs_' + obstacle_num + '": {\n\t\t\t\t\t'
    obstacles_json += '"obstacle_radius": ' + obstacle_radius + ',\n\t\t\t\t\t'
    obstacles_json += '"linear_velocity": ' + lin_velocity + ',\n\t\t\t\t\t'
    obstacles_json += '"start_pos": [\n\t\t\t\t\t\t' + start_pos_x + ',\n\t\t\t\t\t\t' + start_pos_y + ',\n\t\t\t\t\t\t0'
    obstacles_json += '\n\t\t\t\t\t],\n\t\t\t\t\t"waypoints": [\n\t\t\t\t\t\t[\n\t\t\t\t\t\t\t'
    obstacles_json += waypoint_x + ',\n\t\t\t\t\t\t\t' + waypoint_y + ',\n\t\t\t\t\t\t\t0' # Idea 1 for the waypoints (see above for the explanaition)
    obstacles_json += '\n\t\t\t\t\t\t]\n\t\t\t\t\t],\n\t\t\t\t\t"is_waypoint_relative": true,\n\t\t\t\t\t'
    obstacles_json += '"mode": "' + move + '"'
    obstacles_json += ',\n\t\t\t\t\t"triggers": [\n\t\t\t\t\t\t'

    # since now we have obstacle-watchers connections -> we do not have to take the obstacles and watchers just line by line, but the coresponding lines! -> use the info from obstacle_watcher_connections or obstacle_watcher_connections_2
    #obstacles_json += '"watcher_' + watcher_num + '"'
    j = 0
    for watcher in obstacle_watcher_connections_2[i]:
        obstacles_json += '"watcher_' + str(watcher) + '"'
        if j < (len(obstacle_watcher_connections_2[i]) - 1):
            obstacles_json += ', '
        j += 1
    
    obstacles_json += '\n\t\t\t\t\t]\n\t\t\t\t}'
    if i != len(obstacles) - 1:
        obstacles_json += ','
    i += 1

watchers_json = ''
i = 0
for watcher in watchers:
    watcher_num = str(i)
    watcher_pos_x = str((watcher[0]-new_start_point_x)*scale_x*flip_x_axis+shift_x)
    watcher_pos_y = str((watcher[1]-new_start_point_y)*scale_y*flip_y_axis+shift_y)
    watcher_radius = str(watcher[2]*scale_x)

    watchers_json += '\n\t\t\t\t"watcher_' + watcher_num + '": {\n\t\t\t\t\t"pos": ['
    watchers_json += '\n\t\t\t\t\t\t' + watcher_pos_x + ',\n\t\t\t\t\t\t' + watcher_pos_y + '\n\t\t\t\t\t],'
    watchers_json += '\n\t\t\t\t\t"range": ' + watcher_radius + '\n\t\t\t\t}'
    if i != len(watchers) - 1:
        watchers_json += ','
    i += 1

robot_start_x = str((robot[0][0]-new_start_point_x)*scale_x*flip_x_axis+shift_x)
robot_start_y = str((robot[0][1]-new_start_point_y)*scale_y*flip_y_axis+shift_y)
robot_end_x = str((robot[1][0]-new_start_point_x)*scale_x*flip_x_axis+shift_x)
robot_end_y = str((robot[1][1]-new_start_point_y)*scale_y*flip_y_axis+shift_y)
robot_json = ''
robot_json += '\n\t\t\t\t"start_pos": ['
robot_json += '\n\t\t\t\t\t' + robot_start_x + ',\n\t\t\t\t\t' + robot_start_y + ',\n\t\t\t\t\t0.0\n\t\t\t\t],'
robot_json += '\n\t\t\t\t"goal_pos": ['
robot_json += '\n\t\t\t\t\t' + robot_end_x + ',\n\t\t\t\t\t' + robot_end_y + ',\n\t\t\t\t\t0.0\n\t\t\t\t]'

# make the basic structure of the json file
final_string += '{\n\t"scenerios": [\n\t\t{\n\t\t\t"scene_name": "new_scenario",\n\t\t\t"repeats": 5,\n\t\t\t"dynamic_obstacles": {'
final_string += obstacles_json
final_string += '\n\t\t\t},\n'
final_string += '\t\t\t"static_obstacles": {\n\n\t\t\t},\n\t\t\t"robot": {'
final_string += robot_json
final_string += '\n\t\t\t},\n\t\t\t"watchers": {'
final_string += watchers_json
final_string += '\n\t\t\t}\n\t\t}\n\t]\n}'

fob.write(final_string)
fob.close()

print('/**************** new_scenario.json *****************/')
# print the final version of the json file
with open('output/new_scenario.json') as file:
    file_contents = file.read()
    print(file_contents) # still empty

# when ready -> validate the json: https://jsonformatter.curiousconcept.com/

# TODO NEXT notes:
# !0) comment the code, clear out the prints, make a readme file with all the rules, make a video explaining each step, push the code in a new branch in git
# !1) allow different types of obstacles -> mark them with different colors -> Question: where does the type goes in the json file?
# 2) sometimes an error occurs, when it is clicked somewhere in between?
# 3) different motions per obstacle -> input field vs. dropdown box
# 4) make the window not resizable or resizable (but everyhting should still work fine!)
# 5) obstacles animation, dynamisch hin und her bewegen
# -> man kann am Rand eine hard coded Simulation in demselben map von einem obstacle mit Geschwindigkeit zB 0.3 visualizieren, damit der Nutzer Gefuehl bekommen kann, wie schnell das eigentlich ist
# ?6) Rand von watcher zu rot machen, wenn es zu srash wird; die Robotergeschwindigkeit ist hard coded zu 0.3, man kann also berechnen, wann wird der ROboter der watcher aktivieren; also weiteres, letztes Button "simulate if colision"; man braucht dafuer aber noch global path -> nur mit box2d sim engine machbar

# TODO TEST the code with different maps in rviz:
# - tested with map_small.png and map.png and it works!
# ?- from the console ros topic list echo set goal; in rviz from the map check (x,y)
