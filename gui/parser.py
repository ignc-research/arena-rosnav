import os
from PIL import Image # if necessary run $ python3 -m pip install --upgrade Pillow
from shutil import copyfile
# it also may be necessary to install xclip if an error in the console is received -> do $ sudo apt-get install xclip

# IDEA: from txt files make 1 json file
# parse a txt file: https://www.vipinajayakumar.com/parsing-text-with-python/; https://www.pythontutorial.net/python-basics/python-read-text-file/ (from here is the idea with the lines taken)

print('\nSTART parser.py\n')

# check if all txt files exist and are not empty! (to not trow an error, but print a message if that is the case)
# 'output/internal/obstacle.txt', 'output/internal/watcher.txt' and 'output/internal/vector.txt' could be empty if no obstacles/wathers/wayypoints are used at all
# if 'output/internal/obstacle.txt' is empty, 'output/internal/motion.txt' will be also empty
my_path = ['output/internal/data.txt', 'output/internal/robot.txt']
for my_file in my_path:
    if not (os.path.exists(my_file) and os.path.getsize(my_file) > 0):
        print('ERROR: File ' + my_file + ' does not exist or is empty!')
        os._exit(0)

print('/**************** parsing data.txt *******************/') # done!
with open('output/internal/data.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines = []
with open('output/internal/data.txt') as file:
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
obstacle_waypoint_connections = [] # see obstacle_watcher_connections_2
amount_pedestrians = [] # form: [amount_group1, amount_group2, ...]
chatting_probability = [] # form: see above
obstacle_force_factor = [] # form: see above
desire_force_factor = [] # form: see above

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
            if line != '' and line != '\n':
                # obstacle_vel.append((float(line.split(':')[0]), float(line.split(':')[1]))) # old version
                obstacle_vel.append(float(line))
            else:
                obstacle_vel.append(0.01) # TODO: 0.01 velocity for a static obstacle (it does not work with velocity = 0.0)
                print('A really small default obstacle velocity of "0.01" will be used.')

count_obstacles = count_temp - 12 - 1 # 12 not dynamic lines # 1 label 'Obstacle-watchers connections:'
count = 0
no_pedestrians = 0
for line in lines:
    count += 1
    if count > count_temp and count <= count_temp+count_obstacles:
        ## obstacle_watcher_connections.append((float(line.split(':')[0]), (float(line.split(':')[1].split(',')[0]), float(line.split(':')[1].split(',')[1]))))
        temp_array = []
        #for value in line.split(':')[1].split(','): # old version
        #    obstacle_watcher_connections.append((float(line.split(':')[0]), float(value)))
        #    temp_array.append((float(value)))
        for value in line.split(','):
            # It is not a must that an obstacle has a watcher -> append '/' when this is the case instead of a watcher index.
            # Later on it don't have to be considered as a watcher and calculated to the watchers amount.
            if value == '' or value == '\n': # it could be '\n' since in the txt file a line is reserved for the watcher index
                print('No watchers for obstacle ' + str(count-count_temp-1))
                temp_array.append(('/'))
            else:
                temp_array.append((int(value)))
        obstacle_watcher_connections_2.append(temp_array) # assume the ordering is right starting from 0
        print('Single obstacle-watchers connections: ' + str(line.split('\n')[0]))
    if count > count_temp+count_obstacles+1 and count <= count_temp+2*count_obstacles+1:
        temp_array = []
        for value in line.split(','):
            # The GUI implements dynamic obstacles => minimum one waypoint per obstacle should be always given
            if value == '' or value == '\n': # it could be '\n' since in the txt file a line is reserved for the waypoint index
                # Variant 1: if only dynamic obstacles should be allowed, comment out the following two lines!
                #print('ERROR: All obstacles schould be dynamic, so a minimum of one waypoint per obstacle is required!')
                #os._exit(0)
                # Variant 2: allow not only dynamic obstacles, but also static obstacles (they do not need waypoints)
                print('No waypoints for obstacle ' + str(count-count_temp-count_obstacles-2))
                temp_array.append(('/'))
            else:
                temp_array.append((int(value)))
        obstacle_waypoint_connections.append(temp_array) # assume the ordering is right starting from 0
        print('Single obstacle-waypoints connections: ' + str(line.split('\n')[0]))
    if count > count_temp+2*(count_obstacles+1) and line.split('Obstacle force factor:')[0] == '':
        no_pedestrians = 1
    if count > count_temp+2*(count_obstacles+1) and count <= count_temp+2*(count_obstacles+1)+count_obstacles and no_pedestrians == 0:
        amount_pedestrians.append((int(line.split('\n')[0])))
    if count > count_temp+2*(count_obstacles+1) and count <= count_temp+2*(count_obstacles+1)+count_obstacles and no_pedestrians == 1:
        obstacle_force_factor.append((float(line.split('\n')[0])))
    if count > count_temp+3*(count_obstacles+1) and count <= count_temp+3*(count_obstacles+1)+count_obstacles and no_pedestrians == 0:
        chatting_probability.append((float(line.split('\n')[0])))
    if count > count_temp+3*(count_obstacles+1) and count <= count_temp+3*(count_obstacles+1)+count_obstacles and no_pedestrians == 1:
        desire_force_factor.append((float(line.split('\n')[0])))
    if count > count_temp+4*(count_obstacles+1) and count <= count_temp+4*(count_obstacles+1)+count_obstacles:
        obstacle_force_factor.append((float(line.split('\n')[0])))
    if count > count_temp+5*(count_obstacles+1) and count <= count_temp+5*(count_obstacles+1)+count_obstacles:
        desire_force_factor.append((float(line.split('\n')[0])))

# check if the amount of obstacles given in the text area, is the same as the actually drawn, also as the same as the drawn lines etc.
# obstacle.txt_lines == vector.txt_lines/2 == motion.txt_lines == data.txt_obstacle-vel.lines == data.txt_obst-watch-con.lines
length_right = len(obstacle_vel) # == len(obstacle_watcher_connections_2)

# calculate the amount of all watchers given in the text fileds
watchers_all_amount = 0
for connection in obstacle_watcher_connections_2:
    for watcher in connection:
        if watcher != '/': # this means no connection
            watchers_all_amount += 1

# calculate the amount of all waypoints given in the text fileds
waypoints_all_amount = 0
for connection in obstacle_waypoint_connections:
    for waypoint in connection:
        if waypoint != '/': # this means no connection
            waypoints_all_amount += 1

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
#print('Obstacle-watchers connections:' + str(obstacle_watcher_connections))
print('Obstacle-watchers connections:' + str(obstacle_watcher_connections_2))
print('Obstacle-waypoints connections:' + str(obstacle_waypoint_connections))
print('Amount of pedestrians:' + str(amount_pedestrians))
print('Chatting probability:' + str(chatting_probability))
print('Obstacle force factor:' + str(obstacle_force_factor))
print('Desire force factor:' + str(desire_force_factor))

print('/**************** parsing obstacle.txt ***************/') # done!
lines_obstacles = []
# since obstacle.txt could be empty and this is valid, with it should be dealed differently
obstacle_file = 'output/internal/obstacle.txt'
if not (os.path.exists(obstacle_file) and os.path.getsize(obstacle_file) > 0):
    print('File ' + obstacle_file + ' does not exist or is empty! No obstacles at all are used.')
else:
    with open('output/internal/obstacle.txt') as file:
        file_contents = file.read()
        print(file_contents)

    with open('output/internal/obstacle.txt') as file:
        lines_obstacles = file.readlines()

if len(lines_obstacles) != length_right:
    print('ERROR: Unmatching amount of obstacles given in the text field and drawn on the map! The scenario is incorrect, make a new one!')
    os._exit(0)

# data from obstacle.txt
obstacles = [] # form: [(x_center, y_center, radius, type), ...]

# parse the contents into a useful format
count = 0
for line in lines_obstacles:
    count += 1
    obstacles.append((float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[0]), float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[1]), float(line.split('obstacle (x,y,radius,type): ')[1].split(',')[2]), line.split('obstacle (x,y,radius,type): ')[1].split(',')[3].split('\n')[0]))

# DEBUGGING
print('Obstacles:' + str(obstacles))

print('/**************** parsing watcher.txt ****************/') # done!
lines_watchers = []
drawn_watchers_amount = 0
# data from watcher.txt
watchers = [] # form: [(x_center, y_center, radius), ...]

# since watcher.txt could be empty and this is valid, with it should be dealed differently
watcher_file = 'output/internal/watcher.txt'
if not (os.path.exists(watcher_file) and os.path.getsize(watcher_file) > 0):
    print('File ' + watcher_file + ' does not exist or is empty! No watchers at all are used.')
else:
    with open('output/internal/watcher.txt') as file:
        file_contents = file.read()
        print(file_contents)

    with open('output/internal/watcher.txt') as file:
        lines_watchers = file.readlines()

for line in lines_watchers:
    if line != '' and line != '\n':
        drawn_watchers_amount += 1

# check if the amount of watchers drawn on the map is the same as the amount of all watchers from the obstacle-watchers-connections given in the text fields
if watchers_all_amount != drawn_watchers_amount:
    print('ERROR: Unmatching amount of watchers given in the text fields and drawn on the map! The scenario is incorrect, make a new one!')
    os._exit(0)

# parse the contents into a useful format
count = 0
for line in lines_watchers:
    count += 1
    watchers.append((float(line.split('watcher (x,y,radius): ')[1].split(',')[0]), float(line.split('watcher (x,y,radius): ')[1].split(',')[1]), float(line.split('watcher (x,y,radius): ')[1].split(',')[2])))

# DEBUGGING
print('Watchers:' + str(watchers))

print('/**************** parsing triggered_watchers.txt *****************/') # done!
lines_triggered_watchers = []
triggered_watchers = []
if os.path.exists("output/internal/triggered_watchers.txt") and os.path.getsize("output/internal/triggered_watchers.txt") > 0: # only if exists and not empty
    with open('output/internal/triggered_watchers.txt') as file:
        lines_triggered_watchers = file.readlines()

for line in lines_triggered_watchers:
    triggered_watchers.append(int(line))

# DEBUGGING
print('Triggered watchers:' + str(triggered_watchers))

print('/**************** parsing vector.txt *****************/') # done!
lines_vectors = []
# data from vector.txt
start_pos = [] # form: [(x, y), ...]
end_pos = [] # form: [(x, y), ...]
waypoints = [] # form: [(x, y), ...] # OR [((x_start, y_start),(x_end, y_end)), ...]

# since vector.txt could be empty and this is valid, with it should be dealed differently
vector_file = 'output/internal/vector.txt'
if not (os.path.exists(vector_file) and os.path.getsize(vector_file) > 0):
    print('File ' + vector_file + ' does not exist or is empty! No waypoints at all are used. All obstacles are static.')
else:
    with open('output/internal/vector.txt') as file:
        file_contents = file.read()
        print(file_contents)

    with open('output/internal/vector.txt') as file:
        lines_vectors = file.readlines()

## for when it is allowed only one waypoint per obstacle:
#if int(len(lines_vectors)/2) != length_right:
#    print('ERROR: Unmatching amount of obstacles and waypoints! The scenario is incorrect, make a new one!')
#    os._exit(0)

# check if the amount of waypoints drawn on the map is the same as the amount of all waypoints from the obstacle-waypoints-connections given in the text fields
if waypoints_all_amount != int(len(lines_vectors)/2):
    print('ERROR: Unmatching amount of waypoints given in the text fields and drawn on the map! The scenario is incorrect, make a new one!')
    os._exit(0)

# parse the contents into a useful format
count = 0
for line in lines_vectors:
    count += 1
    if count % 2 != 0:
        start_pos.append((float(line.split('vector start (x,y): ')[1].split(',')[0]), float(line.split('vector start (x,y): ')[1].split(',')[1])))
    else:
        end_pos.append((float(line.split('vector end (x,y): ')[1].split(',')[0]), float(line.split('vector end (x,y): ')[1].split(',')[1])))

# make each element from start_pos[] and end_pos[] relative to the corresponding element of obstacles[]
# all three arrays (obstacles[], start_pos[] and end_pos[]) should have the same length (and be in the right order)!
# Important: pair the waypoint to the obstacle, so that the right relative distance can be calculated, use the data from data.txt (obstacle_waypoint_connections[])
i = 0
for obstacle in obstacles: # len(obstacles)=len(obstacle_waypoint_connections)
    for waypoint_index in obstacle_waypoint_connections[i]:
        if waypoint_index != '/':
            start_rel_x = start_pos[waypoint_index][0] - obstacle[0]
            start_rel_y = start_pos[waypoint_index][1] - obstacle[1]
        
            end_rel_x = end_pos[waypoint_index][0] - obstacle[0]
            end_rel_y = end_pos[waypoint_index][1] - obstacle[1]
        
            waypoints.append(((start_rel_x, start_rel_y), (end_rel_x, end_rel_y)))
        else:
            waypoints.append((('/', '/'), ('/', '/'))) # only as a place holder
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
with open('output/internal/robot.txt') as file:
    file_contents = file.read()
    print(file_contents)

lines_robot = []
with open('output/internal/robot.txt') as file:
    lines_robot = file.readlines()

if len(lines_robot) != 2:
    print('ERROR: Not complete information amount the robot start and end position! The scenario is incorrect, make a new one!')
    os._exit(0)

# data from robot.txt
robot = [] # form: [(x_start, y_start),(x_end, y_end)]

# parse the contents into a useful format
count = 0
for line in lines_robot:
    count += 1
    if count == 1:
        robot.append((float(line.split('robot start position (x,y): ')[1].split(',')[0]), float(line.split('robot start position (x,y): ')[1].split(',')[1])))
    if count == 2:
        robot.append((float(line.split('robot end position (x,y): ')[1].split(',')[0]), float(line.split('robot end position (x,y): ')[1].split(',')[1])))

# DEBUGGING
print('Robot start and end position:' + str(robot))

print('/**************** parsing motion.txt ****************/') # done!
lines_motion = []
# since motion.txt could be empty and this is valid, with it should be dealed differently
motion_file = 'output/internal/motion.txt'
if not (os.path.exists(motion_file) and os.path.getsize(motion_file) > 0):
    print('File ' + motion_file + ' does not exist or is empty! No obstacles at all are used.')
else:
    with open('output/internal/motion.txt') as file:
        file_contents = file.read()
        print(file_contents)

    with open('output/internal/motion.txt') as file:
        lines_motion = file.readlines()

# needed only if only dynamic obstacles are allowed, because then for every obstacle should a motion be specified, but for a static obstacle it is not needed
if len(lines_motion) != length_right or len(obstacle_vel) != length_right: # or len(obstacle_watcher_connections_2) != length_right:
    print('ERROR: Unmatching amount of obstacles and motions! The scenario is incorrect, make a new one!')
    os._exit(0)

# data from robot.txt
motion = [] # form: [motion_obstacle_0, motion-obstacle_1, ...]

motions_valid = ["yoyo", "circle", "once", "loop", "random"] # Important: extend the valid motion types if needed

# parse the contents into a useful format
count = 0
for line in lines_motion:
    count += 1
    # check if the given motion type is valid, otherwise terminate and print an error!
    motion_given = line.split('\n')[0]
    valid = 0
    if motion_given == '': # TODO: if the obstacle is static, no motion is necessary -> still it may need a default value to work
        valid = 0.5
    else:
        for motion_valid in motions_valid:
            if motion_given == motion_valid:
                valid = 1
                break
    if valid == 1:
        motion.append(motion_given)
    elif valid == 0.5:
        motion.append('yoyo') # TODO: a default value for the motion, if no motion was given (important for handling static obstacles)
        print('The default motion type "yoyo" will be used.')
    else:
        print('ERROR: "' + motion_given + '" is an invalid motion type!')
        os._exit(0)

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
# Attention: flip_y_axis or flip_y_axis = -1 (if in rviz the start is in another corner)
flip_x_axis = 1
flip_y_axis = 1
# check if the start point should be actually (0,0) or some other point -> shift all positions by that value (after all of the scaling has already been done!)
shift_x = map_origin[0]
shift_y = map_origin[1]
# Info: map_small for example has a border on all sides -> the beginning of the border will be then the origin!
# Info: the center of the obstacle reaches the goal position (the waypoint), not the nearest part of the obstacle!

obstacles_json = ''
k = 0
i = 0
if len(obstacles) == 0:
    obstacles_json += '\n'
for obstacle in obstacles:
    obstacle_num = str(i)
    # Important: because of how the gui works, scale_x and scale_y will always be the same, which is really good, because we need for example to scale the obstacle_radius, so that at the end the obstacle stays a circle and not an ellipse!
    # -> that is why it does not matter if we use for the obstacle_radius scale_x or scale_y
    obstacle_radius = str(obstacle[2]*scale_x) # good with obstacle[2]*scale_x (should be right!, since the radius is a distace and it was made according to the image size after, but taken as a image size before!)
    lin_velocity = str(obstacle_vel[i]) # before was obstacle_vel[i][1]
    start_pos_x = str((obstacle[0]-new_start_point_x)*scale_x*flip_x_axis+shift_x) # good with (obstacle[0]-new_start_point_x)*scale_x*flip_x_axis+shift_x
    start_pos_y = str((obstacle[1]-new_start_point_y)*scale_y*flip_y_axis+shift_y)
    watcher_num = str(i)
    move = str(motion[i])
    obstacle_type = str(obstacle[3]) # obstacle default type = "circle"
    # take care of the additional parameters:
    if len(amount_pedestrians) > 0:
        amount = str(amount_pedestrians[i])
        chat_probability = str(chatting_probability[i])
    if len(obstacle_force_factor) > 0:
        obstactle_force_fact = str(obstacle_force_factor[i])
        desire_force_fact = str(desire_force_factor[i])

    # since the order of the obstacles is given in the window, it is here assumed that the obstacle velocities and the obstacle-watchers connections are written in the right order
    obstacles_json += '\n\t\t\t\t"dynamic_obs_' + obstacle_num + '": {\n\t\t\t\t\t'
    obstacles_json += '"obstacle_radius": ' + obstacle_radius + ',\n\t\t\t\t\t'
    obstacles_json += '"linear_velocity": ' + lin_velocity + ',\n\t\t\t\t\t'
    obstacles_json += '"type": "' + obstacle_type + '",\n\t\t\t\t\t'
    if len(amount_pedestrians) > 0:
        obstacles_json += '"amount": ' + amount + ',\n\t\t\t\t\t'
        obstacles_json += '"chatting_probability": ' + chat_probability + ',\n\t\t\t\t\t'
    if len(obstacle_force_factor) > 0:
        obstacles_json += '"obstacle_force_factor": ' + obstactle_force_fact + ',\n\t\t\t\t\t'
        obstacles_json += '"desire_force_factor": ' + desire_force_fact + ',\n\t\t\t\t\t'
    obstacles_json += '"start_pos": [\n\t\t\t\t\t\t' + start_pos_x + ',\n\t\t\t\t\t\t' + start_pos_y + ',\n\t\t\t\t\t\t0'
    obstacles_json += '\n\t\t\t\t\t],\n\t\t\t\t\t"waypoints": [\n'

    # since now we have obstacle-waypoints connections -> we do not have to take waypoints just line by line, but the coresponding lines! -> use the info from obstacle_waypoint_connections
    # Idea 1 for the waypoints (see above for the explanation)
    count_temp = 0
    for waypoint in obstacle_waypoint_connections[i]: # use it just as counter = waypoints per obstacle, since in "waypoints[]" the waypoints are already sorted by obstacles
        if waypoint != '/':
            waypoint_x = str((waypoints[k][1][0])*scale_x*flip_x_axis) # the waypoints are relative to the obstacle position! -> new_start_point and shift should be irrelevant!
            waypoint_y = str((waypoints[k][1][1])*scale_y*flip_y_axis) # scale and flip? are only relevant!
            obstacles_json += '\t\t\t\t\t\t[\n\t\t\t\t\t\t\t' + waypoint_x + ',\n\t\t\t\t\t\t\t' + waypoint_y + ',\n\t\t\t\t\t\t\t0' + '\n\t\t\t\t\t\t]'
            if count_temp < (len(obstacle_waypoint_connections[i]) - 1):
                obstacles_json += ','
            obstacles_json +='\n'
        else:
            # TODO NEXT: just a temporary solution for allowing a static obstacle
            # --> later this obstacle should be moved under static obstacles in the json file!
            # --> with relative goal coordinates 0 it crashes, so 0.01 was given
            # --> obstacle velocity (= 0.01) and motion (= 'yoyo') are also not needed, but some default values should be still used
            obstacles_json += '\t\t\t\t\t\t[\n\t\t\t\t\t\t\t' + '0.01' + ',\n\t\t\t\t\t\t\t' + '0.01' + ',\n\t\t\t\t\t\t\t0' + '\n\t\t\t\t\t\t]\n'
        count_temp += 1
        k += 1
    
    obstacles_json += '\t\t\t\t\t],\n\t\t\t\t\t"is_waypoint_relative": true,\n\t\t\t\t\t'
    obstacles_json += '"mode": "' + move + '"'
    obstacles_json += ',\n\t\t\t\t\t"triggers": [\n\t\t\t\t\t\t'

    # since now we have obstacle-watchers connections -> we do not have to take the watchers just line by line, but the coresponding lines! -> use the info from obstacle_watcher_connections or obstacle_watcher_connections_2
    #obstacles_json += '"watcher_' + watcher_num + '"'
    j = 0
    for watcher in obstacle_watcher_connections_2[i]:
        if watcher != '/':
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
    watcher_radius = str(watcher[2]*scale_x) # again here it does not matter if we use scale_x or scale_y, since both are the same!

    watchers_json += '\t\t\t\t"watcher_' + watcher_num + '": {\n\t\t\t\t\t"pos": ['
    watchers_json += '\n\t\t\t\t\t\t' + watcher_pos_x + ',\n\t\t\t\t\t\t' + watcher_pos_y + '\n\t\t\t\t\t],'
    watchers_json += '\n\t\t\t\t\t"range": ' + watcher_radius + '\n\t\t\t\t}'
    if i != len(watchers) - 1:
        watchers_json += ',\n'
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
final_string += '{\n\t"scenarios": [\n\t\t{\n\t\t\t"scene_name": "new_scenario",\n\t\t\t"repeats": 5,\n\t\t\t"dynamic_obstacles": {'
final_string += obstacles_json
final_string += '\n\t\t\t},\n'
final_string += '\t\t\t"static_obstacles": {\n\n\t\t\t},\n\t\t\t"robot": {'
final_string += robot_json
final_string += '\n\t\t\t},\n\t\t\t"watchers": {\n'
final_string += watchers_json
final_string += '\n\t\t\t}\n\t\t}\n\t]\n}'

fob.write(final_string)
fob.close()

print('/**************** new_scenario.json *****************/')
# print the final version of the json file
with open('output/new_scenario.json') as file:
    file_contents = file.read()
    print(file_contents)

# copy and paste the file to arena-rosnav/simulator-setup/scenarios, where all scenarious are stored
copyfile('output/new_scenario.json', '../simulator_setup/scenarios/new_scenario.json')
# to do it manuelly from the console, from arena-rosnav/gui run:
# $ sudo cp output/new_scenario.json ../simulator_setup/scenarios/

# when ready -> validate the json: https://jsonformatter.curiousconcept.com/

print('/**************** user_data.txt & scenario.png *****************/')
### save only the relevant to the user data in a new txt file (user_data.txt), from where the user could easily check his/hers inputs
# -> map resolution, obstacle velocities, obstacle-watchers connections from data.txt, the motions from motion.txt, obstacle types from obstacle.txt and if a watcher is triggered or not from triggered_watchers.txt in a nicer form
fob = open('output/user_data.txt','w')
fob.write('Map resolution:\n' + str(map_res) + "\n")
fob.write('Map origin:\n' + str(map_origin) + "\n")
if len(amount_pedestrians) == 0 and len(obstacle_force_factor) == 0:
    fob.write('ObstacleID - ObstacleType - Velocity - WatchersIDs(triggered-/+/?) - WaypointsIDs - Motion:\n')
if len(amount_pedestrians) == 0 and len(obstacle_force_factor) > 0:
    fob.write('ObstacleID - ObstacleType - Velocity - WatchersIDs(triggered-/+/?) - WaypointsIDs - Motion - Obstacle force factor - Desire force factor:\n')
if len(amount_pedestrians) > 0 and len(obstacle_force_factor) > 0:
    fob.write('ObstacleID - ObstacleType - Velocity - WatchersIDs(triggered-/+/?) - WaypointsIDs - Motion - Pedestrians amount - Chatting probability - Obstacle force factor - Desire force factor:\n')
i = 0
for obstacle in obstacles:
    fob.write(str(i) + ' - ' + str(obstacle[3]) + ' - ' + str(obstacle_vel[i]) + ' - ')
    j = 0
    for watcher in obstacle_watcher_connections_2[i]:
        fob.write(str(watcher))
        triggered_bool = False
        for triggered in triggered_watchers:
            if watcher == triggered:
                triggered_bool = True
        # '+' if the watcher has been triggered, otherwise '-'
        # '?' if the array is empty, so the simulation did not run and it can not be known if a watcher was triggered or not
        if len(triggered_watchers) > 0:
            if triggered_bool:
                fob.write('(' + '+' + ')')
            else:
                fob.write('(' + '-' + ')')
        else:
            fob.write('(' + '?' + ')')
        if j < (len(obstacle_watcher_connections_2[i]) - 1):
            fob.write(',')
        j += 1
    fob.write(' - ')
    k = 0
    for waypoint in obstacle_waypoint_connections[i]:
        fob.write(str(waypoint))
        if k < (len(obstacle_waypoint_connections[i]) - 1):
            fob.write(',')
        k += 1
    fob.write(' - ' + motion[i])
    # take care of the additional parameters
    if len(amount_pedestrians) == 0 and len(obstacle_force_factor) == 0:
        fob.write('\n')
    if len(amount_pedestrians) == 0 and len(obstacle_force_factor) > 0:
        fob.write(' - ' + str(obstacle_force_factor[i]))
        fob.write(' - ' + str(desire_force_factor[i]) + '\n')
    if len(amount_pedestrians) > 0 and len(obstacle_force_factor) > 0:
        fob.write(' - ' + str(amount_pedestrians[i]))
        fob.write(' - ' + str(chatting_probability[i]))
        fob.write(' - ' + str(obstacle_force_factor[i]))
        fob.write(' - ' + str(desire_force_factor[i]) + '\n')
    i += 1
fob.close()
# print the content
with open('output/user_data.txt') as file:
    file_contents = file.read()
    print(file_contents)

### save further more the resulted scenario as an image (so only the map with everything on it), to be able later to compare the resulted map with the ones in rviz
# 5_ready.png image should be cut/cropped according to the relative map position and size
im = Image.open(r"output/internal/5_ready.png")
width, height = im.size # size of orginal image: (800,600)=the size of the kivy window
# cropped image of the following dimension
left = image_corners[0][0]
top = height - image_corners[2][1]
right = image_corners[2][0]
bottom = height - image_corners[0][1]
im_scenario = im.crop((left, top, right, bottom))
im_scenario.save("output/scenario.png")
im_scenario.show() # show the image

### save also the cutted image from 6_sim.png with the dark red marked triggered watchers
im_sim = Image.open(r"output/internal/6_sim.png")
im_scenario_sim = im_sim.crop((left, top, right, bottom))
im_scenario_sim.save("output/scenario_sim.png")
im_scenario_sim.show() # show the image

# INFO:
# The code is tested with different maps (map_small.png and map.png) in rviz and it works.
# The scenario can be launched with following simple command:
# $ roslaunch arena_bringup start_arena_flatland.launch local_planner:="teb" use_viz:="true" map_file:="map1" rviz_file:="nav2"
# In nav2.rviz is the map horizontal and the first couple of obstacles are already with the start visible. It still works with all rviz files, just different visualized!
