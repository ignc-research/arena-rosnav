import io, os, shutil
from kivy.config import Config

# Idea 1: make the window not resizable
# for this to work, it should be placed before including the other kivy modules! (https://www.geeksforgeeks.org/python-window-size-adjustment-in-kivy/)
MAX_SIZE = (800, 600)
MIN_SIZE = (800, 600)
Config.set('graphics', 'resizable', False) # no resize button on the window, the window can not be resized even with dragging the corners
Config.set('graphics', 'width', MAX_SIZE[0]) # the window can not be resized bigger then this values
Config.set('graphics', 'height', MAX_SIZE[1])

#import kivy
from kivy.app import App
from kivy.uix.widget import Widget
from kivy.uix.dropdown import DropDown
from kivy.uix.button import Button
from kivy.graphics import Color, Ellipse, Line
from kivy.uix.slider import Slider
from kivy.lang import Builder
from kivy.core.image import Image as CoreImage
from kivy.uix.image import Image
from kivy.uix.gridlayout import GridLayout
from kivy.core.window import Window
from kivy.uix.label import Label
from kivy.uix.textinput import TextInput
from kivy.uix.scrollview import ScrollView

# global variables
color_r = 1
color_g = 0
color_b = 0
counter = 0
obstacle_type = [('cleaner', (0, 0, 1)), ('random', (1, 0.5, 0)), ('turtlebot', (1, 0, 0.5)), ('walker', (0.5, 0, 0.5))]

class MyPaintWidgetCircleObstacle(Widget): # obstacle

    def file_len(self, fname):
        with open(fname) as f:
            for i, l in enumerate(f):
                pass
        if 'i' in locals(): return i + 1
        else: return 0

    # IDEA: on_touch_down - start the circle, on_touch_move - keep expanding the circle, on_touch_up - finish the circle
    def on_touch_down(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
           # the radius of the ellipse is changable by the user
            with self.canvas:
                #Color(1, 0, 0)
                Color(color_r, color_g, color_b)
                d = 20.
                #touch.ud['line'] = Line(points=(touch.x, touch.y)) # for debugging
                touch.ud['ellipse'] = Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d)) # should be shiftet, because the position specifies the bottom left corner of the ellipse’s bounding box
            #print('Touch event for placing the obstacle with center at: (' + str(touch.x - d / 2) + ', ' + str(touch.y - d / 2) + ')')
            
            # TODO IDEA: check somehow where the click is made, if on top of buttons for example do not do anything etc. (if touch.grab_current is self)
            # TODO IDEA: make here a text area for the obstacle velocity, which was just drawn, the text area will appear every time on the same place -> problem text area not aditable
            # TODO IDEA: try to write the velocity itself in the circles, but maybe difficult later to match the circles with the velocity

    def on_touch_move(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            #touch.ud['line'].points += [touch.x, touch.y] # for debigging
            # ! in this way the circle could be moved to a new position, as long as the mouse remains clicked!
            #d = 20.
            #touch.ud['ellipse'].pos = (touch.x - d / 2, touch.y - d / 2)
            # in this way the radius is canged to the new value if the circle is dragged (the user clicks and moves the mouse)
            #touch.ud['ellipse'].size = (50,50)
            # IDEA: ellipse takes the bottom left corner as input -> try to scale it, while the user is moving the mouse;
            # Idea 1 (implemented only for 1.Q and commented after that, for the others is still a TODO ): consider the 4 corners (move in +x&+y, +x&-y, -x&+y, -x&-y direction) -> move slowly: scale the distace between on_touch_down and current on_touch_move
            # Idea 2 (implemented): no matter of the direction, to which the mouse is dragged, the circle will become bigger without changing the position of the center (for this solution is no if-else needed)
            d_before = touch.ud['ellipse'].size[0]
            x_pos_before = touch.ud['ellipse'].pos[0]
            y_pos_before = touch.ud['ellipse'].pos[1]
            touch_x_before = x_pos_before + d_before / 2
            touch_y_before = y_pos_before + d_before / 2
            # start (0,0) is the bottom left window corner
            # TODO IDEA: small amount of 1.Q-4.Q (a little bit at the beginning and a little bit at the end) use to move the block in x and y
            if (touch.x-touch_x_before) > 0:
                # moved left
                #if (touch.y-touch_y_before) > -10 and (touch.y-touch_y_before) < 10: # TODO: as the first of three posibilities -> difficult to train!
                #    # move exactly to the right # NEW
                #    print("between 1.Q & 4.Q -> move right")
                #    # the circle will be moved to the new position, as long as the mouse remains clicked!
                #    touch.ud['ellipse'].pos = (touch.x - d_before / 2, touch.y - d_before / 2)
                if (touch.y-touch_y_before) > 0:
                    # moved up
                    print("1.Q")
                    d_now = touch.x-touch_x_before # it doesn't matter if we take x or y, since we work with circles
                    print("new radius: " + str(d_now))
                    touch.ud['ellipse'].size = (d_now,d_now)
                    # only in this situation is not necessary to change the value of touch.ud['ellipse'].pos since the right corner is per default taken
                    # ! this way the circle will become bigger without the center being changed
                    x_pos_now = touch_x_before - d_now / 2
                    y_pos_now = touch_y_before - d_now / 2
                    touch.ud['ellipse'].pos = (x_pos_now, y_pos_now)
                else:
                    # moved down
                    print("4.Q")
                    d_now = touch.x-touch_x_before
                    print("new radius: " + str(d_now))
                    touch.ud['ellipse'].size = (d_now,d_now)
                    # the value of touch.ud['ellipse'].pos should be change -> the bottom corner of the new circle should be calculated; move from top left to bottom left -> so change only y
                    # ! this way the circle will become bigger without the center being changed
                    x_pos_now = touch_x_before - d_now / 2
                    y_pos_now = touch_y_before - d_now / 2
                    touch.ud['ellipse'].pos = (x_pos_now, y_pos_now)
            else:
                # moved right
                if (touch.y-touch_y_before) > 0:
                    # moved up
                    print("2.Q")
                    d_now = touch_x_before-touch.x
                    print("new radius: " + str(d_now))
                    touch.ud['ellipse'].size = (d_now,d_now)
                    # ! this way the circle will become bigger without the center being changed
                    x_pos_now = touch_x_before - d_now / 2
                    y_pos_now = touch_y_before - d_now / 2
                    touch.ud['ellipse'].pos = (x_pos_now, y_pos_now)
                else:
                    # moved down
                    print("3.Q")
                    d_now = touch_x_before-touch.x
                    print("new radius: " + str(d_now))
                    touch.ud['ellipse'].size = (d_now,d_now)
                    # ! this way the circle will become bigger without the center being changed
                    x_pos_now = touch_x_before - d_now / 2
                    y_pos_now = touch_y_before - d_now / 2
                    touch.ud['ellipse'].pos = (x_pos_now, y_pos_now)

    def on_touch_up(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        # check if the positions are inside the 4 corners of the image
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
        #if (touch.x > 144.33781190019192 and touch.x < 655.6621880998081 and touch.y > 200 and touch.y < 600): # ((144.33781190019192, 200), (144.33781190019192, 600), (655.6621880998081, 600), (655.6621880998081, 200))
            # save the obstacle positions (x,y,radius) in a txt file
            fob = open('output/obstacle.txt','a') # 'w'=write (overrides the content every time), better use 'a'=append but make sure the file is at first empty
            d_last = touch.ud['ellipse'].size[0]
            x_pos_last = touch.ud['ellipse'].pos[0]
            y_pos_last = touch.ud['ellipse'].pos[1]
            fob.write('obstacle (x,y,radius,type): ' + str(x_pos_last+d_last/2) + ',' + str(y_pos_last+d_last/2) + ',' + str(d_last/2)) # center and radius # should be called here in on_touch_up so that the data is right!
            # the center is in (x_pos_last+d_last/2, y_pos_last+d_last/2) ! -> then the radius is also not d_last, but d_last/2 !
            # save also the color of the obstacle or even directly the type of the obstacle
            global color_r
            global color_g
            global color_b
            #fob.write(', ' + str(color_r) + ', ' + str(color_g) + ', ' + str(color_b)) # write the color
            type_unknown = 1
            for obst_type in obstacle_type:
                if (obst_type[1][0] == color_r) and (obst_type[1][1] == color_g) and (obst_type[1][2] == color_b):
                    fob.write(',' + obst_type[0] + "\n") # write directly the obstacle type
                    type_unknown = 0
            if type_unknown == 1:
                fob.write(',default' + "\n")

            # put numbers in the green circles and then separately ask the user for the velocity of every obstacle in a text field
            count = self.file_len('output/obstacle.txt') # a counter for all on_touch_down events for this class MyPaintWidgetCircleObstacle -> idea: count the lines from obstacle.txt
            #label_num = Label(text=str(count), pos=(x_pos_last, y_pos_last), size=(d_last, d_last), color=(0,1,0), disabled_color=(0,1,0)) # place the number in the middle of the circle # right
            label_num = Label(text=str(count), pos=(x_pos_last+d_last/2, y_pos_last+d_last/2), size=(1, 1), color=(0,1,0), disabled_color=(0,1,0)) # place the number in the middle of the circle # right
            self.add_widget(label_num)
        
            fob.close()

class MyPaintWidgetCircleWatcher(Widget): # watcher (bigger then the obstacle)

    def file_len(self, fname):
        with open(fname) as f:
            for i, l in enumerate(f):
                pass
        if 'i' in locals(): return i + 1
        else: return 0

    def on_touch_down(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            # the radius of the ellipse is changable by the user (by moving the mouse while clicked)
            # TODO: would be better if it was a circle without a filling -> draw a circumference (not possible in kivy??)
            with self.canvas:
                Color(1, 1, 0)
                d = 50.
                # should be shiftet, because the position specifies the bottom left corner of the ellipse’s bounding box
                #Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))
                touch.ud['ellipse2'] = Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d))

    def on_touch_move(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            # Idea 2: no matter of the direction, to which the mouse is dragged, the circle will become bigger without changing the position of the center (for this solution is no if-else needed)
            d_before = touch.ud['ellipse2'].size[0]
            x_pos_before = touch.ud['ellipse2'].pos[0]
            y_pos_before = touch.ud['ellipse2'].pos[1]
            touch_x_before = x_pos_before + d_before / 2
            touch_y_before = y_pos_before + d_before / 2
            # start (0,0) is the bottom left window corner
            if (touch.x-touch_x_before) > 0: # it doesn't matter if we take x or y, since we work with circles
                # moved left
                d_now = touch.x-touch_x_before
                print("new radius: " + str(d_now))
                touch.ud['ellipse2'].size = (d_now,d_now)
                x_pos_now = touch_x_before - d_now / 2
                y_pos_now = touch_y_before - d_now / 2
                touch.ud['ellipse2'].pos = (x_pos_now, y_pos_now)
            else:
                # moved right
                d_now = touch_x_before-touch.x
                print("new radius: " + str(d_now))
                touch.ud['ellipse2'].size = (d_now,d_now)
                x_pos_now = touch_x_before - d_now / 2
                y_pos_now = touch_y_before - d_now / 2
                touch.ud['ellipse2'].pos = (x_pos_now, y_pos_now)

    def on_touch_up(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        # check if the positions are inside the 4 corners of the image
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            # save the watcher positions (x,y,radius) in a txt file
            fob = open('output/watcher.txt','a') # 'w'=write (overrides the content every time), better use 'a'=append but make sure the file is at first empty
            d_last = touch.ud['ellipse2'].size[0]
            x_pos_last = touch.ud['ellipse2'].pos[0]
            y_pos_last = touch.ud['ellipse2'].pos[1]
            fob.write('watcher (x,y,radius): ' + str(x_pos_last+d_last/2) + ',' + str(y_pos_last+d_last/2) + ',' + str(d_last/2) + "\n") # center and radius

            # IDEA: give also the watchers an ID and then separately ask the user in a text field to connect the obstacles to the watchers
            # TODO NEXT: might be a problem if the center of the watcher is placed under the obstacle, it will not be visible
            count = self.file_len('output/watcher.txt') # count the lines from watcher.txt
            label_num = Label(text=str(count), pos=(x_pos_last, y_pos_last), size=(d_last, d_last), color=(0,0,1), disabled_color=(0,0,1)) # place the number in the middle of the circle
            self.add_widget(label_num)

            fob.close()

class MyPaintWidgetLine(Widget):

    def on_touch_down(self, touch): # lines = vectors/trajectories with start and end position
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            with self.canvas:
                Color(1, 0, 0)
                touch.ud['line'] = Line(points=(touch.x, touch.y)) # TODO: make the lines smooth!
            # save the vector positions start(x,y), end(x,y) in a txt file
            fob = open('output/vector.txt','a') # 'w'=write (overrides the content every time), better use 'a'=append but make sure the file is at first empty
            fob.write('vector start (x,y): ' + str(touch.x) + ',' + str(touch.y) + "\n")
            fob.close()
    
    def on_touch_move(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            touch.ud['line'].points += [touch.x, touch.y]
            #fob = open('output/vector.txt','a') # TEST
            #fob.write('vector test (x,y): ' + str(touch.x) + ',' + str(touch.y) + "\n") # TEST
    
    def on_touch_up(self, touch):
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            # save the vector positions start(x,y), end(x,y) in a txt file
            fob = open('output/vector.txt','a') # 'w'=write (overrides the content every time), better use 'a'=append but make sure the file is at first empty
            fob.write('vector end (x,y): ' + str(touch.x) + ',' + str(touch.y) + "\n")
            fob.close()

class MyPaintWidgetRobot(Widget): # for drawing two circles with a constant small radius for the start and end position of the robot (so just take x and y for both positions)
    def file_len(self, fname):
        with open(fname) as f:
            for i, l in enumerate(f):
                pass
        if 'i' in locals(): return i + 1
        else: return 0
    
    # the first circle that is done should be the start and the second should be the end
    def on_touch_down(self, touch): # lines = vectors/trajectories with start and end position
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom # internal.txt
        lines = []
        with open('output/internal.txt') as file:
            lines = file.readlines()
        
        if (touch.y < float(lines[0]) and touch.y > float(lines[1]) and touch.x < float(lines[2]) and touch.x > float(lines[3])):
            # try to contol the user to click only twice! -> nothing will happen after the second click!
            fob = open('output/robot.txt','a') # 'w'=write (overrides the content every time), better use 'a'=append but make sure the file is at first empty
            if self.file_len('output/robot.txt') < 2:
                with self.canvas:
                    Color(0, 1, 1)
                    d = 20.
                    Ellipse(pos=(touch.x - d / 2, touch.y - d / 2), size=(d, d)) # should be shiftet, because the position specifies the bottom left corner of the ellipse’s bounding box
            
            # save x and y positions of both start and end drawn circles in another txt file
            if self.file_len('output/robot.txt') == 0:
                fob.write('robot start position (x,y): ' + str(touch.x) + ',' + str(touch.y) + "\n")
            if self.file_len('output/robot.txt') == 1:
                fob.write('robot end position (x,y): ' + str(touch.x) + ',' + str(touch.y) + "\n")
            fob.close()

class MyPaintApp(App):

    def check_resize(self, instance, x, y):

        # Idea 1: make the window not resizable
        # always keep the size to the default size (800, 600) - even if the user tries to resize it by dragging the corners (https://stackoverflow.com/questions/44641260/kivy-how-do-i-set-my-app-to-be-resizable-up-to-a-specified-size)
        #if (x > MAX_SIZE[0]) or (x < MIN_SIZE[0]):  # resize X
        #    Window.size = (800, Window.size[1])
        #if (y > MAX_SIZE[1]) or (y < MIN_SIZE[1]): # resize Y
        #    Window.size = (Window.size[0], 600)
        print('WIndow size: ' + str(Window.size)) # here are the changes of the window considered! # for me is (800, 600) -> (1853, 1025)

        # TODO NEXT: call self.parent_widget() here -> still work to be done! -> the changes are made on top, so multiple widgets of one kind
        self.parent_widget()

    # https://stackoverflow.com/questions/185936/how-to-delete-the-contents-of-a-folder
    def delete_files_in_folder(self):
        folder = 'output'
        for filename in os.listdir(folder):
            file_path = os.path.join(folder, filename)
            try:
                if os.path.isfile(file_path) or os.path.islink(file_path):
                    os.unlink(file_path)
                elif os.path.isdir(file_path):
                    shutil.rmtree(file_path)
            except Exception as e:
                print('Failed to delete %s. Reason: %s' % (file_path, e))

    def build(self):

        # at the beggining of the programm clear the data in the output directory to start fresh, since not all of the files are open to be rewriten, but also during the program information to be appended
        self.delete_files_in_folder()

        self.parent = Widget()

        # Idea 1: make the window not resizable
        Window.bind(on_resize=self.check_resize)
        # Idea 2: make the window properly resizable -> TODO NEXT: all heights and widths should be then dependabple on the window height and width!?
        # - to how to get the updated windows size as soon as it changes, call self.parent_widget() in check_resize()!
        # - use pos_hint and size_hint on every widget (from parent to child) !? (https://stackoverflow.com/questions/48351129/kivy-buttons-wont-stay-put-during-window-resizing)

        #self.parent_widget()

        return self.parent

    def parent_widget(self):
        
        # TODO NEXT: the window should be clearly separated into areas (area for the map, for the buttons etc.)

        # reset the parent (otherwise after resizing the resized widgets will just show above the old ones) # TODO NEXT
        for child in self.parent.children:
            for child2 in child.children:
                child.remove_widget(child2)
            self.parent.remove_widget(child)

        parent_draw = Widget() # its children are only the widgets with drawing behavior! # necessary for the return button behavior!

        window_sizes=Window.size # (800, 600) at the beginning per default
        print('new window size:' + str(window_sizes))
        print('parent size: ' + str(self.parent.size)) # called in build() it givies always the default value (100, 100), called in check_resize() it changes after every resize!

        # create drawing widgets
        self.painter_circle_watcher = MyPaintWidgetCircleWatcher()
        self.painter_circle_obstacle = MyPaintWidgetCircleObstacle()
        self.painter_line = MyPaintWidgetLine()
        self.painter_robot = MyPaintWidgetRobot()
        
        # create a layout for the buttons
        height_layout_btn = window_sizes[1]/3 # 200 # TODO: make it smaller and scale accordingly the other widgets
        width_layout_btn = 0
        border = 5
        layout_btn = GridLayout(cols=4, size=(window_sizes[0],height_layout_btn), size_hint=(None, None))
        #layout_btn = GridLayout(cols=4, size_hint=(1, .9)) # TEST # TODO NEXT -> it appears always in the bottom left corner with the fixed size of the parent (100,100)

        # load the user map as a background image for the drawings
        # Important: it should work with every map (try also with 'input/map.png')
        width_layout_map = 140 + 2*border
        size_given_width = window_sizes[0] - 2*width_layout_map
        size_given_height = window_sizes[1] - height_layout_btn
        # pos gives the start, so the bottom left corner of the image, and from there the size will be kept or resized!
        wimg_input_map = Image(source='input/map_small.png', size=(size_given_width, size_given_height), pos=(width_layout_map, height_layout_btn)) # push the image up with the heght of the buttons # https://www.geeksforgeeks.org/change-the-size-and-position-of-button-in-kivy/
        # find the real size of the uploaded image
        image_size_before = (wimg_input_map.texture_size[0], wimg_input_map.texture_size[1])
        print('size image before:' + str(image_size_before)) # (666, 521) for map_small

        # find the real size of the showed image; find out how the width and height will be resized; find a relation between the old and new size
        width_image_showed = size_given_width # in a perfect world # init
        height_image_showed = size_given_height # in a perfect world # init
        if (image_size_before[0] > size_given_width) and (image_size_before[1] > size_given_height): # both width and height are bigger then the area width and height
            if (image_size_before[0] - size_given_width) >= (image_size_before[1] - size_given_height): # width difference >= height diference
                # the width will resize to the area width, the resized height should be calculated
                width_image_showed = size_given_width
                relation_width = image_size_before[0]/width_image_showed
                height_image_showed = image_size_before[1]/relation_width
            elif (image_size_before[0] - size_given_width) < (image_size_before[1] - size_given_height): # width difference < height diference
                # the height wil resize to the area height, the resized width should be calculated (that is what happens for map_small)
                height_image_showed = size_given_height
                relation_height = image_size_before[1]/height_image_showed
                width_image_showed = image_size_before[0]/relation_height
        elif (image_size_before[0] > size_given_width) and (image_size_before[1] <= size_given_height): # only the width is bigger then the area width
            # the width will resize to the area width, the resized height should be calculated
            width_image_showed = size_given_width
            relation_width = image_size_before[0]/width_image_showed
            height_image_showed = image_size_before[1]/relation_width
        elif (image_size_before[0] <= size_given_width) and (image_size_before[1] > size_given_height): # only the height is bigger then the area height
            # the height will resize to the area height, the resized width should be calculated
            height_image_showed = size_given_height
            relation_height = image_size_before[1]/height_image_showed
            width_image_showed = image_size_before[0]/relation_height
        else: # both width and height are smaller then the area width and height
            # the image will not resize and stay the same
            width_image_showed = image_size_before[0]
            height_image_showed = image_size_before[1]
        image_size_after = (width_image_showed, height_image_showed)
        print('size image after:' + str(image_size_after)) # (511.32437619961615, 400) for map_small

        # Important: after the image has been resized, it will show up in the center of the resized width or height (not aligned to the bottom left corner)!
        # TODO NEXT: test the borders left, right, top and bottom!
        border_margin_left_right = (window_sizes[0] - image_size_after[0])/2 # margin for the left and right side # that is enough since the area for the image is already centered regarding the width
        border_margin_top = (window_sizes[1] - height_layout_btn - image_size_after[1])/2 # margin for the top side
        border_margin_bottom = border_margin_top + height_layout_btn # margin for the bottom side
        print('border margin left-right:' + str(border_margin_left_right) + ', top:' + str(border_margin_top) + ', bottom:' + str(border_margin_bottom)) # left-right: 144.33781190019192;
        # find the 4 corners, where the image is visualized # TODO NEXT: only inside of them should the obstacles etc. be considered
        image_corners = ((border_margin_left_right, border_margin_bottom),(border_margin_left_right, border_margin_bottom+image_size_after[1]),(border_margin_left_right+image_size_after[0], border_margin_bottom+image_size_after[1]),(border_margin_left_right+image_size_after[0], border_margin_bottom)) # left bottom, left top, right top, right bottom
        print('4 corners shown image:' + str(image_corners)) # ((144.33781190019192, 200), (144.33781190019192, 600), (655.6621880998081, 600), (655.6621880998081, 200))
        
        # scale the image: from the uploaded image -> to the drawing area -> to the whole window -> in parser.py (using "resolution" and "origin" from the yaml file) to the real world area
        scale = (image_size_before[0]/image_size_after[0], image_size_before[1]/image_size_after[1])
        print('scale = image_size_before/image_size_after:' + str(scale)) # image_size_after * scale = image_size_before # (1.3025, 1.3025)
        # the image range should be scaled up to the window-size range
        print('size image window:' + str(window_sizes)) # [800, 600]
        scale2 = (image_size_after[0]/window_sizes[0], image_size_after[1]/window_sizes[1])
        print('scale 2 = image_size_after/window_size:' + str(scale2)) # window_size * scale2 = image_size_after # (0.6391554702495202, 0.6666666666666666)
        scale_total = (image_size_before[0]/window_sizes[0], image_size_before[1]/window_sizes[1])
        print('scale total = image_size_before/window_size:' + str(scale_total)) # window_size * scale_total = image_size_before # (0.8325, 0.8683333333333333)
        # Important: all positions(x,y) and radiuses should be multiplied with the scale(x,y) value to be right (done in parser.py)
        # in parser.py will for now only scale be used, but nevertheless all there parameters (scale, scale2 and scale_total are passed), since it is better that parser.py has more information then less informtation
        # the proportion to the window size is not needed that much as just the (0,0) point of the showd image -> so the shift of the start -> so the corners of the showed image (more specifically the bottom left corner, so the first corner); the corners are already passed to the parser.py file
        
        # save the the 4 corners, where the image is visualized, but in another form in another txt file that is just for internal use and is not necessary to be parserd later with parser.py
        # form: height_top\nheight_bottom\nwidth_top\nwidth_bottom
        # used as a internal restriction to allow the user to draw only on the uploaded image
        fob = open('output/internal.txt','w') # 'w'=write (overrides the content every time)
        fob.write(str(image_corners[1][1]) + '\n' + str(image_corners[0][1]) + '\n' + str(image_corners[2][0]) + '\n' + str(image_corners[0][0]))
        fob.close()

        # specify some borders
        height_up_border=window_sizes[1]/10 #60
        width_left_border=5
        width_right_border=5

        # add a text area, where the user should specify from the begining how many obstacles he will place on the map
        # this should be one of the first things the user does, since this information is needed for later on!
        height_layout_num_obstacles = 60 # TODO NEXT
        width_layout_num_obstacles = 140
        layout_num_obstacles = GridLayout(cols=1, rows=2, size=(width_layout_num_obstacles,height_layout_num_obstacles), size_hint=(None, None), pos=(width_left_border, window_sizes[1]-height_up_border-height_layout_num_obstacles/2))
        label_num_obstacles = Label(text='#obstacles:')
        textinput_num_obstacles = TextInput(text='3') # TODO: it causes an error?? (every text area!?)
        layout_num_obstacles.add_widget(label_num_obstacles)
        layout_num_obstacles.add_widget(textinput_num_obstacles)

        # another user input should be the resolution of the map, as an example see "resolution" in arena-rosnav/simulator_setup/maps/map1/map.yaml!
        height_layout_res = 60
        width_layout_res = 140
        layout_res = GridLayout(cols=1, rows=2, size=(width_layout_res,height_layout_res), size_hint=(None, None), pos=(width_left_border, window_sizes[1]-height_up_border-height_layout_num_obstacles-height_layout_res/2))
        label_res = Label(text='Map resolution:')
        textinput_res = TextInput(text='0.05') # editable; give an example value already written in the box
        layout_res.add_widget(label_res)
        layout_res.add_widget(textinput_res)

        # another user input should be the origin of the map, as an example see "origin" in arena-rosnav/simulator_setup/maps/map1/map.yaml!
        # form: [x_pos,y_pos,z_pos]
        height_layout_origin = 60
        width_layout_origin = 140
        layout_origin = GridLayout(cols=1, rows=2, size=(width_layout_origin,height_layout_origin), size_hint=(None, None), pos=(width_left_border, window_sizes[1]-height_up_border-height_layout_num_obstacles-height_layout_res-height_layout_origin/2))
        label_origin = Label(text='Map origin:') # the form should be "x_pos, y_pos, z_pos"
        textinput_origin = TextInput(text='-6.0, -6.0, 0.0') # editable; give an example value already written in the box
        layout_origin.add_widget(label_origin)
        layout_origin.add_widget(textinput_origin)

        # a dropdown button with all obstacle types (cleaner, random, turtlebot, walker)
        # the different obstacle types should be not only visualized with different colors, but their value should be also saved in a file
        height_btn_obstacle_type = 30
        width_btn_obstacle_type = 140
        dropdown_obstacle_type = DropDown() # drop-down menu with different motions
        btn_cleaner = Button(text='cleaner (blue)', size_hint_y=None, height=height_btn_obstacle_type)
        btn_cleaner.bind(on_release=lambda btn: dropdown_obstacle_type.select(btn_cleaner.text), on_press=lambda x: self.button_obstacle_type_cleaner_callback_down(mainbutton_obstacle_type, self.parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res))
        dropdown_obstacle_type.add_widget(btn_cleaner)
        btn_random = Button(text='random (orange)', size_hint_y=None, height=height_btn_obstacle_type)
        btn_random.bind(on_release=lambda btn: dropdown_obstacle_type.select(btn_random.text), on_press=lambda x: self.button_obstacle_type_random_callback_down(mainbutton_obstacle_type, self.parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res))
        dropdown_obstacle_type.add_widget(btn_random)
        btn_turtlebot = Button(text='turtlebot (pink)', size_hint_y=None, height=height_btn_obstacle_type)
        btn_turtlebot.bind(on_release=lambda btn: dropdown_obstacle_type.select(btn_turtlebot.text), on_press=lambda x: self.button_obstacle_type_turtlebot_callback_down(mainbutton_obstacle_type, self.parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res))
        dropdown_obstacle_type.add_widget(btn_turtlebot)
        btn_walker = Button(text='walker (purple)', size_hint_y=None, height=height_btn_obstacle_type)
        btn_walker.bind(on_release=lambda btn: dropdown_obstacle_type.select(btn_walker.text), on_press=lambda x: self.button_obstacle_type_walker_callback_down(mainbutton_obstacle_type, self.parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res))
        dropdown_obstacle_type.add_widget(btn_walker)
        mainbutton_obstacle_type = Button(text='obstacle type', size_hint=(None, None), size=(width_btn_obstacle_type,height_btn_obstacle_type), pos=(width_left_border, window_sizes[1]-height_up_border-height_layout_num_obstacles-height_layout_res-height_layout_origin-height_btn_obstacle_type/2)) #size_hint_y=None
        mainbutton_obstacle_type.bind(on_release=dropdown_obstacle_type.open)
        dropdown_obstacle_type.bind(on_select=lambda instance, x: setattr(mainbutton_obstacle_type, 'text', x))

        # return button -> return one step (so if now drawing lines is turned on, after the button is clicked, start fresh with the lines, so delete all lines (and update the coresponding txt file!))
        height_btn_return = 30
        width_btn_return = 140
        button_return = Button(text='return', size_hint=(None, None), size=(width_btn_return,height_btn_return), pos=(border,height_layout_btn+height_btn_return))
        button_return.bind(on_press=lambda x: self.button_return_callback_down(self.parent, parent_draw, button, button2, button3, button4, self.painter_circle_obstacle, self.painter_circle_watcher, self.painter_line, self.painter_robot))

        # TODO NEXT: Rules for the readme file
        # 1) Once something is drawn it can not be moved or deleted; 2) To change the radius move still the clicked mouse to the right/left; ...

        # dynamically filled lists (needed for the txt files)
        textinput_velocity_list = []
        textinput_obstacle_watchers_connection_list = []
        mainbutton_motion_list = []
        
        # create buttons
        button = Button(text='Click when ready\nwith the obstacles\nnumber & position', font_size=14)
        button2 = Button(text='Click when ready\nwith the watchers &\nsetting the parameters\non the right & left', font_size=14)
        # the position of the robot should be also given by the user -> make another button for the beginning and allow the user to click on start and end position
        button3 = Button(text='Click when ready\nwith the waypoints', font_size=14)
        button4 = Button(text='Click when ready\nwith robot start\nand end position\n-> Done! :)', font_size=14)

        button.bind(on_press=lambda x: self.button_callback(self.parent, parent_draw, button, button2, mainbutton_obstacle_type, wimg_input_map, layout_btn, layout_origin, layout_res, textinput_res, textinput_origin, textinput_num_obstacles, image_corners, scale, scale2, scale_total, width_left_border, height_up_border, height_layout_btn, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list))
        button2.bind(on_press=lambda x: self.button2_callback(self.parent, parent_draw, button2, button3, layout_btn, wimg_input_map, textinput_num_obstacles, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list))
        button3.bind(on_press=lambda x: self.button3_callback(self.parent, parent_draw, button3, button4, layout_btn, wimg_input_map))
        button4.bind(on_press=lambda x: self.button4_callback(self.parent, parent_draw, button4, layout_btn))
        
        #button.disabled = True # at first should be disabled
        button2.disabled = True # at first should be disabled
        button3.disabled = True # at first should be disabled
        button4.disabled = True # at first should be disabled

        layout_btn.add_widget(button)
        layout_btn.add_widget(button2)
        layout_btn.add_widget(button3)
        layout_btn.add_widget(button4)

        # add the neu widgets to the parent widgets
        #parent_draw.add_widget(self.painter_circle_obstacle) # Important: add this widget if you want to have only one 'default' red obstacle type! If you are using the dropdown button for different obstacle types this widget do not have to be added!
        self.parent.add_widget(wimg_input_map)
        self.parent.add_widget(parent_draw)
        self.parent.add_widget(layout_btn)
        self.parent.add_widget(layout_num_obstacles)
        self.parent.add_widget(layout_origin)
        self.parent.add_widget(layout_res)
        self.parent.add_widget(button_return)
        self.parent.add_widget(mainbutton_obstacle_type)

        # Important: check the txt files, if everything is correct or some unnecessary information if sometimes added!
        # Important: to be able to match obstacle with a vector, they should be clicked in the same order!
        # TODO NEXT: sometimes the error with "KeyError: 'ellipse'" occures !?? -> do not allow to draw on top of the buttons/labels etc. -> allow drawings only on the image area

    # IDEA: save as image -> load back -> then delete the first widget (that is how the drawings won't disappear) and enable the next widget
    # IDEA: make a button (click when ready with the obstacles => save the positions and radius), then draw the watchers and again click and last draw the lines
    def button_callback(self, parent, parent_draw, button, button2, mainbutton_obstacle_type, wimg_input_map, layout_btn, layout_origin, layout_res, textinput_res, textinput_origin, textinput_num_obstacles, image_corners, scale, scale2, scale_total, width_left_border, height_up_border, height_layout_btn, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list):
        print('The button <%s> is being pressed' % ' '.join(button.text.split('\n')))
        # the number of obstacles can not be changed once the button "done" has been clicked
        textinput_num_obstacles.disabled = True # disable the button, should be clicked only once!
        
        # the user should first say how many obstacles he put -> after that can be created the same amount of text boxes and drop down boxes as the amount of obstacles -> so putting obstacle velocity, watchers and motion per obstacle can be done individually!
        print('Number of obstacles: ' + textinput_num_obstacles.text)
        # on one row a text area for the obstacle velocity and for the obstacle-watcher connection and a button for the motion
        height_layout_connect = (Window.size[1]-2*height_up_border-height_layout_btn)*2
        width_layout_connect = 140
        label_connect = Label(text='Obstacle - Velocity -\nWatchers - Motion', size=(width_layout_connect,60), size_hint=(None, None), pos=(Window.size[0]-width_left_border-140, Window.size[1]-height_up_border-30))

        # local variables, not needed to be global
        label_index_list = []
        btn_yoyo_list = []
        btn_circle_list = []
        dropdown_motion_list = []

        # IDEA 1 - make everything with text inputs (no upper bound regarding the number of obstacles), scrollable after the fist 10 obstacles
        scrollable_area = self.set_obstacle_params(textinput_num_obstacles, label_index_list, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list, height_layout_connect, width_layout_connect, width_left_border, height_up_border)
        # IDEA 2 - make text inputs only for the velocity and watchers -> for the motions make dropdown boxes (upper bound of max 10-20 obstacles), scrollable after the fist 10 obstacles
        #scrollable_areas = self.set_obstacle_params_2(textinput_num_obstacles, label_index_list, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list, height_layout_connect, width_layout_connect, width_left_border, height_up_border)

        # save the data from the text areas also to a txt file
        # at this place in the code are they for sure not empty and are their final version
        fob = open('output/data.txt','w') # 'w'=write (overrides the content every time)
        fob.write('Image corners:\n' + str(image_corners[0][0]) + ',' + str(image_corners[0][1]) + '\n' + str(image_corners[1][0]) + ',' + str(image_corners[1][1]) + '\n' + str(image_corners[2][0]) + ',' + str(image_corners[2][1]) + '\n' + str(image_corners[3][0]) + ',' + str(image_corners[3][1]) + '\nPositions scale:\n' + str(scale[0]) + ',' + str(scale[1]) + ',' + str(scale2[0]) + ',' + str(scale2[1]) + ',' + str(scale_total[0]) + ',' + str(scale_total[1]) + '\nMap resolution:\n' + str(textinput_res.text) + '\nMap origin:\n' +  str(textinput_origin.text))

        parent.remove_widget(wimg_input_map) # !
        # the text boxes about the obstacle velocities and obstacle-watchers connections schould be editable also after the first button click!
        # the problem is that the whole window is downloaded and uploaded as an image, and not just the part with the map on!
        # it is necessary to make the widgets inactive and after png download and upload active again, otherwise they are just considered as nothing more then an image
        # the info from these text boxes should be written to the data.txt file at the beginning of the second button (because there will the final version be visible!)
        # TODO NEXT
        parent.remove_widget(layout_btn)
        parent.remove_widget(layout_origin)
        parent.remove_widget(layout_res)

        parent.export_to_png("output/obstacles.png")
        for child in parent_draw.children: # !
            parent_draw.remove_widget(child)
        parent.remove_widget(parent_draw)

        # TODO: add to the widget tree only if not already there (otherwise -> error)
        parent.add_widget(wimg_input_map) # !
        parent_draw.add_widget(self.painter_circle_watcher)
        parent.add_widget(parent_draw)
        parent.add_widget(layout_btn)
        parent.add_widget(layout_origin)
        parent.add_widget(layout_res)
        parent.add_widget(label_connect)
        parent.add_widget(scrollable_area) # for Idea 1
        #parent.add_widget(scrollable_areas[0]) # for Idea 2 with the dropdown boxes
        #parent.add_widget(scrollable_areas[1]) # for Idea 2 with the dropdown boxes

        wimg_obstacles = Image(source='output/obstacles.png', size=(parent.width, parent.height))
        parent.add_widget(wimg_obstacles)
        button.disabled = True # disable the button, should be clicked only once!
        button2.disabled = False # enable the next button
        mainbutton_obstacle_type.disabled = True # the dropdown button should not be change anymore

    def button2_callback(self, parent, parent_draw, button2, button3, layout_btn, wimg_input_map, textinput_num_obstacles, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list):
        # should be done here and not in button_callback, because there the button values are still not visible!
        fob = open('output/data.txt','a') # 'a'=append
        fob.write('\nObstacle velocities:\n')
        for i in range(int(textinput_num_obstacles.text)):
            #fob.write(str(globals()['textinput_velocity_%s' % i].text))
            fob.write(str(textinput_velocity_list[i].text))
            if i < int(textinput_num_obstacles.text) - 1:
                fob.write('\n')
        fob.write('\nObstacle-watchers connections:\n')
        for i in range(int(textinput_num_obstacles.text)):
            fob.write(str(textinput_obstacle_watchers_connection_list[i].text))
            if i < int(textinput_num_obstacles.text) - 1:
                fob.write('\n')
        fob.close()

        # save the motion values in a file (again assume that the order is right and it starts with the motion for obstacle 0)
        fob = open('output/motion.txt','w') # the button is clicked only once
        max_obstacles = 20
        cur_obstacles = int(textinput_num_obstacles.text)
        if cur_obstacles <= max_obstacles:
            for i in range(cur_obstacles):
                print('dropdown value: ' + str(mainbutton_motion_list[i].text))
                fob.write(str(mainbutton_motion_list[i].text))
                if i < cur_obstacles - 1:
                    fob.write('\n')
                mainbutton_motion_list[i].disabled = True # the buttons should be disabled
        else:
            print("Max allowed obstacles are " + str(max_obstacles) + '!')
            os._exit(0)
        fob.close()

        print('The button <%s> is being pressed' % ' '.join(button2.text.split('\n')))
        parent.remove_widget(wimg_input_map) # !
        parent.remove_widget(layout_btn)
        parent.export_to_png("output/watchers_obstacles.png")
        for child in parent_draw.children:
            parent_draw.remove_widget(child)
        parent.remove_widget(parent_draw)
        parent.add_widget(wimg_input_map) # !
        parent_draw.add_widget(self.painter_line)
        parent.add_widget(parent_draw)
        parent.add_widget(layout_btn)
        wimg_watchers_obstacles = Image(source='output/watchers_obstacles.png', size=(parent.width, parent.height))
        parent.add_widget(wimg_watchers_obstacles)
        button2.disabled = True # disable the button, should be clicked only once!
        button3.disabled = False # enable the next button

    def button3_callback(self, parent, parent_draw, button3, button4, layout_btn, wimg_input_map):
        print('The button <%s> is being pressed' % ' '.join(button3.text.split('\n')))
        parent.remove_widget(wimg_input_map) # !
        parent.remove_widget(layout_btn)
        parent.export_to_png("output/watchers_obstacles_waypoints.png")
        for child in parent_draw.children:
            parent_draw.remove_widget(child)
        parent.remove_widget(parent_draw)
        parent.add_widget(wimg_input_map) # !
        parent_draw.add_widget(self.painter_robot)
        parent.add_widget(parent_draw)
        parent.add_widget(layout_btn)
        wimg_watchers_obstacles_waypoints = Image(source='output/watchers_obstacles_waypoints.png', size=(parent.width, parent.height))
        parent.add_widget(wimg_watchers_obstacles_waypoints)
        button3.disabled = True # disable the button, should be clicked only once!
        button4.disabled = False # enable the next button
        

    def button4_callback(self, parent, parent_draw, button4, layout_btn):
        print('The button <%s> is being pressed' % ' '.join(button4.text.split('\n')))
        parent.remove_widget(layout_btn)
        parent.export_to_png("output/ready.png")
        for child in parent_draw.children:
            parent_draw.remove_widget(child)
        parent.remove_widget(parent_draw)
        parent.add_widget(layout_btn)
        wimg_ready = Image(source='output/ready.png', size=(parent.width, parent.height))
        parent.add_widget(wimg_ready)
        button4.disabled = True # disable the button, should be clicked only once!
        label_done = Label(text='Done! Run parser.py and see the generated json file in the output folder!', pos=(Window.size[1]*1.7/3,Window.size[1]-60)) # write a label, that the image and the json are ready (generated in the root folder)
        parent.add_widget(label_done)
        # TODO: maybe even terminate?
        # As a next step -> the python script parser.py should be run

    def button_obstacle_type(self, mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res, color_r_temp, color_g_temp, color_b_temp):
        global counter
        counter += 1
        
        # IDEA: make the obstacle circles appear with a different color, save it first as an image, upload it again,
        # delete the obstacle widget and generate another with different color
        parent.remove_widget(wimg_input_map) # !
        parent.remove_widget(layout_btn)
        parent.remove_widget(layout_origin)
        parent.remove_widget(layout_res)
        parent.remove_widget(mainbutton_obstacle_type) # important so that the chosen type could be refreshed into the dropdown button !

        # delete the image when done, it should be just temporary, so that a new image with the same name could be later saved again ! (still another method is used)
        #if os.path.exists("output/obstacles_temp.png"):
        #    os.remove("output/obstacles_temp.png")
        #parent.export_to_png("output/obstacles_temp.png")
        parent.export_to_png("output/obstacles_temp__" + str(counter) + ".png")
        for child in parent_draw.children: # !
            parent_draw.remove_widget(child)
        parent.remove_widget(parent_draw)
        
        global color_r # needed to be able to change a global variable!
        global color_g
        global color_b
        color_r = color_r_temp
        color_g = color_g_temp
        color_b = color_b_temp
        painter_circle_obstacle_new = MyPaintWidgetCircleObstacle() # generate another widget with the same behaviour!
        
        parent.add_widget(wimg_input_map) # !

        #wimg_obstacles_temp = Image(source='output/obstacles_temp.png', size=(parent.width, parent.height))
        # the problem with the line above if that the name of the image file is always the same !? (so it interprets it as the same widget? it didn't help to delete the image when done!?)
        wimg_obstacles_temp = Image(source="output/obstacles_temp__" + str(counter) + ".png", size=(parent.width, parent.height))
        parent.add_widget(wimg_obstacles_temp)

        parent_draw.add_widget(painter_circle_obstacle_new)
        parent.add_widget(parent_draw)
        parent.add_widget(layout_btn)
        parent.add_widget(layout_origin)
        parent.add_widget(layout_res)
        parent.add_widget(mainbutton_obstacle_type) # !

        # Important: because of multiple obstacle types, the return button should be updated!

    def button_obstacle_type_cleaner_callback_down(self, mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res):
        print('Obstacle type was chosen - cleaner')
        # blue
        color_r_temp = 0
        color_g_temp = 0
        color_b_temp = 1
        self.button_obstacle_type(mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res, color_r_temp, color_g_temp, color_b_temp)

    def button_obstacle_type_random_callback_down(self, mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res):
        print('Obstacle type was chosen - random')
        # orange
        color_r_temp = 1
        color_g_temp = 0.5
        color_b_temp = 0
        self.button_obstacle_type(mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res, color_r_temp, color_g_temp, color_b_temp)

    def button_obstacle_type_turtlebot_callback_down(self, mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res):
        print('Obstacle type was chosen - turtlebot')
        # pink
        color_r_temp = 1
        color_g_temp = 0
        color_b_temp = 0.5
        self.button_obstacle_type(mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res, color_r_temp, color_g_temp, color_b_temp)

    def button_obstacle_type_walker_callback_down(self, mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res):
        print('Obstacle type was chosen - walker')
        # purple
        color_r_temp = 0.5
        color_g_temp = 0
        color_b_temp = 0.5
        self.button_obstacle_type(mainbutton_obstacle_type, parent, wimg_input_map, parent_draw, layout_btn, layout_origin, layout_res, color_r_temp, color_g_temp, color_b_temp)

    # return button -> return one step (so if now drawing lines is turned on, after the button is clicked, start fresh with the lines, so delete all lines (and update/restart the coresponding txt file!))
    def button_return_callback_down(self, parent, parent_draw, button, button2, button3, button4, painter_circle_obstacle, painter_circle_watcher, painter_line, painter_robot):
        if button.disabled == False:
            print('Drawing obstacles is enabled -> return to the beginning of this stage!')
            parent.remove_widget(painter_circle_obstacle)
            for child in parent_draw.children:
                parent_draw.remove_widget(child)
            painter_circle_obstacle_new = MyPaintWidgetCircleObstacle() # generate another widget with the same behaviour!
            parent_draw.add_widget(painter_circle_obstacle_new)
            
            # IDEA 1: a single obstacle type
            #if os.path.exists("output/obstacle.txt"):
            #    os.remove("output/obstacle.txt") # reset the txt-file

            # IDEA 2: multiple obstacle types -> delete only the last lines with the last obstacle type from the txt file

            # test if the chosen type in the dropdown button is the last saved type, if not, print "nothing to delete!"
            last_type_chosed_dropdown = 'default'
            for obst_type in obstacle_type:
                if (obst_type[1][0] == color_r) and (obst_type[1][1] == color_g) and (obst_type[1][2] == color_b):
                    last_type_chosed_dropdown = obst_type[0]

            lines_obstacles = []
            last_type_clicked = 'default'
            if os.path.exists("output/obstacle.txt"):
                with open('output/obstacle.txt', "r") as file:
                    lines_obstacles = file.readlines()
                last_type_clicked = lines_obstacles[len(lines_obstacles)-1].split('obstacle (x,y,radius,type): ')[1].split(',')[3].split('\n')[0]
            lines_remove = []
            count = len(lines_obstacles)
            if last_type_chosed_dropdown == last_type_clicked: # only if the chosen type in the dropdown is the same as the last clicked obstacle type in the map
                for line in reversed(lines_obstacles):
                    if line.split('obstacle (x,y,radius,type): ')[1].split(',')[3].split('\n')[0] == last_type_clicked:
                        lines_remove.insert(0,count)
                    else:
                        break
                    count -= 1
            if len(lines_remove) != 0:
                print('Remove lines with type ' + last_type_chosed_dropdown + ': ' + str(lines_remove))
            else:
                print('Nothing to remove!')
            count = 0
            bool_test = 0
            # do not delete every obstacle from the current type, but the last obstacles with the current type (so type 1, type 2, type 1, return will leave type 1, type 2 (only without the second type 1))
            # -> important, otherwise the new following index will be wrong, because deleted from the file, but not from the map, because already saved as an image
            if os.path.exists("output/obstacle.txt"):
                with open('output/obstacle.txt', "w") as file:
                    for line in lines_obstacles:
                        count += 1
                        # start from the last line and delete only until a new type comes
                        for line_remove in lines_remove:
                            if count == line_remove:
                                bool_test = 1
                                break
                        if bool_test == 0:
                            file.write(line)

        elif button2.disabled == False:
            print('Drawing watchers is enabled -> return to the beginning of this stage!')
            parent.remove_widget(painter_circle_watcher)
            for child in parent_draw.children:
                parent_draw.remove_widget(child)
            painter_circle_watcher_new = MyPaintWidgetCircleWatcher() # generate another widget with the same behaviour!
            parent_draw.add_widget(painter_circle_watcher_new)
            if os.path.exists("output/watcher.txt"):
                os.remove("output/watcher.txt") # reset the txt-file
        elif button3.disabled == False:
            print('Drawing vectors is enabled -> return to the beginning of this stage!')
            parent.remove_widget(painter_line)
            for child in parent_draw.children:
                parent_draw.remove_widget(child)
            painter_line_new = MyPaintWidgetLine() # generate another widget with the same behaviour!
            parent_draw.add_widget(painter_line_new)
            if os.path.exists("output/vector.txt"):
                os.remove("output/vector.txt") # reset the txt-file
        elif button4.disabled == False:
            print('Drawing robot positions is enabled -> return to the beginning of this stage!')
            parent.remove_widget(painter_robot)
            for child in parent_draw.children:
                parent_draw.remove_widget(child)
            painter_robot_new = MyPaintWidgetRobot() # generate another widget with the same behaviour!
            parent_draw.add_widget(painter_robot_new)
            if os.path.exists("output/robot.txt"):
                os.remove("output/robot.txt") # reset the txt-file
        else:
            print('All done, nothing to return!')
    
    def set_obstacle_params(self, textinput_num_obstacles, label_index_list, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list, height_layout_connect, width_layout_connect, width_left_border, height_up_border): # IDEA 1 - make everything with text inputs (so save from here the information also about the motions)
        # scrollable (make place for setting up 10 obstacles before the area gets scrollable) (https://kivy.org/doc/stable/api-kivy.uix.scrollview.html)
        layout_connect = GridLayout(cols=4, size_hint_y=None)
        layout_connect.bind(minimum_height=layout_connect.setter('height'))
        for index in range(int(textinput_num_obstacles.text)): # index starts with 0
            label_index_list.append(Label(text=str(index), size_hint_x=None, width=15, size_hint_y=None, height=height_layout_connect/2/10)) # height=28=height_layout_connect/2/10 to have place for 10 obstacle before it starts to scroll
            textinput_velocity_list.append(TextInput(text='0.3', size_hint_y=None, height=height_layout_connect/2/10)) # editable; give an example value already written in the box
            textinput_obstacle_watchers_connection_list.append(TextInput(text=str(index), size_hint_y=None, height=height_layout_connect/2/10))
            mainbutton_motion_list.append(TextInput(text='yoyo', size_hint=(None,None), height=height_layout_connect/2/10, width=50))
            layout_connect.add_widget(label_index_list[index])
            layout_connect.add_widget(textinput_velocity_list[index])
            layout_connect.add_widget(textinput_obstacle_watchers_connection_list[index])
            layout_connect.add_widget(mainbutton_motion_list[index])
        scrollable_area = ScrollView(size_hint=(1, None), size=(width_layout_connect,height_layout_connect/2), pos=(Window.size[0]-width_left_border-140, Window.size[1]-height_up_border-height_layout_connect/2-30))
        scrollable_area.add_widget(layout_connect)
        return scrollable_area

    def set_obstacle_params_2(self, textinput_num_obstacles, label_index_list, textinput_velocity_list, textinput_obstacle_watchers_connection_list, mainbutton_motion_list, height_layout_connect, width_layout_connect, width_left_border, height_up_border): # IDEA 2 - make text inputs only for the velocity and watchers -> for the motions make dropdown boxes
        ## not scrollable (if more then 10 obstacles -> too small boxes -> can not be read anymore!)
        #layout_connect = GridLayout(cols=3, rows=int(textinput_num_obstacles.text), size=(width_layout_connect-50,height_layout_connect/2), size_hint=(None, None), pos=(Window.size[0]-width_left_border-140, Window.size[1]-height_up_border-height_layout_connect/2-30))
        ## scrollable (make place for setting up 10 obstacles before the area gets scrollable)
        layout_connect = GridLayout(cols=3, size_hint_y=None)
        layout_connect.bind(minimum_height=layout_connect.setter('height'))
        for index in range(int(textinput_num_obstacles.text)): # index starts with 0
            label_index_list.append(Label(text=str(index), size_hint_x=None, width=15, size_hint_y=None, height=height_layout_connect/2/10)) # height=28=height_layout_connect/2/10 to have place for 10 obstacle before it starts to scroll
            textinput_velocity_list.append(TextInput(text='0.3', size_hint_y=None, height=height_layout_connect/2/10)) # editable; give an example value already written in the box
            textinput_obstacle_watchers_connection_list.append(TextInput(text=str(index), size_hint_y=None, height=height_layout_connect/2/10)) # editable; give an example value already written in the box
            layout_connect.add_widget(label_index_list[index])
            layout_connect.add_widget(textinput_velocity_list[index])
            layout_connect.add_widget(textinput_obstacle_watchers_connection_list[index])
        scrollable_area = ScrollView(size_hint=(1, None), size=(width_layout_connect-50,height_layout_connect/2), pos=(Window.size[0]-width_left_border-140, Window.size[1]-height_up_border-height_layout_connect/2-30))
        scrollable_area.add_widget(layout_connect)

        # TODO NEXT!
        # multiple drop-down buttons does not work correctly!? => for now hard coded for max 20 obstacles -> make the hard-coded part in another file!?
        ## not scrollable
        #layout_connect_2 = GridLayout(cols=1, rows=int(textinput_num_obstacles.text), size=(50,height_layout_connect/2), size_hint=(None, None), pos=(Window.size[0]-width_left_border-50, Window.size[1]-height_up_border-height_layout_connect/2-30))
        ## scrollable
        layout_connect_2 = GridLayout(cols=1, size_hint_y=None)
        layout_connect_2.bind(minimum_height=layout_connect_2.setter('height'))
        #for index in range(int(textinput_num_obstacles.text)):
        #    dropdown_motion_list.append(DropDown()) # drop-down menu with different motions
        #    btn_yoyo_list.append(Button(text='yoyo', size_hint_y=None))
        #    btn_circle_list.append(Button(text='circle', size_hint_y=None))
        #    mainbutton_motion_list.append(Button(text='move'))
        #    btn_yoyo_list[index].bind(on_release=lambda btn: dropdown_motion_list[index].select(btn_yoyo_list[index].text))
        #    dropdown_motion_list[index].add_widget(btn_yoyo_list[index])
        #    btn_circle_list[index].bind(on_release=lambda btn: dropdown_motion_list[index].select(btn_circle_list[index].text))
        #    dropdown_motion_list[index].add_widget(btn_circle_list[index])
        #    mainbutton_motion_list[index].bind(on_release=dropdown_motion_list[index].open)
        #    dropdown_motion_list[index].bind(on_select=lambda instance, x: setattr(mainbutton_motion_list[index], 'text', x)) # TODO NEXT: mainbutton_motion_list[0]; mainbutton_motion_list[index-1]; only the button from the last iteration in the loop works correctly
        #    layout_connect_2.add_widget(mainbutton_motion_list[index])
        #    ##layout_connect.add_widget(self.dropdown_get()[index]) # even that does not work out
        ## alternative: create different variable names in a loop (not that good practice, better use a list) -> globals()['dropdown_motion_%s' % index] = DropDown() OR globals()["dropdown_motion_"+str(index)] = DropDown() # still does not work, the same as with the lists
        self.dropdown_get(int(textinput_num_obstacles.text), layout_connect_2, mainbutton_motion_list, height_layout_connect) # TODO NEXT: for now hard coded for max 10 obstacles
        scrollable_area_2 = ScrollView(size_hint=(1, None), size=(50,height_layout_connect/2), pos=(Window.size[0]-width_left_border-50, Window.size[1]-height_up_border-height_layout_connect/2-30))
        scrollable_area_2.add_widget(layout_connect_2)

        return (scrollable_area, scrollable_area_2)

    def dropdown_get(self, r, layout_connect_2, mainbutton_motion_list, height_layout_connect): # no other way was found out -> so hard coded for max 10 obstacles
        dropdown_motion_ = DropDown() # drop-down menu with different motions
        btn_yoyo_ = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_.bind(on_release=lambda btn: dropdown_motion_.select(btn_yoyo_.text))
        dropdown_motion_.add_widget(btn_yoyo_)
        btn_circle_ = Button(text='circle', size_hint_y=None)
        btn_circle_.bind(on_release=lambda btn: dropdown_motion_.select(btn_circle_.text))
        dropdown_motion_.add_widget(btn_circle_)
        mainbutton_motion_ = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None # Button(text='move') will separate the siven space so that all buttons fit # Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) will demand that each button has this height and will make the area scrollable!
        mainbutton_motion_.bind(on_release=dropdown_motion_.open)
        dropdown_motion_.bind(on_select=lambda instance, x: setattr(mainbutton_motion_, 'text', x))

        dropdown_motion_1 = DropDown() # drop-down menu with different motions
        btn_yoyo_1 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_1.bind(on_release=lambda btn: dropdown_motion_1.select(btn_yoyo_1.text))
        dropdown_motion_1.add_widget(btn_yoyo_1)
        btn_circle_1 = Button(text='circle', size_hint_y=None)
        btn_circle_1.bind(on_release=lambda btn: dropdown_motion_1.select(btn_circle_1.text))
        dropdown_motion_1.add_widget(btn_circle_1)
        mainbutton_motion_1 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_1.bind(on_release=dropdown_motion_1.open)
        dropdown_motion_1.bind(on_select=lambda instance, x: setattr(mainbutton_motion_1, 'text', x))

        dropdown_motion_2 = DropDown() # drop-down menu with different motions
        btn_yoyo_2 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_2.bind(on_release=lambda btn: dropdown_motion_2.select(btn_yoyo_2.text))
        dropdown_motion_2.add_widget(btn_yoyo_2)
        btn_circle_2 = Button(text='circle', size_hint_y=None)
        btn_circle_2.bind(on_release=lambda btn: dropdown_motion_2.select(btn_circle_2.text))
        dropdown_motion_2.add_widget(btn_circle_2)
        mainbutton_motion_2 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_2.bind(on_release=dropdown_motion_2.open)
        dropdown_motion_2.bind(on_select=lambda instance, x: setattr(mainbutton_motion_2, 'text', x))

        dropdown_motion_3 = DropDown() # drop-down menu with different motions
        btn_yoyo_3 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_3.bind(on_release=lambda btn: dropdown_motion_3.select(btn_yoyo_3.text))
        dropdown_motion_3.add_widget(btn_yoyo_3)
        btn_circle_3 = Button(text='circle', size_hint_y=None)
        btn_circle_3.bind(on_release=lambda btn: dropdown_motion_3.select(btn_circle_3.text))
        dropdown_motion_3.add_widget(btn_circle_3)
        mainbutton_motion_3 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_3.bind(on_release=dropdown_motion_3.open)
        dropdown_motion_3.bind(on_select=lambda instance, x: setattr(mainbutton_motion_3, 'text', x))

        dropdown_motion_4 = DropDown() # drop-down menu with different motions
        btn_yoyo_4 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_4.bind(on_release=lambda btn: dropdown_motion_4.select(btn_yoyo_4.text))
        dropdown_motion_4.add_widget(btn_yoyo_4)
        btn_circle_4 = Button(text='circle', size_hint_y=None)
        btn_circle_4.bind(on_release=lambda btn: dropdown_motion_4.select(btn_circle_4.text))
        dropdown_motion_4.add_widget(btn_circle_4)
        mainbutton_motion_4 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_4.bind(on_release=dropdown_motion_4.open)
        dropdown_motion_4.bind(on_select=lambda instance, x: setattr(mainbutton_motion_4, 'text', x))

        dropdown_motion_5 = DropDown() # drop-down menu with different motions
        btn_yoyo_5 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_5.bind(on_release=lambda btn: dropdown_motion_5.select(btn_yoyo_5.text))
        dropdown_motion_5.add_widget(btn_yoyo_5)
        btn_circle_5 = Button(text='circle', size_hint_y=None)
        btn_circle_5.bind(on_release=lambda btn: dropdown_motion_5.select(btn_circle_5.text))
        dropdown_motion_5.add_widget(btn_circle_5)
        mainbutton_motion_5 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_5.bind(on_release=dropdown_motion_5.open)
        dropdown_motion_5.bind(on_select=lambda instance, x: setattr(mainbutton_motion_5, 'text', x))

        dropdown_motion_6 = DropDown() # drop-down menu with different motions
        btn_yoyo_6 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_6.bind(on_release=lambda btn: dropdown_motion_6.select(btn_yoyo_6.text))
        dropdown_motion_6.add_widget(btn_yoyo_6)
        btn_circle_6 = Button(text='circle', size_hint_y=None)
        btn_circle_6.bind(on_release=lambda btn: dropdown_motion_6.select(btn_circle_6.text))
        dropdown_motion_6.add_widget(btn_circle_6)
        mainbutton_motion_6 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_6.bind(on_release=dropdown_motion_6.open)
        dropdown_motion_6.bind(on_select=lambda instance, x: setattr(mainbutton_motion_6, 'text', x))

        dropdown_motion_7 = DropDown() # drop-down menu with different motions
        btn_yoyo_7 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_7.bind(on_release=lambda btn: dropdown_motion_7.select(btn_yoyo_7.text))
        dropdown_motion_7.add_widget(btn_yoyo_7)
        btn_circle_7 = Button(text='circle', size_hint_y=None)
        btn_circle_7.bind(on_release=lambda btn: dropdown_motion_7.select(btn_circle_7.text))
        dropdown_motion_7.add_widget(btn_circle_7)
        mainbutton_motion_7 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_7.bind(on_release=dropdown_motion_7.open)
        dropdown_motion_7.bind(on_select=lambda instance, x: setattr(mainbutton_motion_7, 'text', x))

        dropdown_motion_8 = DropDown() # drop-down menu with different motions
        btn_yoyo_8 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_8.bind(on_release=lambda btn: dropdown_motion_8.select(btn_yoyo_8.text))
        dropdown_motion_8.add_widget(btn_yoyo_8)
        btn_circle_8 = Button(text='circle', size_hint_y=None)
        btn_circle_8.bind(on_release=lambda btn: dropdown_motion_8.select(btn_circle_8.text))
        dropdown_motion_8.add_widget(btn_circle_8)
        mainbutton_motion_8 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_8.bind(on_release=dropdown_motion_8.open)
        dropdown_motion_8.bind(on_select=lambda instance, x: setattr(mainbutton_motion_8, 'text', x))

        dropdown_motion_9 = DropDown() # drop-down menu with different motions
        btn_yoyo_9 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_9.bind(on_release=lambda btn: dropdown_motion_9.select(btn_yoyo_9.text))
        dropdown_motion_9.add_widget(btn_yoyo_9)
        btn_circle_9 = Button(text='circle', size_hint_y=None)
        btn_circle_9.bind(on_release=lambda btn: dropdown_motion_9.select(btn_circle_9.text))
        dropdown_motion_9.add_widget(btn_circle_9)
        mainbutton_motion_9 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_9.bind(on_release=dropdown_motion_9.open)
        dropdown_motion_9.bind(on_select=lambda instance, x: setattr(mainbutton_motion_9, 'text', x))

        dropdown_motion_10 = DropDown() # drop-down menu with different motions
        btn_yoyo_10 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_10.bind(on_release=lambda btn: dropdown_motion_10.select(btn_yoyo_10.text))
        dropdown_motion_10.add_widget(btn_yoyo_10)
        btn_circle_10 = Button(text='circle', size_hint_y=None)
        btn_circle_10.bind(on_release=lambda btn: dropdown_motion_10.select(btn_circle_10.text))
        dropdown_motion_10.add_widget(btn_circle_10)
        mainbutton_motion_10 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_10.bind(on_release=dropdown_motion_10.open)
        dropdown_motion_10.bind(on_select=lambda instance, x: setattr(mainbutton_motion_10, 'text', x))

        dropdown_motion_11 = DropDown() # drop-down menu with different motions
        btn_yoyo_11 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_11.bind(on_release=lambda btn: dropdown_motion_11.select(btn_yoyo_11.text))
        dropdown_motion_11.add_widget(btn_yoyo_11)
        btn_circle_11 = Button(text='circle', size_hint_y=None)
        btn_circle_11.bind(on_release=lambda btn: dropdown_motion_11.select(btn_circle_11.text))
        dropdown_motion_11.add_widget(btn_circle_11)
        mainbutton_motion_11 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_11.bind(on_release=dropdown_motion_11.open)
        dropdown_motion_11.bind(on_select=lambda instance, x: setattr(mainbutton_motion_11, 'text', x))

        dropdown_motion_12 = DropDown() # drop-down menu with different motions
        btn_yoyo_12 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_12.bind(on_release=lambda btn: dropdown_motion_12.select(btn_yoyo_12.text))
        dropdown_motion_12.add_widget(btn_yoyo_12)
        btn_circle_12 = Button(text='circle', size_hint_y=None)
        btn_circle_12.bind(on_release=lambda btn: dropdown_motion_12.select(btn_circle_12.text))
        dropdown_motion_12.add_widget(btn_circle_12)
        mainbutton_motion_12 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_12.bind(on_release=dropdown_motion_12.open)
        dropdown_motion_12.bind(on_select=lambda instance, x: setattr(mainbutton_motion_12, 'text', x))

        dropdown_motion_13 = DropDown() # drop-down menu with different motions
        btn_yoyo_13 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_13.bind(on_release=lambda btn: dropdown_motion_13.select(btn_yoyo_13.text))
        dropdown_motion_13.add_widget(btn_yoyo_13)
        btn_circle_13 = Button(text='circle', size_hint_y=None)
        btn_circle_13.bind(on_release=lambda btn: dropdown_motion_13.select(btn_circle_13.text))
        dropdown_motion_13.add_widget(btn_circle_13)
        mainbutton_motion_13 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_13.bind(on_release=dropdown_motion_13.open)
        dropdown_motion_13.bind(on_select=lambda instance, x: setattr(mainbutton_motion_13, 'text', x))

        dropdown_motion_14 = DropDown() # drop-down menu with different motions
        btn_yoyo_14 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_14.bind(on_release=lambda btn: dropdown_motion_14.select(btn_yoyo_14.text))
        dropdown_motion_14.add_widget(btn_yoyo_14)
        btn_circle_14 = Button(text='circle', size_hint_y=None)
        btn_circle_14.bind(on_release=lambda btn: dropdown_motion_14.select(btn_circle_14.text))
        dropdown_motion_14.add_widget(btn_circle_14)
        mainbutton_motion_14 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_14.bind(on_release=dropdown_motion_14.open)
        dropdown_motion_14.bind(on_select=lambda instance, x: setattr(mainbutton_motion_14, 'text', x))

        dropdown_motion_15 = DropDown() # drop-down menu with different motions
        btn_yoyo_15 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_15.bind(on_release=lambda btn: dropdown_motion_15.select(btn_yoyo_15.text))
        dropdown_motion_15.add_widget(btn_yoyo_15)
        btn_circle_15 = Button(text='circle', size_hint_y=None)
        btn_circle_15.bind(on_release=lambda btn: dropdown_motion_15.select(btn_circle_15.text))
        dropdown_motion_15.add_widget(btn_circle_15)
        mainbutton_motion_15 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_15.bind(on_release=dropdown_motion_15.open)
        dropdown_motion_15.bind(on_select=lambda instance, x: setattr(mainbutton_motion_15, 'text', x))

        dropdown_motion_16 = DropDown() # drop-down menu with different motions
        btn_yoyo_16 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_16.bind(on_release=lambda btn: dropdown_motion_16.select(btn_yoyo_16.text))
        dropdown_motion_16.add_widget(btn_yoyo_16)
        btn_circle_16 = Button(text='circle', size_hint_y=None)
        btn_circle_16.bind(on_release=lambda btn: dropdown_motion_16.select(btn_circle_16.text))
        dropdown_motion_16.add_widget(btn_circle_16)
        mainbutton_motion_16 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_16.bind(on_release=dropdown_motion_16.open)
        dropdown_motion_16.bind(on_select=lambda instance, x: setattr(mainbutton_motion_16, 'text', x))

        dropdown_motion_17 = DropDown() # drop-down menu with different motions
        btn_yoyo_17 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_17.bind(on_release=lambda btn: dropdown_motion_17.select(btn_yoyo_17.text))
        dropdown_motion_17.add_widget(btn_yoyo_17)
        btn_circle_17 = Button(text='circle', size_hint_y=None)
        btn_circle_17.bind(on_release=lambda btn: dropdown_motion_17.select(btn_circle_17.text))
        dropdown_motion_17.add_widget(btn_circle_17)
        mainbutton_motion_17 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_17.bind(on_release=dropdown_motion_17.open)
        dropdown_motion_17.bind(on_select=lambda instance, x: setattr(mainbutton_motion_17, 'text', x))

        dropdown_motion_18 = DropDown() # drop-down menu with different motions
        btn_yoyo_18 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_18.bind(on_release=lambda btn: dropdown_motion_18.select(btn_yoyo_18.text))
        dropdown_motion_18.add_widget(btn_yoyo_18)
        btn_circle_18 = Button(text='circle', size_hint_y=None)
        btn_circle_18.bind(on_release=lambda btn: dropdown_motion_18.select(btn_circle_18.text))
        dropdown_motion_18.add_widget(btn_circle_18)
        mainbutton_motion_18 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_18.bind(on_release=dropdown_motion_18.open)
        dropdown_motion_18.bind(on_select=lambda instance, x: setattr(mainbutton_motion_18, 'text', x))

        dropdown_motion_19 = DropDown() # drop-down menu with different motions
        btn_yoyo_19 = Button(text='yoyo', size_hint_y=None)
        btn_yoyo_19.bind(on_release=lambda btn: dropdown_motion_19.select(btn_yoyo_19.text))
        dropdown_motion_19.add_widget(btn_yoyo_19)
        btn_circle_19 = Button(text='circle', size_hint_y=None)
        btn_circle_19.bind(on_release=lambda btn: dropdown_motion_19.select(btn_circle_19.text))
        dropdown_motion_19.add_widget(btn_circle_19)
        mainbutton_motion_19 = Button(text='move', size_hint_y=None, height=height_layout_connect/2/10) #size_hint_y=None
        mainbutton_motion_19.bind(on_release=dropdown_motion_19.open)
        dropdown_motion_19.bind(on_select=lambda instance, x: setattr(mainbutton_motion_19, 'text', x))
        if(r > 0):
            layout_connect_2.add_widget(mainbutton_motion_)
            mainbutton_motion_list.append(mainbutton_motion_) # a global variable -> necessary for motion.txt
        if(r > 1):
            layout_connect_2.add_widget(mainbutton_motion_1)
            mainbutton_motion_list.append(mainbutton_motion_1)
        if(r > 2):
            layout_connect_2.add_widget(mainbutton_motion_2)
            mainbutton_motion_list.append(mainbutton_motion_2)
        if(r > 3):
            layout_connect_2.add_widget(mainbutton_motion_3)
            mainbutton_motion_list.append(mainbutton_motion_3)
        if(r > 4):
            layout_connect_2.add_widget(mainbutton_motion_4)
            mainbutton_motion_list.append(mainbutton_motion_4)
        if(r > 5):
            layout_connect_2.add_widget(mainbutton_motion_5)
            mainbutton_motion_list.append(mainbutton_motion_5)
        if(r > 6):
            layout_connect_2.add_widget(mainbutton_motion_6)
            mainbutton_motion_list.append(mainbutton_motion_6)
        if(r > 7):
            layout_connect_2.add_widget(mainbutton_motion_7)
            mainbutton_motion_list.append(mainbutton_motion_7)
        if(r > 8):
            layout_connect_2.add_widget(mainbutton_motion_8)
            mainbutton_motion_list.append(mainbutton_motion_8)
        if(r > 9):
            layout_connect_2.add_widget(mainbutton_motion_9)
            mainbutton_motion_list.append(mainbutton_motion_9)
        if(r > 10):
            layout_connect_2.add_widget(mainbutton_motion_10)
            mainbutton_motion_list.append(mainbutton_motion_10)
        if(r > 11):
            layout_connect_2.add_widget(mainbutton_motion_11)
            mainbutton_motion_list.append(mainbutton_motion_11)
        if(r > 12):
            layout_connect_2.add_widget(mainbutton_motion_12)
            mainbutton_motion_list.append(mainbutton_motion_12)
        if(r > 13):
            layout_connect_2.add_widget(mainbutton_motion_13)
            mainbutton_motion_list.append(mainbutton_motion_13)
        if(r > 14):
            layout_connect_2.add_widget(mainbutton_motion_14)
            mainbutton_motion_list.append(mainbutton_motion_14)
        if(r > 15):
            layout_connect_2.add_widget(mainbutton_motion_15)
            mainbutton_motion_list.append(mainbutton_motion_15)
        if(r > 16):
            layout_connect_2.add_widget(mainbutton_motion_16)
            mainbutton_motion_list.append(mainbutton_motion_16)
        if(r > 17):
            layout_connect_2.add_widget(mainbutton_motion_17)
            mainbutton_motion_list.append(mainbutton_motion_17)
        if(r > 18):
            layout_connect_2.add_widget(mainbutton_motion_18)
            mainbutton_motion_list.append(mainbutton_motion_18)
        if(r > 19):
            layout_connect_2.add_widget(mainbutton_motion_19)
            mainbutton_motion_list.append(mainbutton_motion_19)
        
        #return (mainbutton_motion_, mainbutton_motion_1, mainbutton_motion_2, mainbutton_motion_3, mainbutton_motion_4, mainbutton_motion_5, mainbutton_motion_6, mainbutton_motion_7, mainbutton_motion_8, mainbutton_motion_9)

if __name__ == '__main__':
    MyPaintApp().run()