import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
import ros_numpy


import cv2
import time
import  math
import ros_numpy
import numpy as np
import dearpygui.dearpygui as dpg
from utils.logger import mvLogger 
# from utils.keyboard import get_key
from pynput import keyboard
from subprocess import Popen




dpg.create_context()


out_x = [0]
out_y = [0]
in_x = [0]
in_y = [0]

# DIR
BAGS = "dearpy-ros/bags"

# TOPICS
PUB_SIMPLE = "dearpy_simple_pub"
PUB_CMD    = "cmd_vel"

SUB_COUNTER = "dearpy_counter"
SUB_ODOM = "odom"
SUB_LEFT_RAW = "/tortabot/camera/left/image_raw"
SUB_RIGHT_RAW = "/tortabot/camera/right/image_raw"


# LAYOUT
CONTORL_PANEL_POS = (0,100)
LOGGER_POS = (0,1000)
MENU_BAR_POS = (0,0)


# GLOBALS 
bag_process = ''


#  ROS
def get_counter(msg):
    counter_sub_data = dpg.set_value(item="Counter_Sub", value=f"{msg.data}")
    out_x.append(out_x[-1]+1)
    out_y.append(msg.data)
    in_x.append(in_x[-1]+1)
    in_y.append(in_x[-1]+1)
    update_series()

def get_odom(msg):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    theta = msg.pose.pose.orientation.z

    dpg.set_value(item="state_x", value=f"{x:.5}")
    dpg.set_value(item="state_y", value=f"{y:.5}")
    dpg.set_value(item="state_theta", value=f"{theta:.5}")

def np_to_list(img):
    pass

def get_left_stream(msg):
    start = time.time()

    # left_img = []
    left_stream = ros_numpy.numpify(msg)
    rgba = cv2.cvtColor(left_stream, cv2.COLOR_RGB2RGBA)
    rgba[:,:,3] = 255 
    left_stream = cv2.resize(rgba, (640,350))/255.0
    left_stream = list(left_stream.reshape(-1))

    # for i in range(0,350):
    #     for j in range(0,640):
    #         left_img.append(left_stream[i,j,0])
    #         left_img.append(left_stream[i,j,1])
    #         left_img.append(left_stream[i,j,2])
    #         left_img.append(1)

    end = time.time()

    dpg.set_value("camera_left_stream", left_stream)

def get_right_stream(msg):
    start = time.time()

    # right_img = []
    right_stream = ros_numpy.numpify(msg)
    rgba = cv2.cvtColor(right_stream, cv2.COLOR_RGB2RGBA)
    rgba[:,:,3] = 255 
    right_stream = cv2.resize(rgba, (640,350))/255.0
    right_stream = list(right_stream.reshape(-1))
    
    end = time.time()

    dpg.set_value("camera_right_stream",     right_stream)




rospy.init_node("Dearpy_Node")
pub_simple = rospy.Publisher(PUB_SIMPLE, String, queue_size=2)
pub_cmd    = rospy.Publisher(PUB_CMD, Twist, queue_size=2)

rospy.Subscriber(SUB_COUNTER, Int16,  get_counter)
rospy.Subscriber(SUB_ODOM, Odometry,  get_odom)
rospy.Subscriber(SUB_LEFT_RAW, Image,  get_left_stream, queue_size=1)
rospy.Subscriber(SUB_RIGHT_RAW, Image,  get_right_stream, queue_size=1)


#  INTERFACE
logger = mvLogger(pos=LOGGER_POS)

def button_callback(sender, app_data):
    msg = dpg.get_value(item="publisher_text")
    pub_simple.publish(data=msg)
    logger.log_info(message=f"Publisher Simple {msg}")

def get_key(sender, app_data):
    if dpg.is_key_down(dpg.mvKey_W):
        print("Key W is pressed")

def check_keyboard():
    key_check = dpg.get_value('keyboard_checkbox')
    print(key_check)


def check_joy():
    print("Here")


def manual_control(sender, app_data, user_data):
    '''
    This Function for Take Manual control from GUI
    '''
    print(f"sender: {sender}, \t app_data: {app_data}, \t user_data: {user_data}")
    speed = dpg.get_value(item='Manual_Speed')
    cmd_speeds = Twist()


    if  user_data=="STOP":
        pub_simple.publish(f"UP Speed {speed}")
        logger.log_info(message=f"Published Stop")
        


    elif user_data=='UP':
        pub_simple.publish(f"UP Speed {speed}")
        logger.log_info(message=f"Published {speed} UP")
        cmd_speeds.linear.x = speed

    elif user_data=='DOWN':
        pub_simple.publish(f"DOWN Speed {speed}")
        logger.log_info(message=f"Published {speed} DOWN")
        cmd_speeds.linear.x = -speed


    
    elif user_data=='LEFT':
        pub_simple.publish(f"LEFT Speed {speed}")
        logger.log_info(message=f"Published {speed} LEFT")
        cmd_speeds.angular.z = -speed



    elif user_data=='RIGHT':
        pub_simple.publish(f"RIGHT Speed {speed}")
        logger.log_info(message=f"Published {speed} RIGHT")
        cmd_speeds.angular.z = speed


    if speed > 5:
        logger.log_warning(f"Speed is {speed}")

    pub_cmd.publish(cmd_speeds)


def record_bag(sender, app_data, user_data):
    global bag_process
    # Unpack the user_data that is currently associated with the button
    state, enabled_theme, disabled_theme = user_data



    if   not state:
        bag_name = dpg.get_value("bag_name")
        bag_dir  = f"{BAGS}/{bag_name}.bag" 
        bag_process = Popen(f"rosbag record -a -o {bag_dir}", shell=True)
        dpg.set_item_label(item='record_bag', label='Stop' )
    else:
        bag_process.kill()
        dpg.set_item_label(item='record_bag', label='Record' )

    # Flip the state
    state = not state
    # Apply the appropriate theme
    dpg.bind_item_theme(sender, enabled_theme if state is False else disabled_theme)
    # Update the user_data associated with the button
    dpg.set_item_user_data(sender, (state, enabled_theme, disabled_theme,))


def update_series():
    dpg.set_value('series_tag_output', [out_x, out_y])
    dpg.set_item_label('series_tag_output', "Output")

    dpg.set_value('series_tag_input', [in_x, in_y])
    dpg.set_item_label('series_tag_input', "Input")

    dpg.fit_axis_data('x_axis')
    dpg.fit_axis_data('y_axis') 


def _hsv_to_rgb(h, s, v):
    if s == 0.0: return (v, v, v)
    i = int(h*6.) # XXX assume int() truncates!
    f = (h*6.)-i; p,q,t = v*(1.-s), v*(1.-s*f), v*(1.-s*(1.-f)); i%=6
    if i == 0: return (255*v, 255*t, 255*p)
    if i == 1: return (255*q, 255*v, 255*p)
    if i == 2: return (255*p, 255*v, 255*t)
    if i == 3: return (255*p, 255*q, 255*v)
    if i == 4: return (255*t, 255*p, 255*v)
    if i == 5: return (255*v, 255*p, 255*q)

def _on_demo_close(sender, app_data, user_data):
    dpg.delete_item(sender)

def keyboard_handlers(sender, app_data):
    if dpg.is_key_down(dpg.mvKey_C):
        dpg.destroy_context()

# add a font registry
with dpg.font_registry():
    # first argument ids the path to the .ttf or .otf file
    default_font = dpg.add_font("dearpy-ros/assets/Fonts/Retron2000.ttf", 30)

with dpg.theme(tag="button_theme"):
    with dpg.theme_component(dpg.mvButton):
        dpg.add_theme_color(dpg.mvThemeCol_Button, _hsv_to_rgb(7/7.0, 0.6, 0.6))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, _hsv_to_rgb(7/7.0, 0.8, 0.8))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, _hsv_to_rgb(7/7.0, 0.7, 0.7))
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 7*5)
        dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7*3, 7*3)

with dpg.theme() as enabled_theme:
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_Button, _hsv_to_rgb(7/7.0, 0.6, 0.6))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, _hsv_to_rgb(7/7.0, 0.8, 0.8))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, _hsv_to_rgb(7/7.0, 0.7, 0.7))
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 7*5)
        dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7*3, 7*3)
        dpg.add_theme_color(dpg.mvThemeCol_Text, (255, 255, 255), category=dpg.mvThemeCat_Core)

# - This theme should make the label text on the button red
with dpg.theme() as disabled_theme:
    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_color(dpg.mvThemeCol_Button, _hsv_to_rgb(7/7.0, 0.6, 0.6))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, _hsv_to_rgb(7/7.0, 0.8, 0.8))
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, _hsv_to_rgb(7/7.0, 0.7, 0.7))
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 7*5)
        dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 7*3, 7*3)
        dpg.add_theme_color(dpg.mvThemeCol_Text, (0, 0, 0), category=dpg.mvThemeCat_Core)


with dpg.window(label="Menu_Bar", height=100, width=1900 ,pos=MENU_BAR_POS, on_close=_on_demo_close, tag="menu_bar",no_close=True, no_collapse=True, no_move=True, no_resize=True):
    with dpg.menu_bar():
        with dpg.menu(label="Menu"):

            dpg.add_menu_item(label="New")
            dpg.add_menu_item(label="Open")

            with dpg.menu(label="Open Recent"):

                dpg.add_menu_item(label="harrel.c")
                dpg.add_menu_item(label="patty.h")
                dpg.add_menu_item(label="nick.py")

            dpg.add_menu_item(label="Save")
            dpg.add_menu_item(label="Save As...")

            with dpg.menu(label="Settings"):

                dpg.add_menu_item(label="Option 1", callback=manual_control)
                dpg.add_menu_item(label="Option 2", check=True, callback=manual_control)
                dpg.add_menu_item(label="Option 3", check=True, default_value=True, callback=manual_control)

                with dpg.child_window(height=60, autosize_x=True, delay_search=True):
                    for i in range(10):
                        dpg.add_text(f"Scolling Text{i}")

                dpg.add_slider_float(label="Slider Float")
                dpg.add_input_int(label="Input Int")
                dpg.add_combo(("Yes", "No", "Maybe"), label="Combo")

        with dpg.menu(label="Tools"):

            dpg.add_menu_item(label="Show About", callback=lambda:dpg.show_tool(dpg.mvTool_About))
            dpg.add_menu_item(label="Show Metrics", callback=lambda:dpg.show_tool(dpg.mvTool_Metrics))
            dpg.add_menu_item(label="Show Documentation", callback=lambda:dpg.show_tool(dpg.mvTool_Doc))
            dpg.add_menu_item(label="Show Debug", callback=lambda:dpg.show_tool(dpg.mvTool_Debug))
            dpg.add_menu_item(label="Show Style Editor", callback=lambda:dpg.show_tool(dpg.mvTool_Style))
            dpg.add_menu_item(label="Show Font Manager", callback=lambda:dpg.show_tool(dpg.mvTool_Font))
            dpg.add_menu_item(label="Show Item Registry", callback=lambda:dpg.show_tool(dpg.mvTool_ItemRegistry))

        with dpg.menu(label="Settings"):

            dpg.add_menu_item(label="Wait For Input", check=True, callback=lambda s, a: dpg.configure_app(wait_for_input=a))
            dpg.add_menu_item(label="Toggle Fullscreen", callback=lambda:dpg.toggle_viewport_fullscreen())

with dpg.window(label="Control_Panel", height=900, width=600, pos=CONTORL_PANEL_POS, on_close=_on_demo_close, tag="control_panel"):
    #set font of specific widget
    dpg.bind_font(default_font)

    # Menu bar
    with dpg.handler_registry():
        dpg.add_key_press_handler(dpg.mvKey_Control ,callback=keyboard_handlers)

    
    # with dpg.window(label="Hello Publisher", height=400, width=600, pos=(0,0)):
    #     dpg.add_button(label="Print to Terminal", callback=button_callback)

    #     # set font of specific widget
    #     dpg.bind_font(default_font)


    with dpg.collapsing_header(label="Simple Publisher"):

        with dpg.group(horizontal=True):
            '''
            Publisher
            '''
            dpg.add_button(label="Publisher", callback=button_callback)
            dpg.add_input_text(label="", default_value="Pub", tag='publisher_text')

    with dpg.collapsing_header(label="Simple Subscriber"):

        with dpg.group(horizontal=True):
            '''
            Subscriber
            '''
            dpg.add_text("Subscriber: ")
            counter_sub_data = dpg.add_text(tag='Counter_Sub', default_value="0" ,)


    with dpg.collapsing_header(label="State of Robot"):

            with dpg.group(horizontal=True):
                '''
                Subscriber
                '''
                dpg.add_text("x  :   ")
                dpg.add_text("0", tag='state_x')


            with dpg.group(horizontal=True):
                '''
                Subscriber
                '''
                dpg.add_text("y  :   ")
                dpg.add_text("0", tag='state_y')


            with dpg.group(horizontal=True):
                '''
                Subscriber
                '''
                dpg.add_text("t  :   ")
                dpg.add_text("0", tag='state_theta')


    with dpg.collapsing_header(label="Manual Control"):
        with dpg.group(horizontal=True):
            dpg.add_checkbox(label="Keyboard" ,callback=check_keyboard, tag='keyboard_checkbox')
            dpg.add_checkbox(label="Joy" , callback=check_joy, tag='joy_checkbox')


        with dpg.group(horizontal=True, indent=200):
            dpg.add_button(label="UP", callback=manual_control,width=100,indent=20, arrow=True, direction=dpg.mvDir_Up,user_data='UP',tag='up_button') # default direction is mvDir_Up

        with dpg.group(horizontal=True, indent=130):
            dpg.add_button(label="LEFT", callback=manual_control, arrow=True, user_data='LEFT' , direction=dpg.mvDir_Left, tag="left_button")
            dpg.add_button(label="STOP", callback=manual_control, user_data='STOP', tag='stop_button')
            dpg.add_button(label="RIGHT", callback=manual_control, arrow=True, user_data='RIGHT', direction=dpg.mvDir_Right, tag="right_button")

        with dpg.group(horizontal=True,  indent=200):
            dpg.add_button(label="DOWN", callback=manual_control,indent=20, arrow=True, user_data='DOWN' ,direction=dpg.mvDir_Down, tag="down_button") # default direction is mvDir_Up


        dpg.bind_item_theme(item='stop_button', theme="button_theme")
        dpg.bind_item_theme(item='right_button', theme="button_theme")
        dpg.bind_item_theme(item='left_button', theme="button_theme")
        dpg.bind_item_theme(item='up_button', theme="button_theme")
        dpg.bind_item_theme(item='down_button', theme="button_theme")
        dpg.add_slider_float(label="Speed", default_value=0.1, max_value=10, tag='Manual_Speed')
        # dpg.add_slider_float(label="Turn", default_value=0.1, max_value=10, tag='Manual_Speed')

        with dpg.handler_registry():
            dpg.add_key_press_handler( callback=get_key)


    ## ROSBAG
    with dpg.collapsing_header(label="Rosbag"):
        dpg.add_button(label="Record", callback=record_bag, indent=200 ,user_data=(False, enabled_theme, disabled_theme,), tag='record_bag')
        dpg.add_input_text(label="bag_name", default_value="", tag='bag_name')

        dpg.bind_item_theme(item='record_bag', theme=enabled_theme)





    with dpg.window(label="Plots", width=1300, height=500, pos=(600,500)):
        # create a theme for the plot
        with dpg.theme(tag="plot_theme_blue"):
            with dpg.theme_component(dpg.mvStemSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (150, 255, 0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Diamond, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)

            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (60, 150, 200), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Square, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 4, category=dpg.mvThemeCat_Plots)
        
        with dpg.theme(tag="plot_theme_red"):
            with dpg.theme_component(dpg.mvStemSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Diamond, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)

            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (255, 0, 0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Square, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 4, category=dpg.mvThemeCat_Plots)



        with dpg.plot(label='Line Series', height=-1, width=-1):
            # optionally create legend
            dpg.add_plot_legend()

            # REQUIRED: create x and y axes, set to auto scale.
            x_axis = dpg.add_plot_axis(dpg.mvXAxis, label='x', tag='x_axis')
            y_axis = dpg.add_plot_axis(dpg.mvYAxis, label='y', tag='y_axis')


            # series belong to a y axis. Note the tag name is used in the update
            # function update_data
            dpg.add_line_series(x=list(out_x),y=list(out_y), 
                                label='Counter_Output', parent='y_axis', 
                                tag='series_tag_output')
            
            dpg.add_line_series(x=list(in_x),y=list(in_y), 
                                label='Counter_Intput', parent='y_axis', 
                                tag='series_tag_input')
            


            # apply theme to series
            dpg.bind_item_theme("series_tag_output", "plot_theme_blue")
            dpg.bind_item_theme("series_tag_input", "plot_theme_red")



    # STREAM
    texture_data = []
    for i in range(0, 640 * 350):
        texture_data.append(0 / 255)
        texture_data.append(0)
        texture_data.append(255 / 255)
        texture_data.append(255 / 255)

    with dpg.texture_registry(show=True):
        dpg.add_dynamic_texture(width=640, height=350, default_value=texture_data,  tag="camera_left_stream")
        dpg.add_dynamic_texture(width=640, height=350, default_value=texture_data, tag="camera_right_stream")
        # dpg.add_raw_texture(width=1280, height=400, default_value=texture_data, format=dpg.mvFormat_Float_rgb, tag="camera_stream")



    with dpg.window(label="Left Camera", pos=(600,100), height=400,no_scrollbar=True):
        dpg.add_image("camera_left_stream")

    with dpg.window(label="Right Camera", pos=(1240,100), height=400,no_scrollbar=True):
        dpg.add_image("camera_right_stream")


    # with keyboard.Listener( on_press=on_press, on_release=on_release) as listener: 
    #     listener.join()


dpg.create_viewport(title='ROS App', width=1900, height=1400, small_icon=r'dearpy-ros/assets/icons/robot.png', large_icon=r'dearpy-ros/assets/icons/robot.ico')
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
