import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
import  math
import dearpygui.dearpygui as dpg
from utils.logger import mvLogger 


dpg.create_context()


out_x = [0]
out_y = [0]
in_x = [0]
in_y = [0]

# TOPICS
PUB_SIMPLE = "dearpy_simple_pub"

SUB_COUNTER = "dearpy_counter"


# LAYOUT
CONTORL_PANEL_POS = (0,0)
LOGGER_POS = (0,900)

#  ROS
def get_counter(msg):
    counter_sub_data = dpg.set_value(item="Counter_Sub", value=f"{msg.data}")
    out_x.append(out_x[-1]+1)
    out_y.append(msg.data)
    in_x.append(in_x[-1]+1)
    in_y.append(in_x[-1]+1)
    update_series()
    


rospy.init_node("Dearpy_Node")
pub_simple = rospy.Publisher(PUB_SIMPLE, String, queue_size=2)
rospy.Subscriber(SUB_COUNTER, Int16,  get_counter)


#  INTERFACE
logger = mvLogger(pos=LOGGER_POS)

def button_callback(sender, app_data):
    msg = dpg.get_value(item="publisher_text")
    pub_simple.publish(data=msg)
    logger.log_info(message=f"Publisher Simple {msg}")

def manual_control(sender, app_data, user_data):
    print(f"sender: {sender}, \t app_data: {app_data}, \t user_data: {user_data}")
    speed = dpg.get_value(item='Manual_Speed')
    if user_data=='UP':
        pub_simple.publish(f"UP Speed {speed}")
        logger.log_info(message=f"Published {speed} UP")


    elif user_data=='DOWN':
        pub_simple.publish(f"DOWN Speed {speed}")
        logger.log_info(message=f"Published {speed} DOWN")

    
    elif user_data=='LEFT':
        pub_simple.publish(f"LEFT Speed {speed}")
        logger.log_info(message=f"Published {speed} LEFT")


    elif user_data=='RIGHT':
        pub_simple.publish(f"RIGHT Speed {speed}")
        logger.log_info(message=f"Published {speed} RIGHT")

    if speed > 100:
        logger.log_warning(f"Speed is {speed}")



def update_series():
    dpg.set_value('series_tag_output', [out_x, out_y])
    dpg.set_item_label('series_tag_output', "Output")

    dpg.set_value('series_tag_input', [in_x, in_y])
    dpg.set_item_label('series_tag_input', "Input")

    dpg.fit_axis_data('x_axis')
    dpg.fit_axis_data('y_axis') 


def _on_demo_close(sender, app_data, user_data):
    dpg.delete_item(sender)

# add a font registry
with dpg.font_registry():
    # first argument ids the path to the .ttf or .otf file
    default_font = dpg.add_font("dearpy-ros/Assets/Fonts/Retron2000.ttf", 30)


with dpg.window(label="Control_Panel", height=900, width=600, pos=CONTORL_PANEL_POS, on_close=_on_demo_close):
    #set font of specific widget
    dpg.bind_font(default_font)

    # Menu bar
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

    # with dpg.window(label="Hello Publisher", height=400, width=600, pos=(0,0)):
    #     dpg.add_button(label="Print to Terminal", callback=button_callback)

    #     # set font of specific widget
    #     dpg.bind_font(default_font)

    dpg.add_separator()

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

    with dpg.collapsing_header(label="Manual Control"):
        with dpg.group(horizontal=True, indent=250):
            dpg.add_button(label="UP", callback=manual_control,indent=20, arrow=True, direction=dpg.mvDir_Up,user_data='UP',tag='UP_button') # default direction is mvDir_Up

        with dpg.group(horizontal=True, indent=250):
            dpg.add_button(label="LEFT", callback=manual_control, arrow=True, user_data='LEFT' , direction=dpg.mvDir_Left)
            dpg.add_button(label="RIGHT", callback=manual_control, arrow=True, user_data='RIGHT', direction=dpg.mvDir_Right)

        with dpg.group(horizontal=True,  indent=250):
            dpg.add_button(label="DOWN", callback=manual_control,indent=20, arrow=True, user_data='DOWN' ,direction=dpg.mvDir_Down) # default direction is mvDir_Up

        dpg.add_slider_float(label="Speed", default_value=50, max_value=255, tag='Manual_Speed')



    with dpg.window(label="Plots", width=1200, height=500, pos=(600,400)):
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
    for i in range(0, 1200 * 400):
        texture_data.append(255 / 255)
        texture_data.append(0)
        texture_data.append(255 / 255)
        texture_data.append(255 / 255)

    with dpg.texture_registry(show=True):
        dpg.add_dynamic_texture(width=1200, height=400, default_value=texture_data, tag="camera_stream")


    def _update_dynamic_textures(sender, app_data, user_data):
        new_color = dpg.get_value(sender)
        new_color[0] = new_color[0] / 255
        new_color[1] = new_color[1] / 255
        new_color[2] = new_color[2] / 255
        new_color[3] = new_color[3] / 255

        new_texture_data = []
        for i in range(0, 400 * 1200):
            new_texture_data.append(new_color[0])
            new_texture_data.append(new_color[1])
            new_texture_data.append(new_color[2])
            new_texture_data.append(new_color[3])

        dpg.set_value("camera_stream", new_texture_data)


    with dpg.window(label="Stream", pos=(600,0), height=400):
        dpg.add_image("camera_stream")
        # dpg.add_color_picker((255, 0, 255, 255), label="Texture",
        #                     no_side_preview=True, alpha_bar=True, width=150,height=150,
        #                     callback=_update_dynamic_textures)


dpg.create_viewport(title='ROS App', width=1800, height=1400, small_icon=r'dearpy-ros/Assets/icons/robot.png', large_icon=r'dearpy-ros/Assets/icons/robot.ico')
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
