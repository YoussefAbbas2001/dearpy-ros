import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16

import  math
import dearpygui.dearpygui as dpg

dpg.create_context()


sindatax = []
sindatay = []
for i in range(0, 100):
    sindatax.append(i / 100)
    sindatay.append(0.5 + 0.5 * math.sin(50 * i / 100))
sindatay2 = []
for i in range(0, 100):
    sindatay2.append(2 + 0.5 * math.sin(50 * i / 100))

#  ROS

def get_counter(msg):
    counter_sub_data = dpg.set_value(item="Counter_Sub", value=f"{msg.data}")


rospy.init_node("Dearpy_Node")
dp_pub = rospy.Publisher("dearpy_simple_pub", String, queue_size=2)
rospy.Subscriber("dearpy_counter", Int16,  get_counter)


#  INTERFACE

def button_callback(sender, app_data):
    msg = dpg.get_value(item="publisher_text")
    dp_pub.publish(data=msg)
    print(f"You publisher at (/dearpy_simple_pub): {msg}")

def _log(sender, app_data, user_data):
    print(f"sender: {sender}, \t app_data: {app_data}, \t user_data: {user_data}")

def update_series():
    cosdatax = []
    cosdatay = []
    for i in range(0, 500):
        cosdatax.append(i / 1000)
        cosdatay.append(0.5 + 0.5 * math.cos(50 * i / 1000))
    dpg.set_value('series_tag', [cosdatax, cosdatay])
    dpg.set_item_label('series_tag', "0.5 + 0.5 * cos(x)")


def _on_demo_close(sender, app_data, user_data):
    dpg.delete_item(sender)

# add a font registry
with dpg.font_registry():
    # first argument ids the path to the .ttf or .otf file
    default_font = dpg.add_font("dearpy-ros/Assets/Fonts/Retron2000.ttf", 30)


with dpg.window(label="Dear_Interface", height=1200, width=600, pos=(0,0), on_close=_on_demo_close):
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

                dpg.add_menu_item(label="Option 1", callback=_log)
                dpg.add_menu_item(label="Option 2", check=True, callback=_log)
                dpg.add_menu_item(label="Option 3", check=True, default_value=True, callback=_log)

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


    with dpg.window(label="Plots", width=1200, height=600, pos=(600,0)):
        # create a theme for the plot
        with dpg.theme(tag="plot_theme"):
            with dpg.theme_component(dpg.mvStemSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (150, 255, 0), category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_Marker, dpg.mvPlotMarker_Diamond, category=dpg.mvThemeCat_Plots)
                dpg.add_theme_style(dpg.mvPlotStyleVar_MarkerSize, 7, category=dpg.mvThemeCat_Plots)

            with dpg.theme_component(dpg.mvScatterSeries):
                dpg.add_theme_color(dpg.mvPlotCol_Line, (60, 150, 200), category=dpg.mvThemeCat_Plots)
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
            dpg.add_line_series(x=list(sindatax),y=list(sindatay), 
                                label='Temp', parent='y_axis', 
                                tag='series_tag')
            
            # apply theme to series
            dpg.bind_item_theme("series_tag", "plot_theme")







dpg.create_viewport(title='ROS App', width=1800, height=1600, small_icon=r'dearpy-ros/Assets/icons/robot.png', large_icon=r'dearpy-ros/Assets/icons/robot.ico')
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
