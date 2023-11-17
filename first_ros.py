import rospy
from std_msgs.msg import String

import  math
import dearpygui.dearpygui as dpg



sindatax = []
sindatay = []
for i in range(0, 100):
    sindatax.append(i / 100)
    sindatay.append(0.5 + 0.5 * math.sin(50 * i / 100))
sindatay2 = []
for i in range(0, 100):
    sindatay2.append(2 + 0.5 * math.sin(50 * i / 100))


dpg.create_context()

def button_callback(sender, app_data):
    msg = "Hello_World"
    print(msg)
    dp_pub.publish(data=msg)


def update_series():

    cosdatax = []
    cosdatay = []
    for i in range(0, 500):
        cosdatax.append(i / 1000)
        cosdatay.append(0.5 + 0.5 * math.cos(50 * i / 1000))
    dpg.set_value('series_tag', [cosdatax, cosdatay])
    dpg.set_item_label('series_tag', "0.5 + 0.5 * cos(x)")

rospy.init_node("Dearpy_Node")
dp_pub = rospy.Publisher("dearpy_hello", String, queue_size=2)

# add a font registry
with dpg.font_registry():
    # first argument ids the path to the .ttf or .otf file
    default_font = dpg.add_font("dearpy-ros/Assets/Fonts/Retron2000.ttf", 30)


with dpg.window(label="Hello Publisher", height=400, width=600, pos=(0,0)):
    dpg.add_button(label="Print to Terminal", callback=button_callback)

    # set font of specific widget
    dpg.bind_font(default_font)

    with dpg.window(label="Tutorial", width=800, height=600, pos=(600,0)):
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

        # create plot
        with dpg.plot(tag="plot", label="Line Series", height=-1, width=-1):

            # optionally create legend
            dpg.add_plot_legend()

            # REQUIRED: create x and y axes
            dpg.add_plot_axis(dpg.mvXAxis, label="x")
            dpg.add_plot_axis(dpg.mvYAxis, label="y", tag="yaxis")

            # series belong to a y axis
            dpg.add_stem_series(sindatax, sindatay, label="0.5 + 0.5 * sin(x)", parent="yaxis", tag="series_data")
            dpg.add_scatter_series(sindatax, sindatay2, label="2 + 0.5 * sin(x)", parent="yaxis", tag="series_data2")

            # apply theme to series
            dpg.bind_item_theme("series_data", "plot_theme")
            dpg.bind_item_theme("series_data2", "plot_theme")






dpg.create_viewport(title='ROS App', width=1800, height=1600, small_icon=r'dearpy-ros/Assets/icons/robot.png', large_icon=r'dearpy-ros/Assets/icons/robot.ico')
dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()
