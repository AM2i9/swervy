# GUI configuration tool for swervy
# Author: Patrick Brennan (AM2i9)

from collections import deque
import re
import time
import serial
import serial.tools
from threading import Thread, Lock
import dearpygui.dearpygui as dpg
import dearpygui.demo as demo
import serial.tools.list_ports

MAX_DEQUE = 600

tlock = Lock()
init_time = time.time()

dpg.create_context()
dpg.create_viewport(title='swervy - Config Tool', width=1340, height=1000, resizable=False, clear_color=(25,25,25))
dpg.set_global_font_scale(1)

serial_devs = serial.tools.list_ports.comports(include_links=True)

ser = serial.Serial()
ser.baudrate = 115200

published_vals = {}
graphed = []

def _log_msg(msg: str):
    # determine ANSI color usage
    colors = re.findall("\033\[(\d+)m", msg)
    msg = re.sub("\033\[(\d+)m", "", msg)
    color = (-255, 0, 0, 255)
    if "32" in colors:
        # Info
        color = (0, 255, 0)
    elif "31" in colors:
        # Error
        color = (255, 0, 0)
    elif "33" in colors:
        # Warn
        color = (255, 199, 6)
    dpg.add_text(msg, parent=message_log_window, color=color)

def _log_var(var_name: str, value_str: str):
    try:
        if var_name not in published_vals:
            if value_str.strip().replace(".", "", 1).isdecimal():
                published_vals[var_name] = {"type": float, "value": float(value_str), "history": deque(maxlen=MAX_DEQUE)}
                published_vals[var_name]["history"].append((time.time()-init_time, published_vals[var_name]["value"]))

                with dpg.table_row(parent=pub_val_table):
                    dpg.add_text(var_name, drag_callback=lambda: print("dragging"))
                    dpg.add_text(value_str, tag=f"pub_val_{var_name}", drag_callback=lambda: print("dragging"))
                    def _plot_button_callback(sender):
                        _plot_var(var_name)
                        dpg.disable_item(sender)
                    dpg.add_button(tag=f"pub_val_button_{var_name}", label="->", callback=_plot_button_callback)

        else:
            published_vals[var_name]["value"] = published_vals[var_name]["type"](value_str)
            if "history" in published_vals[var_name]:
                published_vals[var_name]["history"].append((time.time()-init_time, published_vals[var_name]["value"]))
            dpg.set_value(f"pub_val_{var_name}", str(published_vals[var_name]["value"]))
    except ValueError:
        # for some reason i keep recieving serial messages that combined two messages
        # I spent 2 hours debugging this and I have no idea why it happens
        # we're just gonna ignore it because, as Linkin Park once said, "In the end, it doesn't even matter"
        pass

def _plot_var(var_name):
    data = published_vals[var_name]["history"]
    x, y = zip(*list(data))
    dpg.add_line_series(
        parent=yaxes_id,
        x=x,
        y=y,
        label=var_name,
        tag=f"pub_val_graph_{var_name}"
    )
    dpg.fit_axis_data(xaxes_id)
    dpg.fit_axis_data(yaxes_id)
    graphed.append(var_name)

def _update_plot():
    while True:
        if ser is None:
            break
        for v in graphed:
            data = published_vals[v]["history"]
            x, y = zip(*list(data))
            dpg.configure_item(f"pub_val_graph_{v}", x=x, y=y)
        time.sleep(0.01)

        dpg.fit_axis_data(xaxes_id)

def _clear_msg_log():
    global msg_log
    dpg.delete_item(message_log_window, children_only=True)

def _refresh_devs():
    global serial_devs
    serial_devs = serial.tools.list_ports.comports()

def _set_status(status):
    if status == 0:
        dpg.set_value(port_status, "Not Connected")
        dpg.configure_item(port_status, color=(255,0,0))
    elif status == 1:
        dpg.set_value(port_status, "Connected")
        dpg.configure_item(port_status, color=(0,255,0))

def _connect_dev():
    device = dpg.get_value(serial_port_select)
    device = device.split(" - ")[0].strip()
    if not ser.is_open and device != "":
        _log_msg(f"Connecting to device on {device}...")

        ser.port = device
        try:
            ser.open()
        except serial.SerialException as e:
            _log_msg(str(e))
            ser.close()
            _set_status(0)
        else:
            _set_status(1)

def _close_dev():
    ser.close()
    _set_status(0)

def _dev_log_thread():
    while True:
        if ser is None:
            break
        if ser.is_open:
            buf = ser.readline()
            msg = buf.decode()
            if msg.startswith("VAR!:"):
                x = msg.split(":")
                # VAR!:var_name:value
                _log_var(x[1], x[2])
            elif msg != "":
                _log_msg(msg)
        time.sleep(0.001)

# Serial Info
with dpg.window(label="Serial", width=450, pos=(10,10)) as serial_win:

    with dpg.group(horizontal=True):
        dpg.add_text("Status:")
        port_status = dpg.add_text("Not Connected", color=(255,0,0))

    with dpg.group(horizontal=True):
        dpg.add_text("Device:")
        serial_port_select = dpg.add_combo(items=serial_devs, width=300)
        dpg.add_button(label="Refresh", callback=_refresh_devs)
    
    with dpg.group(horizontal=True):
        dpg.add_button(label="Connect", callback=_connect_dev)
        dpg.add_button(label="Close", callback=_close_dev)

# Message Log
with dpg.window(label="Message Log", width=450, height=300, pos=(10,130)) as serial_win:
    dpg.add_button(label="Clear", callback=_clear_msg_log)

    message_log_window = dpg.add_child_window()
    
    with dpg.theme() as item_theme:
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 4, 1, category=dpg.mvThemeCat_Core)
    
    dpg.bind_item_theme(message_log_window, item_theme)

# Published Values
with dpg.window(label="Published Values", width=450, height=420, pos=(470,10)) as pub_win:
    pub_val_table = dpg.add_table()
    dpg.add_table_column(parent=pub_val_table,label="Name")
    dpg.add_table_column(parent=pub_val_table,label="Value")
    dpg.add_table_column(parent=pub_val_table, width_fixed=True, width=25)

plot_id = dpg.generate_uuid()
xaxes_id = dpg.generate_uuid()
yaxes_id = dpg.generate_uuid()

# Graph values
with dpg.window(label="Graph", height=550, width=1000, pos=(10, 440)):
    with dpg.plot(label="Line Series", width=-1, height=-25, tag=plot_id):
        dpg.add_plot_legend()

        dpg.add_plot_axis(dpg.mvXAxis, label="Time (s)", tag=xaxes_id)
        dpg.add_plot_axis(dpg.mvYAxis, label="Value", tag=yaxes_id)

# colors :)
with dpg.theme() as global_theme:

    with dpg.theme_component(dpg.mvAll):
        dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 10, category=dpg.mvThemeCat_Core)
        dpg.add_theme_style(dpg.mvStyleVar_WindowRounding, 10, category=dpg.mvThemeCat_Core)
        dpg.add_theme_style(dpg.mvStyleVar_ChildRounding, 10, category=dpg.mvThemeCat_Core)
        dpg.add_theme_style(dpg.mvStyleVar_PopupRounding, 10, category=dpg.mvThemeCat_Core)

        dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 4, category=dpg.mvThemeCat_Core)
        
        dpg.add_theme_color(dpg.mvThemeCol_Button, (154,17,17), category=dpg.mvThemeCat_Core)
        dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (255,100,100, 153), category=dpg.mvThemeCat_Core)
        dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (255,0,0, 103), category=dpg.mvThemeCat_Core)

        dpg.add_theme_color(dpg.mvThemeCol_TitleBgActive, (154,17,17), category=dpg.mvThemeCat_Core)

        dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (80,80,80, 103), category=dpg.mvThemeCat_Core)

        dpg.add_theme_color(dpg.mvThemeCol_HeaderHovered, (236, 29, 29, 103), category=dpg.mvThemeCat_Core)
        dpg.add_theme_color(dpg.mvThemeCol_HeaderActive, (200, 0, 0, 153), category=dpg.mvThemeCat_Core)
    
    with dpg.theme_component(dpg.mvButton, enabled_state=False):
            dpg.add_theme_color(dpg.mvThemeCol_Text, (180, 180, 180), category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_Button, (35,8,8), category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (35,8,8), category=dpg.mvThemeCat_Core)
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (35,8,8), category=dpg.mvThemeCat_Core)

dpg.bind_theme(global_theme)

serial_recv_thread = Thread(target=_dev_log_thread)
graph_update_thread = Thread(target=_update_plot)

serial_recv_thread.start()
graph_update_thread.start()


# demo.show_demo()

# dpg.show_style_editor()

# _log_var("test_var", "12345")

dpg.setup_dearpygui()
dpg.show_viewport()
dpg.start_dearpygui()
dpg.destroy_context()

ser = None

serial_recv_thread.join()
graph_update_thread.join()
