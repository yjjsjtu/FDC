#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

from nicegui import ui, app
from motorcontroller import MotorController
from queue import Queue
import threading
import time
import yaml 
import json
from datetime import datetime


is_recording = False
recorded_data = []
recording_thread = None
recording_file = "motor_positions.json"


def start_damping():
    enable_all_motors()
    for motor in controller.motors.values():
        motor.set_damping_mode()
        # motor.reference_status()
    ui.notify("All motors set to damping mode")

def stop_damping():
    disable_all_motors()
    # enable_all_motors()
    for motor in controller.motors.values():
        motor.set_stop_damping_mode()
        # motor.reference_status()
    ui.notify("All motors disabled damping mode")

# 在函数部分添加以下三个新函数
def start_recording():
    global is_recording, recorded_data, recording_thread
    
    if is_recording:
        ui.notify("Recording is already in progress!")
        return
    
    is_recording = True
    recorded_data = []
    ui.notify("Recording started")
    
    def record_positions():
        while is_recording:
            positions = {}
            status_data = []
            for status in controller.get_all_motor_status():
                status_data.append({
                    'node_id': status['node_id'],
                    'position': status['position']
                })
            
            # 添加时间戳
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            recorded_data.append({
                'timestamp': timestamp,
                'positions': status_data
            })
            
            time.sleep(0.5)  # 每0.5秒记录一次
    
    recording_thread = threading.Thread(target=record_positions, daemon=True)
    recording_thread.start()

def stop_recording():
    global is_recording
    
    if not is_recording:
        ui.notify("No active recording to stop!")
        return
    
    is_recording = False
    ui.notify("Recording stopped")
    
    try:
        with open(recording_file, 'w') as f:
            json.dump(recorded_data, f, indent=4)
        ui.notify(f"Data saved to {recording_file}")
    except Exception as e:
        ui.notify(f"Error saving data: {str(e)}")

def replay_recording():
    if is_recording:
        ui.notify("Cannot replay while recording!")
        return
    
    try:
        with open(recording_file, 'r') as f:
            data = json.load(f)
    except Exception as e:
        ui.notify(f"Error loading recording: {str(e)}")
        return
    
    if not data:
        ui.notify("No recording data to replay!")
        return
    
    ui.notify(f"Replaying {len(data)} position records")
    
    def replay_task():
        for record in data:
            positions = record['positions']
            for pos in positions:
                motor_id = pos['node_id']
                position = pos['position']
                if motor_id in controller.motors:
                    controller.motors[motor_id].set_position(position)
            time.sleep(0.5)  # 保持与记录时相同的时间间隔
    
    threading.Thread(target=replay_task, daemon=True).start()

# Create the motor controller
controller = MotorController()

with open('motors.yaml', 'r') as file:
    motor_config = yaml.safe_load(file)

# Initialize motors based on YAML config
for node in motor_config['nodes']:
    node_id = node['id']
    reduction = node['reduction']
    try:
        controller.add_motor(node_id, reduction=reduction)
    except Exception as e:
        print(f"Error initializing motor {node_id}: {e}")
        
# Start the CAN notifier if bus is available
if controller.is_initialized():
    controller.start()

message_queue = Queue()

# def calibration():
#     i = 0
#     for motor in controller.motors.values():
#         i += 1
#         if i == 2:
#             print("Calibrating motor", motor.node_id)
#             # motor.start_calibration()

def process_messages():
    while not message_queue.empty():
        message = message_queue.get()
        ui.notify(message)
# Functions for button actions
def enable_all_motors():
    for motor in controller.motors.values():
        motor.enable()
        # motor.reference_status()
    message_queue.put("All motors enabled")
    # get_all_motors_status()

homing_event = threading.Event()

def restore_positions():
     for motor in controller.motors.values():
        pos = motor.saved_position
        motor.set_position(pos)

def manual_homing_all_motors():
    if homing_event.is_set():
        ui.notify("Manual homing is already in progress!")
        return
    
    def _manual_homing_task():
        homing_event.set()
        try:
            message_queue.put("Manual homing is on the progress!")
            disable_all_motors()
            time.sleep(1)
            i = 0
            for motor in controller.motors.values():
                i += 1
                if i == 2:
                    motor.set_home(80)
                elif i == 3:
                    motor.set_home(92)
                elif i == 5:
                    motor.set_home(105)
                else:
                    motor.set_home(0)
                time.sleep(0.5)
            enable_all_motors()
            time.sleep(3)
            # i = 0
            for motor in controller.motors.values():
                # i += 1
                motor.set_position(0.0)
            time.sleep(3)
            message_queue.put("All motors have set to home position!!!")
        finally:
            homing_event.clear()

    threading.Thread(target=_manual_homing_task, daemon=True).start()


#disabled function
def homing_all_motors():

    if homing_event.is_set():
        ui.notify("Homing is already in progress!")
        return
    def _homing_task():
        homing_event.set()
        try:
            i = 0
            msg = "Homing started"
            for motor in controller.motors.values():
                i += 1
                try:
                    message_queue.put(f"homing motor {i}")
                    if i == 1:
                        msg = motor.set_homing(3.0, -360, 180, 30, 0.5)
                        if msg != "success":
                            break
                    elif i == 2:
                        msg = motor.set_homing(5.0, -360, 81, 30, 0.5)
                        if msg != "success":
                            break
                    elif i == 3:
                        msg = motor.set_homing(5.0, 360, -95, 30, 0.5)
                        if msg != "success":
                            break
                    elif i == 5:
                        msg = motor.set_homing(3.0, -360, 115, 30, 0.5)
                        if msg != "success":
                            break
                    else:
                        time.sleep(1)
                        motor.disable()
                        time.sleep(1)
                        motor.set_home(0)
                        time.sleep(1)
                        motor.enable()
                        time.sleep(1)
                except Exception as e:
                    msg = f"Error in motor {i}: {str(e)}"
                    break

            print(f"homing status: {msg}")
            message_queue.put(f"homing status: {msg}")
        finally:
            homing_event.clear()

    threading.Thread(target=_homing_task, daemon=True).start()

def get_all_motors_status():
    for motor in controller.motors.values():
        motor.reference_status()
        motor.reference_saved_position()
        motor.reference_value1()
    # ui.notify("All motors enabled")

def disable_all_motors():
    for motor in controller.motors.values():
        motor.disable()
    message_queue.put("All motors disabled")

def clear_up_errors():
    for motor in controller.motors.values():
        motor.error_resets()
    ui.notify("All motors error resets")
def move_motors(position: float):
    for motor in controller.motors.values():
        motor.set_position(position)
    ui.notify(f"All motors moving to position {position}")

def move_motors_range(position: float):
    i = 0
    for motor in controller.motors.values():
        i += 1
        if i == 2:
            motor.set_position(-90)
        elif i == 3:
            motor.set_position(-90)
        elif i == 5:
            motor.set_position(0)
        else:
            motor.set_position(0)
    ui.notify(f"All motors moving to position {position}")

def move_motors_ready(position: float):
    i = 0
    # print("move_motors_ready", position)
    # return
    for motor in controller.motors.values():
        i += 1
        if i == 2:
            motor.set_position(79)
        elif i == 3:
            motor.set_position(89)
        elif i == 5:
            motor.set_position(105)
        else:
            motor.set_position(0)
    ui.notify(f"All motors moving to long range {position}")

def loop_motors(position: float):
    if homing_event.is_set():
        ui.notify("Looping is already in progress!")
        return
    def _loop_task():
        homing_event.set()
        try:
            for _ in range(10):    
                for motor in controller.motors.values():
                    motor.set_position(position * -1)
                time.sleep(7)
                for motor in controller.motors.values():
                    motor.set_position(position)
                time.sleep(7)
        finally:
            homing_event.clear()
    threading.Thread(target=_loop_task, daemon=True).start()
    ui.notify(f"All motors moving to position {position}")

def move_motors_home(position: float):
    # i = 0
    # for motor in controller.motors.values():
    #     i += 1
    #     if i == 2:
    #         motor.set_position(80)
    #     elif i == 3:
    #         motor.set_position(90)
    #     elif i == 5:
    #         motor.set_position(0)
    #     else:
    #         motor.set_position(0)
    for motor in controller.motors.values():
        motor.set_position(position * 0)
    # time.sleep(5)
    # for motor in controller.motors.values():
    #     # time.sleep(1)
    #     motor.disable()
    # time.sleep(1)
    # for motor in controller.motors.values():
    #     # time.sleep(1)
    #     motor.disable()
    # time.sleep(1)
    # for motor in controller.motors.values():
    #     # time.sleep(1)
    #     motor.set_home()
    # time.sleep(1)
    # for motor in controller.motors.values():
    #     motor.enable()
    # time.sleep(1)
    ui.notify(f"All motors moving to home position {position}")

calibration_progress = 0  # 0~100 百分比
calibration_in_progress = False

# 修改 calibration 函数
def calibration():
    global calibration_progress, calibration_in_progress
    
    if calibration_in_progress:
        ui.notify("Calibration is already in progress!")
        return
    
    calibration_in_progress = True
    calibration_progress = 0
    progress_bar.visible = True
    
    def _calibration_task():
        global calibration_progress
        global calibration_in_progress
        try:
            i = 0
            for motor in controller.motors.values():
                i += 1
                if i == 2:
                    motor.start_calibration()
                    while motor.calibration_progress < 137:
                        time.sleep(0.1)
                        calibration_progress = motor.calibration_progress
            time.sleep(1)
            # message_queue.put("Calibration completed!")
            # calibration_in_progress = False
            # progress_bar.style.update({'background-color': 'green'})
            # ui.notify("Calibration completed!")
        finally:
            message_queue.put("Calibration completed final!")
            calibration_in_progress = False
            progress_bar.visible = False
    
    threading.Thread(target=_calibration_task, daemon=True).start()

def move_motors_neg(position: float):
    for motor in controller.motors.values():
        motor.set_position(position * -1)
    ui.notify(f"All motors moving to position {position}")

# Function to update status display
def update_status_display():
    process_messages()
    get_all_motors_status()
    # print("Updating status display")
    status_data = []
    for status in controller.get_all_motor_status():
        errors = [k for k, v in status['errors'].items() if v]
        # calibration_progress =  status['calibration_progress']
        if calibration_in_progress:
            progress_bar.value = calibration_progress / 100  # 转换为 0~1 范围

        status_data.append({
            'node_id': status['node_id'],
            'enabled': 'Yes' if status['enabled'] else 'No',
            'raw_status': str(status['raw_status']),
            'raw_error': str(status['raw_error']),
            'target_reached': 'Yes' if status['target_reached'] else 'No',
            'errors': ', '.join(errors) if errors else 'None',
            'last_update': f"{status['last_update']:.1f}s ago",
            'position':  f"{status['position']:.3f}",
            'saved_position':  f"{status['saved_position']:.3f}",
            'velocity':  f"{status['velocity']:.3f}",
            'torque':  f"{status['torque']:.3f}",
            'current':  f"{status['current']:.3f}",
            'bus_current':  f"{status['bus_current']:.3f}",
            'bus_voltage':  f"{status['bus_voltage']:.3f}",
            # 'motor_current':  f"{status['motor_current']:.3f}",
            'motor_power':  f"{status['motor_power']:.3f}",
            'mosfet_temp':  f"{status['mosfet_temp']:.1f}",
            # 'calibration_progress':  f"{status['calibration_progress']:.3f}'"
        })
    status_table.rows = status_data
    status_table.update()

# NiceGUI App
app.title = "Motor Control Panel"

def show_confirm_dialog():
    with ui.dialog() as dialog:  
        with ui.card():
            ui.label("Are you sure you want to start Motor[1] Calibration?")
            with ui.row():
                ui.button("Yes", on_click=lambda: (calibration(), dialog.close())) 
                ui.button("No", on_click=dialog.close)  

    dialog.open() 

# Create menu structure
with ui.row().classes('w-full'):
    with ui.tabs() as tabs:
        menu1 = ui.tab('Motor Control')
        menu2 = ui.tab('Settings')

# Create content for each menu
with ui.tab_panels(tabs, value=menu1).classes('w-full'):
    # Menu 1 content - Original motor control panel
    with ui.tab_panel(menu1):
        with ui.card().classes('w-full'):
            ui.label('Motor Control Panel').classes('text-h4')
            
            # Status display
            columns = [
                {'name': 'node_id', 'label': 'Motor ID', 'field': 'node_id'},
                {'name': 'enabled', 'label': 'Enabled', 'field': 'enabled'},
                {'name': 'raw_status', 'label': 'Raw Status', 'field': 'raw_status'},
                {'name': 'raw_error', 'label': 'Raw Error', 'field': 'raw_error'},
                # {'name': 'target_reached', 'label': 'Target Reached', 'field': 'target_reached'},
                {'name': 'errors', 'label': 'Errors', 'field': 'errors'},
                {'name': 'last_update', 'label': 'Last Update', 'field': 'last_update'},
                {'name': 'position', 'label': 'Position', 'field': 'position'},
                # {'name': 'saved_position', 'label': 'Saved Position', 'field': 'saved_position'},
                {'name': 'velocity', 'label': 'Velocity', 'field': 'velocity'},
                {'name': 'torque', 'label': 'Torque', 'field': 'torque'},
                {'name': 'current', 'label': 'Current', 'field': 'current'},
                {'name': 'bus_current', 'label': 'Bus Current', 'field': 'bus_current'},
                {'name': 'bus_voltage', 'label': 'Bus Voltage', 'field': 'bus_voltage'},
                # {'name': 'motor_current', 'label': 'Motor Current', 'field': 'motor_current'},
                {'name': 'motor_power', 'label': 'Motor Power', 'field': 'motor_power'},
                {'name': 'mosfet_temp', 'label': 'Mosfet Temp', 'field': 'mosfet_temp'}
            ]
            status_table = ui.table(columns=columns, rows=[]).classes('w-full')
            
            # Position control
            with ui.row().classes('w-full items-center'):
                position_input = ui.number(label='Position (degrees)', value=0.0).classes('w-32')
                ui.button('+', on_click=lambda: move_motors(position_input.value))
                ui.button('Home', on_click=lambda: move_motors_home(position_input.value))
                ui.button('-', on_click=lambda: move_motors_neg(position_input.value))
                ui.button('Max. Range', on_click=lambda: move_motors_range(position_input.value))
                ui.button('Ready', on_click=lambda: move_motors_ready(position_input.value))
                ui.button('Loop', on_click=lambda: loop_motors(position_input.value))
            
            ui.separator().classes('flex-grow')  
            ui.label('Control').classes('text-sm text-gray-500') 
            # Motor control buttons
            with ui.row().classes('w-full items-center'):
                ui.button('Enable All Motors', on_click=enable_all_motors, color='green')
                ui.button('Disable All Motors', on_click=disable_all_motors, color='red')
                ui.button('Reset Errors', on_click=clear_up_errors, color='blue')
                # ui.button('RESTORE POSITIONS', on_click=restore_positions, color='blue')

            ui.separator().classes('flex-grow')  
            ui.label('ARM Calibration').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('HOMING', on_click=manual_homing_all_motors, color='green')
                # ui.button('AUTO HOMING', on_click=homing_all_motors, color='green')
            ui.separator().classes('flex-grow')  
            ui.label('Teaching').classes('text-sm text-gray-500') 
            with ui.row().classes('w-full items-center'):
                ui.button('Record', on_click=start_recording, color='orange')
                ui.button('Stop', on_click=stop_recording, color='red')
                ui.button('Replay', on_click=replay_recording, color='blue')
            ui.separator().classes('flex-grow')  
            # ui.label('Motor Calibration').classes('text-sm text-gray-500') 
            # with ui.row().classes('w-full items-center'):
            #     ui.button('Calibration', on_click=show_confirm_dialog, color='blue')
            #     progress_bar = ui.linear_progress(value=0, color='green').classes('w-64')  # value 范围 0~1 或 0~100
            #     progress_bar.visible = False  # 默认隐藏
        # Menu 2 content - Empty for now
    with ui.tab_panel(menu2):
        with ui.card().classes('w-full'):
            ui.label('Settings').classes('text-h4')
            # ui.markdown('This panel is reserved for future functionality.')

            # Individual motor controls
            # with ui.expansion('Individual Motor Controls').classes('w-full'):
            with ui.tabs().classes('w-full') as motor_tabs:
                for node_id in range(1, 7):
                    ui.tab(f'Motor {node_id}')

            with ui.tab_panels(motor_tabs, value='MOTOR').classes('w-full'):
                for node_id in range(1, 7):
                    with ui.tab_panel(f'Motor {node_id}'):
                        with ui.card().classes('w-full'):
                            ui.label(f'Motor {node_id}').classes('text-h6')
                            pos_input = ui.number(label=f'Position', value=0.0).classes('w-32')
                            ui.button(f'Move', on_click=lambda _, n=node_id, p=pos_input: controller.motors[n].set_position(p.value))
                            with ui.expansion('Motor Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    current_limit = ui.number(label=f'Current Limit', value=3).bind_value(controller.motors[node_id].configs.current_limit, 'current_limit')
                            with ui.expansion('Controller Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    config_node_id = ui.number(label=f'Node Id', value=3).bind_value(controller.motors[node_id].configs.nodeid, 'node_id')
                                    config_mode = ui.number(label=f'Control Mode', value=3).tooltip("3 = profiled position mode").bind_value(controller.motors[node_id].configs.control_mode, 'control_mode')
                                    velocity = ui.number(label=f'Velocity', value=3.0).tooltip("normal=3 middle=7 fast=15 crazzy=30").bind_value(controller.motors[node_id].configs.velocity, 'velocity')
                                    acceleration = ui.number(label=f'Acceleration', value=5.0).tooltip("normal=4.8 middle=11 fast=24 crazzy=50").bind_value(controller.motors[node_id].configs.acceleration, 'acceleration')
                                    deceleration = ui.number(label=f'Deceleration', value=5.0).tooltip("normal=4.8 middle=11 fast=24 crazzy=50").bind_value(controller.motors[node_id].configs.deceleration, 'deceleration')
                                    ui.separator().classes('flex-grow')  
                                    config_kp_gain = ui.number(label=f'KP Gain', value=60.0).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.kp_gain, 'kp_gain')
                                    config_kd_gain = ui.number(label=f'KD Gain', value=0.003, format='%.6f', step=0.0001).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.kd_gain, 'kd_gain')
                                    config_ki_gain = ui.number(label=f'KI Gain', value=0.0001, format='%.6f', step=0.000001).tooltip("value: 0 - 1000").bind_value(controller.motors[node_id].configs.ki_gain, 'ki_gain')
                            with ui.expansion('Security Parameters', icon='menu').classes('w-full'):
                                with ui.row().classes('w-full items-center'):
                                    over_current = ui.number(label=f'Motor Over current', value=4).bind_value(controller.motors[node_id].configs.protect_over_current, 'protect_over_current')
                            ui.separator().classes('flex-grow')  
                            ui.label('Parameters').classes('text-sm text-gray-500') 
                            with ui.row().classes('w-full items-center'):
                                with ui.button(f'Retrieve', on_click=lambda _, n=node_id: controller.motors[n].get_config()):
                                    ui.tooltip('Gets the current configs')
                                with ui.button(f'Apply', on_click=lambda _, n=node_id, p=[current_limit, config_mode, velocity, acceleration, deceleration, over_current, config_kp_gain, config_kd_gain, config_ki_gain]: controller.motors[n].set_config(p)):
                                    ui.tooltip('Applies the current configs')
                                with ui.button(f'Save', on_click=lambda _, n=node_id: controller.motors[n].save_configs(), color='red'):
                                    ui.tooltip('Saves the current configs to Motors MCU')
                            ui.separator().classes('flex-grow')  
                            ui.label('Global Actions').classes('text-sm text-gray-500') 
                            with ui.row().classes('w-full items-center'):
                                with ui.button('ALL TO DEFAULTS', on_click=lambda _, n=node_id: controller.motors[n].reset_all_to_defaults(),  color='red'):
                                    ui.tooltip('Reset ALL motor parameters to default values')
                            ui.separator().classes('flex-grow')  
                            ui.label('Controls').classes('text-sm text-gray-500') 
                            with ui.row().classes('w-full items-center'):
                                with ui.button(f'Home', on_click=lambda _, n=node_id: controller.motors[n].set_home()):
                                    ui.tooltip('Set current position as home position')
                                with ui.button(f'Normal', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([5, 10, 10]) , color='green'):
                                    ui.tooltip('Robot arm speed in normal mode')
                                with ui.button(f'Fast', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([10, 20, 20]) , color='blue'):
                                    ui.tooltip('Robot arm speed in fast mode')
                                with ui.button(f'Sport', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([30, 60, 60]) , color='blue'):
                                    ui.tooltip('Robot arm speed in sport mode')
                                with ui.button(f'Crazzy', on_click=lambda _, n=node_id: controller.motors[n].set_speed_mode([50, 100, 100]), color='red'):
                                    ui.tooltip('Robot arm speed in crazzy mode')
# Timer to update status
ui.timer(0.5, update_status_display)

# On shutdown
app.on_shutdown(controller.stop)

ui.run()

