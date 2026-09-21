import re

file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    orig_content = f.read()

# Define the pattern to replace
start_marker = "            with ui.element('div').style('flex: 2; min-width: 400px; display: flex; flex-direction: column;'):"
end_marker = "                            'box-shadow: 0 0 10px rgba(139,92,246,0.3);')\n"

start_idx = orig_content.find(start_marker)
end_idx = orig_content.find(end_marker, start_idx) + len(end_marker)

replacement = """            with ui.element('div').style('flex: 2; min-width: 500px; display: flex; flex-direction: column; gap: 24px;'):

                # Enable / Disable row & Status
                with ui.element('div').style('display: flex; gap: 16px; align-items: center; flex-wrap: wrap;'):
                    ui.button('ENABLE', on_click=lambda: on_gripper_enable()).style(
                        'padding: 8px 24px; border: none; border-radius: 10px; '
                        'font-weight: 700; cursor: pointer; letter-spacing: 0.05em; '
                        'background: linear-gradient(135deg, #f97316, #ea580c); color: #fff; '
                        'box-shadow: 0 0 12px rgba(249,115,22,0.3);')
                    ui.button('DISABLE', on_click=lambda: on_gripper_disable()).style(
                        'padding: 8px 24px; border: none; border-radius: 10px; '
                        'font-weight: 700; cursor: pointer; letter-spacing: 0.05em; '
                        'background: linear-gradient(135deg, #475569, #334155); color: #fff; '
                        'box-shadow: 0 0 12px rgba(0,0,0,0.3);')

                    ui.element('div').style('width: 2px; height: 24px; background: rgba(255,255,255,0.1); margin: 0 8px;')

                    # Gripper status label
                    ui.html('<span style="font-size: 0.85rem; color: var(--text-dim); font-weight: 600;">Status:</span>')
                    _grip_status = ui.label('Idle').style(
                        'font-size: 0.95rem; color: var(--text-primary); font-weight: 700; background: rgba(249,115,22,0.15); padding: 4px 12px; border-radius: 6px; border: 1px solid rgba(249,115,22,0.3);')
                    ui_labels['gripper_status'] = _grip_status

                # Sliders grid: 2 columns
                with ui.element('div').style(
                        'display: grid; grid-template-columns: repeat(2, 1fr); '
                        'gap: 30px; background: rgba(0,0,0,0.15); padding: 20px 24px 24px; border-radius: 12px; border: 1px solid rgba(255,255,255,0.05);'):

                    # Position slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: var(--accent-cyan); letter-spacing: 0.04em;">Position (rev)</span>')
                            _grip_pos_lbl = ui.label('0.00 rev (0.0°)').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_pos_label'] = _grip_pos_lbl
                        _grip_pos = ui.slider(min=0, max=1.3, step=0.1, value=0).props('label-always color="cyan"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_pos_slider'] = _grip_pos

                    # Torque slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #f97316; letter-spacing: 0.04em;">Torque (Nm)</span>')
                            _grip_torq_lbl = ui.label('0.30 Nm').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_torq_label'] = _grip_torq_lbl
                        _grip_torq = ui.slider(min=0, max=1, step=0.01, value=0.3).props('label-always color="orange"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_torq_slider'] = _grip_torq

                    # Velocity slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #a78bfa; letter-spacing: 0.04em;">Velocity (rev/s)</span>')
                            _grip_vel_lbl = ui.label('1.00 rev/s').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_vel_label'] = _grip_vel_lbl
                        _grip_vel = ui.slider(min=0, max=5, step=0.1, value=1.0).props('label-always color="purple"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_vel_slider'] = _grip_vel

                    # Accel slider
                    with ui.element('div').style('display: flex; flex-direction: column;'):
                        with ui.element('div').style('display: flex; justify-content: space-between; align-items: baseline;'):
                            ui.html('<span style="font-size: 0.85rem; font-weight: 700; color: #fbbf24; letter-spacing: 0.04em;">Accel (rev/s²)</span>')
                            _grip_acc_lbl = ui.label('1.00 rev/s²').style(
                                'font-size: 0.82rem; color: var(--text-primary); font-weight: 600;')
                            ui_labels['gripper_acc_label'] = _grip_acc_lbl
                        _grip_acc = ui.slider(min=0, max=5, step=0.1, value=1.0).props('label-always color="amber"').style('width: 100%; margin-top: 28px;')
                        ui_labels['gripper_acc_slider'] = _grip_acc

                # Move button row
                with ui.element('div').style('display: flex; gap: 24px; align-items: flex-end; flex-wrap: wrap;'):
                    
                    # Actions
                    with ui.element('div').style('display: flex; gap: 12px;'):
                        ui.button('\U0001f4cd Move Gripper', on_click=lambda: on_gripper_move()).style(
                            'padding: 8px 28px; border: none; border-radius: 10px; '
                            'font-weight: 700; cursor: pointer; '
                            'background: linear-gradient(135deg, #f97316, #fbbf24); color: #1a1a2e; '
                            'box-shadow: 0 0 16px rgba(249,115,22,0.4); font-size: 0.95rem;')
                        ui.button('\U0001f3e0 Home (0)', on_click=lambda: on_gripper_home()).style(
                            'padding: 8px 24px; border: none; border-radius: 10px; '
                            'font-weight: 700; cursor: pointer; '
                            'background: rgba(249,115,22,0.15); color: #f97316; '
                            'border: 1px solid rgba(249,115,22,0.35);')

                    ui.element('div').style('width: 2px; height: 32px; background: rgba(255,255,255,0.1);')

                    # Position Limits config
                    with ui.element('div').style('display: flex; align-items: center; gap: 12px; background: rgba(0,0,0,0.15); padding: 8px 20px; border-radius: 10px; border: 1px solid rgba(255,255,255,0.05);'):
                        ui.html('<span style="font-size: 0.85rem; font-weight: 600; color: var(--accent-cyan);">Current (°):</span>')
                        _grip_angle_input = ui.number(
                            value=0.0, format='%.2f', step=0.1
                        ).style('width: 80px; font-weight: 600; text-align: center; font-size: 0.9rem;').props('dense dark filled')
                        ui_labels['gripper_angle_input'] = _grip_angle_input

                        ui.button('SET MIN', on_click=lambda: on_gripper_set_min_pos()).style(
                            'padding: 4px 12px; border-radius: 6px; font-weight: 700; cursor: pointer; font-size: 0.75rem; '
                            'background: #3b82f6; color: #fff; box-shadow: 0 0 8px rgba(59,130,246,0.3);')
                        
                        ui.button('SET MAX', on_click=lambda: on_gripper_set_max_pos()).style(
                            'padding: 4px 12px; border-radius: 6px; font-weight: 700; cursor: pointer; font-size: 0.75rem; '
                            'background: #06b6d4; color: #fff; box-shadow: 0 0 8px rgba(6,182,212,0.3);')
"""
new_content = orig_content[:start_idx] + replacement + orig_content[end_idx:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(new_content)

print("done")
