import re

file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    text = f.read()

# 1. Slider UI HTML label
old_slider_lbl = "ui.html('<span style=\"font-size: 0.85rem; font-weight: 700; color: var(--accent-cyan); letter-spacing: 0.04em;\">Position (rev)</span>')"
new_slider_lbl = "ui.html('<span style=\"font-size: 0.85rem; font-weight: 700; color: var(--accent-cyan); letter-spacing: 0.04em;\">Position (mm)</span>')"
text = text.replace(old_slider_lbl, new_slider_lbl)

# 2. Slider UI value format
old_slider_ui_lbl = "_grip_pos_lbl = ui.label('0.00 rev (0.0°)').style("
new_slider_ui_lbl = "_grip_pos_lbl = ui.label('0.0 mm').style("
text = text.replace(old_slider_ui_lbl, new_slider_ui_lbl)

# 3. Slider bounds
old_slider_comp = "_grip_pos = ui.slider(min=0, max=1.3, step=0.1, value=0).props('label-always color=\"cyan\"').style('width: 100%; margin-top: 28px;')"
new_slider_comp = "_grip_pos = ui.slider(min=0.0, max=95.0, step=0.5, value=0.0).props('label-always color=\"cyan\"').style('width: 100%; margin-top: 28px;')"
text = text.replace(old_slider_comp, new_slider_comp)

# 4. Limit configuration Label
old_limit_lbl = "ui.html('<span style=\"font-size: 0.85rem; font-weight: 600; color: var(--accent-cyan);\">Current (°):</span>')"
new_limit_lbl = "ui.html('<span style=\"font-size: 0.85rem; font-weight: 600; color: var(--accent-cyan);\">Current (mm):</span>')"
text = text.replace(old_limit_lbl, new_limit_lbl)

# 5. _get_gripper_params
old_get_grip = """        acc_s = ui_labels.get('gripper_acc_slider')
        pos = float(pos_s.value) if pos_s and pos_s.value is not None else 0.0"""
new_get_grip = """        acc_s = ui_labels.get('gripper_acc_slider')
        pos_mm = float(pos_s.value) if pos_s and pos_s.value is not None else 0.0
        pos = pos_mm / 75.1339  # mm to rev"""
text = text.replace(old_get_grip, new_get_grip)

# 6. on_gripper_move log
old_grp_move = "st.set_text(f'Moving → {pos:.2f} rev ({pos*360:.1f}°)')"
new_grp_move = "st.set_text(f'Moving → {pos*75.1339:.1f} mm')"
text = text.replace(old_grp_move, new_grp_move)

# 7. max min logic replacements
old_max_1 = """        angle_deg = float(angle_input.value)
        max_pos_rev = angle_deg / 360.0"""
new_max_1 = """        pos_mm = float(angle_input.value)
        max_pos_rev = pos_mm / 75.1339"""
text = text.replace(old_max_1, new_max_1)

old_max_2 = "st.set_text(f'Setting max position: {angle_deg:.2f}° ({max_pos_rev:.4f} rev)...')"
new_max_2 = "st.set_text(f'Setting max position: {pos_mm:.1f} mm ({max_pos_rev:.4f} rev)...')"
text = text.replace(old_max_2, new_max_2)

old_max_3 = "print(f'[UI] Set Max Position: {angle_deg:.2f}° → {max_pos_rev:.4f} rev"
new_max_3 = "print(f'[UI] Set Max Position: {pos_mm:.1f} mm → {max_pos_rev:.4f} rev"
text = text.replace(old_max_3, new_max_3)

old_min_1 = """        angle_deg = float(angle_input.value)
        min_pos_rev = angle_deg / 360.0"""
new_min_1 = """        pos_mm = float(angle_input.value)
        min_pos_rev = pos_mm / 75.1339"""
text = text.replace(old_min_1, new_min_1)

old_min_2 = "st.set_text(f'Setting min position: {angle_deg:.2f}° ({min_pos_rev:.4f} rev)...')"
new_min_2 = "st.set_text(f'Setting min position: {pos_mm:.1f} mm ({min_pos_rev:.4f} rev)...')"
text = text.replace(old_min_2, new_min_2)

# 8. update_ui formatting for pos format
old_upd_pos = "_upd('pos', fmt_pos, 'value-position')"
new_upd_pos = """
        def _get_pos_fmt(v):
            if v is None: return 'N/A'
            return f'{v*75.1339:.1f} mm ({v:.3f} rev)' if jid == 7 else fmt_pos(v)
            
        _upd('pos', _get_pos_fmt, 'value-position')"""
text = text.replace(old_upd_pos, new_upd_pos)

# 9. slider label sync
old_slider_sync = """    if _gps and _gpl:
        _gv = float(_gps.value) if _gps.value is not None else 0.0
        _gpl.set_text(f'{_gv:.2f} rev ({_gv*360:.1f}°)')"""
new_slider_sync = """    if _gps and _gpl:
        _gv_mm = float(_gps.value) if _gps.value is not None else 0.0
        _gpl.set_text(f'{_gv_mm:.1f} mm ({_gv_mm / 75.1339:.3f} rev)')"""
text = text.replace(old_slider_sync, new_slider_sync)

# 10. angle input sync
old_angle_sync = """        if _gpos is not None:
            _new_deg = round(_gpos * 360.0, 2)
            # Only auto-update if user hasn't manually edited (avoid fighting user input)
            if _gai.value is None or abs(float(_gai.value) - _new_deg) > 0.5:
                _gai.value = _new_deg
                _gai.update()"""
new_angle_sync = """        if _gpos is not None:
            _new_mm = round(_gpos * 75.1339, 1)
            # Only auto-update if user hasn't manually edited (avoid fighting user input)
            if _gai.value is None or abs(float(_gai.value) - _new_mm) > 0.5:
                _gai.value = _new_mm
                _gai.update()"""
text = text.replace(old_angle_sync, new_angle_sync)

with open(file_path, 'w', encoding='utf-8') as f:
    f.write(text)

print("done")
