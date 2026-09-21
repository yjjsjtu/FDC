import re

file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Fix CSS grid 7 to 6
for i, line in enumerate(lines):
    if '.cards-grid {' in line:
        for j in range(i, i+10):
            if 'repeat(7, 1fr)' in lines[j]:
                lines[j] = lines[j].replace('repeat(7, 1fr)', 'repeat(6, 1fr)')
        break

# Extract Gripper Card
start_idx = -1
end_idx = -1
for i, line in enumerate(lines):
    if '# ---- Gripper Card (ID 7) ----' in line:
        start_idx = i
        break

if start_idx != -1:
    for i in range(start_idx + 1, len(lines)):
        if "color: #06d6a0 !important; flex: 1; font-weight: bold;')" in lines[i]:
            end_idx = i + 1
            break

gripper_card_lines = lines[start_idx:end_idx]
del lines[start_idx:end_idx]

# Find Gripper Control Panel
cp_start = -1
for i, line in enumerate(lines):
    if '# ---- Gripper Control Panel ----' in line:
        cp_start = i
        break

# We will inject the layout wrapping right after the title
# Before:
#     # ---- Gripper Control Panel ----
#     with ui.element('div').classes('control-panel'):
#         ui.html('<div class="control-panel-title" style="color: #f97316;">'
#                 '\U0001f9be 夹爪控制 (Gripper Control)</div>')
#         
#         # Enable / Disable row
title_end = cp_start + 4

# Let's adjust the indentation of gripper_card_lines to match the new scope.
# It was inside `for jid ...` and `cards-grid` ? Wait, original indentation:
# `        # ---- Gripper Card (ID 7) ----` (8 spaces)
# In Control Panel, inside `with ui.element('div').classes('control-panel'):` we have 8 spaces indentation.
# Inside a new flex div we will need 12 spaces.
flex_start = [
    "        with ui.element('div').style('display: flex; gap: 40px; align-items: flex-start; flex-wrap: wrap;'):\n",
    "            with ui.element('div').style('min-width: 300px; max-width: 380px; flex: 1;'):\n"
]
# Increase indent of gripper card by 4 spaces
new_gripper_card = ["    " + line for line in gripper_card_lines]

controls_start = [
    "            with ui.element('div').style('flex: 2; min-width: 400px; display: flex; flex-direction: column;'):\n"
]

# The remaining lines of Gripper Control Panel currently have 8 spaces indent.
# We need to find the end of Gripper Control Panel to indent them by +4 spaces.
# It ends right before `    # ---- Chart modal ----` or `ui.timer(0.02, update_ui)`
cp_end = -1
for i in range(title_end, len(lines)):
    if '    # ---- Chart modal ----' in lines[i] or 'ui.timer(' in lines[i]:
        cp_end = i
        break

# Indent controls
for i in range(title_end, cp_end):
    if lines[i].strip():
        lines[i] = "    " + lines[i]

# Insert everything
insertion = flex_start + new_gripper_card + controls_start
lines = lines[:title_end] + insertion + lines[title_end:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)

print("Done")
