file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

start_block1 = -1
for i, line in enumerate(lines):
    if '# ---- Gripper Card (ID 7) ----' in line:
        start_block1 = i
        break

end_block1 = -1
for i in range(start_block1, len(lines)):
    if "ui.button('\\U0001f3e0 Set Home'" in lines[i]:
        end_block1 = i + 3
        break

for i in range(start_block1, end_block1):
    if lines[i].strip():
        lines[i] = "    " + lines[i]

# Find controls block
start_block2 = -1
for i in range(end_block1, len(lines)):
    if "# Enable / Disable row" in lines[i]:
        start_block2 = i
        break

end_block2 = -1
for i in range(start_block2, len(lines)):
    if "# ---- Chart modal ----" in lines[i] or "ui.timer(" in lines[i]:
        end_block2 = i
        break

for i in range(start_block2, end_block2):
    if lines[i].strip() and not lines[i].startswith("            with ui.element('div').style('flex: 2"):
        lines[i] = "    " + lines[i]

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)
print("Done")
