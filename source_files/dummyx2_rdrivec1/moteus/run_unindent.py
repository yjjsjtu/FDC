import re

file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

start_idx = -1
for i, line in enumerate(lines):
    if line.startswith("        def _refresh_pid_inputs(jid):"):
        start_idx = i
        break

end_idx = -1
for i in range(start_idx, len(lines)):
    if line.startswith("    dlg.open()"):
        end_idx = i + 1
        break

if start_idx != -1 and end_idx != -1:
    for i in range(start_idx, end_idx):
        if lines[i].startswith("    "):
            lines[i] = lines[i][4:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)
    
print(f"Unindented from {start_idx} to {end_idx}")
