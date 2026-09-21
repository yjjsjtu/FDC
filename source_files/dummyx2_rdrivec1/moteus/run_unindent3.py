file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

for i in range(2299, len(lines)):
    if 'def update_ui():' in lines[i]:
        end_idx = i
        break

for i in range(2299, end_idx):
    if lines[i].startswith("    "):
        lines[i] = lines[i][4:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)
