file_path = '/home/liyq/moteus/utils/webgui.py'
with open(file_path, 'r', encoding='utf-8') as f:
    lines = f.readlines()

start_idx = 2230
end_idx = start_idx
for i in range(start_idx, len(lines)):
    if 'dlg.open()' in lines[i]:
        end_idx = i + 1
        break

for i in range(start_idx, end_idx):
    if lines[i].startswith("    "):
        lines[i] = lines[i][4:]

with open(file_path, 'w', encoding='utf-8') as f:
    f.writelines(lines)
    
print(f"Unindented from {start_idx} to {end_idx}")
