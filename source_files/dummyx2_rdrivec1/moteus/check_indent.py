with open('/home/liyq/moteus/utils/webgui.py', 'r') as f:
    lines = f.readlines()
for i in range(2225, 2940, 50):
    print(f"Line {i}: {repr(lines[i][:16])}...")
