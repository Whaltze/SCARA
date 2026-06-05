
with open('V_monitor.py', 'r', encoding='utf-8') as f:
    lines = f.readlines()

# Patch 1: add sample_dt param
for i, l in enumerate(lines):
    if 'def __init__(self, kinematics_engine):' in l:
        lines[i] = '    def __init__(self, kinematics_engine, sample_dt=0.02):' + chr(10)
        break

# Patch 2: replace start_timestamp with dt + virtual_time
for i, l in enumerate(lines):
    if 'self.start_timestamp = time.perf_counter()' in l:
        lines[i] = '        self.dt = sample_dt  # Fixed planned sampling interval' + chr(10)
        lines.insert(i+1, '        self.virtual_time = 0.0  # Accumulated planned time' + chr(10))
        break

# Patch 3: remove perf_counter in process_new_data
for i, l in enumerate(lines):
    if 'now = time.perf_counter()' in l and 'def process_new_data' in ''.join(lines[max(0,i-20):i]):
        lines[i] = chr(10)  # blank line
        break

# Patch 4: simplify theta check
for i, l in enumerate(lines):
    if 'if self.last_theta is not None and self.last_time is not None:' in l:
        lines[i] = '        if self.last_theta is not None:' + chr(10)
        break

# Patch 5: fixed dt
for i, l in enumerate(lines):
    if 'dt = now - self.last_time' in l:
        lines[i] = '            dt = self.dt  # Fixed planned interval' + chr(10)
        break

# Patch 6: fixed t_rel
for i, l in enumerate(lines):
    if 't_rel = now - self.start_timestamp' in l:
        lines[i] = '                t_rel = self.virtual_time' + chr(10)
        break

# Patch 7: virtual_time accumulation instead of last_time = now
for i, l in enumerate(lines):
    if 'self.last_time = now' in l and 'process_new_data' in ''.join(lines[max(0,i-20):i]):
        lines[i] = '        self.virtual_time += self.dt' + chr(10)
        break

# Patch 8: clear_all reset
for i, l in enumerate(lines):
    if 'self.start_timestamp = time.perf_counter()' in l:
        lines[i] = '        self.virtual_time = 0.0' + chr(10)
        break

with open('V_monitor.py', 'w', encoding='utf-8') as f:
    f.writelines(lines)

compile(open('V_monitor.py', encoding='utf-8').read(), 'V.py', 'exec')
print('Syntax OK')
