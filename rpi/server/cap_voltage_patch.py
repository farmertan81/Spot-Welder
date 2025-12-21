import sys

# Read the file
with open('app.py', 'r') as f:
    lines = f.readlines()

# Find the import section and add our import
for i, line in enumerate(lines):
    if 'from ads1256 import' in line or 'import ads1256' in line:
        # Add after existing ads1256 import
        if 'V_CAP_CH' not in line:
            lines[i] = line.rstrip() + ', V_CAP_CH, counts_to_cap_volts\n'
        break
else:
    # No ads1256 import found, add it after other imports
    for i, line in enumerate(lines):
        if line.startswith('import ') or line.startswith('from '):
            continue
        else:
            lines.insert(i, 'from ads1256 import V_CAP_CH, counts_to_cap_volts\n')
            break

# Add SUPERCAP constant after other constants
for i, line in enumerate(lines):
    if 'TOTAL_RESISTANCE_OHM' in line:
        # Add after this line if not already there
        if i+1 < len(lines) and 'SUPERCAP_FARADS' not in lines[i+1]:
            lines.insert(i+1, 'SUPERCAP_FARADS = 58.0  # Total capacitance of 2S pack\n')
        break

# Find PEDAL trigger and add v_start reading
for i, line in enumerate(lines):
    if '"trigger weld" in msg:' in line:
        # Find the next line with weld_lock
        for j in range(i, min(i+10, len(lines))):
            if 'with weld_lock:' in lines[j]:
                # Add v_start reading after the with statement
                indent = '                '
                insert_pos = j + 1
                if 'v_start' not in ''.join(lines[j:j+5]):
                    lines.insert(insert_pos, f'{indent}# Read cap voltage before weld\n')
                    lines.insert(insert_pos+1, f'{indent}v_start_counts = adc.read_channel(V_CAP_CH)\n')
                    lines.insert(insert_pos+2, f'{indent}v_start = counts_to_cap_volts(v_start_counts)\n')
                    lines.insert(insert_pos+3, f'{indent}log(f"📊 Cap voltage before weld: {{v_start:.2f}} V")\n')
                break
        break

# Find FIRED handler and add v_end reading + energy calc
for i, line in enumerate(lines):
    if 'if msg.startswith("FIRED,") or msg.startswith("FIRED"):' in line:
        # Find where we stop capturing
        for j in range(i, min(i+20, len(lines))):
            if 'is_capturing = False' in lines[j]:
                indent = '                '
                insert_pos = j + 1
                if 'v_end' not in ''.join(lines[j:j+10]):
                    lines.insert(insert_pos, f'{indent}# Read cap voltage after weld\n')
                    lines.insert(insert_pos+1, f'{indent}v_end_counts = adc.read_channel(V_CAP_CH)\n')
                    lines.insert(insert_pos+2, f'{indent}v_end = counts_to_cap_volts(v_end_counts)\n')
                    lines.insert(insert_pos+3, f'{indent}E_cap = 0.5 * SUPERCAP_FARADS * (v_start**2 - v_end**2)\n')
                    lines.insert(insert_pos+4, f'{indent}log(f"📊 Cap voltage after weld: {{v_end:.2f}} V, Energy from cap: {{E_cap:.2f}} J")\n')
                break
        break

# Write back
with open('app.py', 'w') as f:
    f.writelines(lines)

print("✅ Patched app.py with cap voltage sensing")
