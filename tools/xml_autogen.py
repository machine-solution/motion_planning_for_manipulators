def collision_section(joints, obstacles):
    output = ''
    for i in range(joints):
        for j in range(obstacles):
            output += ' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom obstacle {j}"/>\n'
    for i in range(joints):
        for j in range(joints):
            if j - i > 1:
                output += ' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom edge {j}"/>\n'
    return output

def manipulator_autogen(joints, size, pos):
    output = ''
    output_suffix = []
    output_prefix = ''
    current_space = '        '
    for i in range(joints):
        if i == 0:
            euler = '\"0 90 0\"'
        else:
            euler = '\"0 0 0\"'
        output_prefix += f'{current_space}<body name=\"edge {i}\" pos={pos[f"edge_{i}"]} euler={euler}>\n'
        
        output_prefix += f'{current_space + "    "}<joint name=\"joint {i}\" type=\"hinge\" axis=\"-1 0 0\" pos={pos["joints"]}/>\n'
        output_prefix += f'{current_space + "    "}<geom name=\"geom edge {i}\" type=\"cylinder\" size={size["cylinder"]} material=\"blue\"/>\n'
        if i == joints - 1:
            output_prefix += f'{current_space + "    "}<site name=\"tip\" size={size["tip"]} pos={pos["tip"]}/>\n'
        output_suffix.append(f'{current_space}</body>\n')
        current_space += '    '
    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'
    current_space = '        '
    output_prefix = ''
    output_suffix = []
    for i in range(joints):
        if i == 0:
            euler = '\"0 90 0\"'
        else:
            euler = '\"0 0 0\"'
        output_prefix += f'{current_space}<body name=\"edge {i} shade\" pos={pos[f"edge_{i}"]} euler={euler}>\n'
        output_prefix += f'{current_space + "    "}<joint name=\"joint {i} shade\" type=\"hinge\" axis=\"-1 0 0\" pos={pos["joints"]}/>\n'
        output_prefix += f'{current_space + "    "}<geom name=\"geom edge {i} shade\" type=\"cylinder\" size={size["cylinder"]} material=\"green\"/>\n'
        output_suffix.append(f'{current_space}</body>\n')
        current_space += '    '
    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1])
    return output

def obstacles_autogen(obstacles, obstacles_types, \
    obstacles_sizes, obstacles_positions):
    output = ''
    for i in range(obstacles):
        output += f'        <body name="obstacle {i}">\n'
        output += f'            <geom name="geom obstacle {i}" type="{obstacles_types[f"obs_{i}"]}" size="{obstacles_sizes[f"obs_{i}"]}" pos="{obstacles_positions[f"obs_{i}"]}" material="blue"/>\n'
        output += f'        </body>\n'
    return output

joints = 2
obstacles = 4

with open(f'tools/{joints}-dof_{obstacles}-obs_manipulator.xml', "w+") as f:
    f.write(open('tools/const/header.txt', 'r').read())
    f.write('\n')
    f.write(manipulator_autogen(joints, {"cylinder":'\"0.5 0 0.1\"', \
    "tip":'\"0.1\"'}, {"edge_0":'\"0.5 0 0.1\"', "edge_1":'\"0 0 1\"', "joints":'\"0 0 -0.5\"', "tip":'\"0 0 0.4\"'}))
    f.write('\n')
    f.write(obstacles_autogen(obstacles, {'obs_0':'box', \
    'obs_1':'sphere', 'obs_2':'sphere', 'obs_3':'box'}, \
    {'obs_0':'0.1 0.2 0.3', 'obs_1':'0.2', \
    'obs_2':'0.4', 'obs_3':'0.2 0.1 0.3'}, \
    {'obs_0':'1.5 0.9 0.0', 'obs_1':'0.0 -1.5 0.0', \
    'obs_2':'-1.5 0.0 0.0', 'obs_3':'1.5 0.9 0.0'}))
    f.write('\n')
    f.write('    </worldbody>\n\n')
    f.write('    <contact>\n')
    f.write(collision_section(joints, obstacles))    
    f.write('    </contact>\n')
    f.write(open('tools/const/footer.txt', 'r').read())
