def collision_section(joints, obstacles):
    output = ''
    for i in range(joints):
        for j in range(obstacles):
            output += ' ' * 8 + f'<pair geom1=\"geom edge {i}\" geom2=\"geom obstacle {j}\"/>\n'
    for i in range(joints):
        for j in range(joints):
            if j - i > 1:
                output += ' ' * 8 + f'<pair geom1=\"geom edge {i}\" geom2=\"geom edge {j}\"/>\n'
    return output

def manipulator_autogen(joints, lengths: list):
    output = ''
    output_suffix = []
    output_prefix = ''
    current_space = '        '

    for i in range(joints):

        if i == 0:
            euler = '\"0 90 0\"'
            prev_len = 0
            current_len = lengths[i]
        else:
            euler = '\"0 0 0\"'
            prev_len = lengths[i - 1]
            current_len = lengths[i]

        if i == 0:
            output_prefix += f'{current_space}<body name=\"edge {i}\" \
pos=\"{f"{prev_len + current_len} 0 0.1"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"edge {i}\" \
pos=\"{f"0 0 {prev_len + current_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + "    "}<joint name=\"joint {i}\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_len}"}\"/>\n'

        output_prefix += f'{current_space + "    "}<geom name=\"geom edge {i}\" \
type=\"cylinder\" size=\"{f"0.05 {current_len}"}\" material=\"red\"/>\n'

        if i == joints - 1:
            output_prefix += f'{current_space + "    "}<site name=\"tip\" size=\"0.1\" \
pos=\"{f"0 0 {current_len}"}\"/>\n'

        output_suffix.append(f'{current_space}</body>\n')
        current_space += '    '

    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'

    current_space = '        '
    output_prefix = ''
    output_suffix = []

    for i in range(joints):

        if i == 0:
            euler = '\"0 90 0\"'
            prev_len = 0
            current_len = lengths[i]
        else:
            euler = '\"0 0 0\"'
            prev_len = lengths[i - 1]
            current_len = lengths[i]

        if i == 0:
            output_prefix += f'{current_space}<body name=\"edge {i} shade\" \
pos=\"{f"{prev_len + current_len} 0 0.1"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"edge {i} shade\" \
pos=\"{f"0 0 {prev_len + current_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + "    "}<joint name=\"joint {i} shade\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_len}"}\"/>\n'

        output_prefix += f'{current_space + "    "}<geom name=\"geom edge {i} shade\" \
type=\"cylinder\" size=\"{f"0.05 {current_len}"}\" material=\"green\"/>\n'

        output_suffix.append(f'{current_space}</body>\n')
        current_space += '    '

    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'

    return output

def obstacles_autogen(billet):
    output = ''
    billet = billet.split('\n')
    if billet[0] == '':
        del billet[0] 
    if billet[-1] == '':
        del billet[-1]
    input = []
    for s in billet:
        obs = {}
        characteristics = s.split('"')
        obs['type'] = characteristics[1]
        obs['size'] = characteristics[3]
        obs['pos'] = characteristics[5]
        input.append(obs)
    for i in range(len(input)):
        output += f'        <body name=\"obstacle {i}\">\n'
        output += f'            <geom name=\"geom obstacle {i}\" type=\"{input[i]["type"]}\" \
size=\"{input[i]["size"]}\" pos=\"{input[i]["pos"]}\" material=\"blue\"/>\n'
        output += f'        </body>\n'
    return output

joints = 3
obstacles = 4

with open(f'tools/{joints}-dof_{obstacles}-obs_manipulator.xml', "w+") as f:
    f.write(open('tools/const/header.txt', 'r').read())
    f.write('\n')
    f.write(manipulator_autogen(joints, [0.3, 0.4, 0.6]))
    f.write('\n')
    f.write(obstacles_autogen( \
"""
type="box" size="0.1 0.2 0.3" pos="1.5 0.9 0.0"
type="sphere" size="0.2" pos="0.0 -1.5 0.0"
""" \
))
    f.write('\n')
    f.write('    </worldbody>\n\n')
    f.write('    <contact>\n')
    f.write(collision_section(joints, obstacles))    
    f.write('    </contact>\n')
    f.write(open('tools/const/footer.txt', 'r').read())
