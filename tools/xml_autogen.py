joints = 3
length = [1, 1.2, 1.6]
path_to_obstacles = 'tools/args/obstacles.txt'
path_to_header = 'tools/const/header.txt'
path_to_footer = 'tools/const/footer.txt'
offset4 = ' ' * 4
offset8 = ' ' * 8


def collision_section(joints, obstacles):
    output = ''
    for i in range(joints):
        for j in range(obstacles):
            output += offset8 + f'<pair geom1=\"geom edge {i}\" geom2=\"geom obstacle {j}\"/>\n'
    for i in range(joints):
        for j in range(joints):
            if j - i > 1:
                output += offset8 + f'<pair geom1=\"geom edge {i}\" geom2=\"geom edge {j}\"/>\n'
    return output


def manipulator_autogen(joints, lengths: list):
    output = ''
    output_suffix = []
    output_prefix = ''
    current_space = offset8

    for i in range(joints):

        if i == 0:
            euler = '\"0 90 0\"'
            prev_half_len = 0
            current_half_len = lengths[i] / 2
        else:
            euler = '\"0 0 0\"'
            prev_half_len = lengths[i - 1] / 2
            current_half_len = lengths[i] / 2

        if i == 0:
            output_prefix += f'{current_space}<body name=\"edge {i}\" \
pos=\"{f"{prev_half_len + current_half_len} 0 0.1"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"edge {i}\" \
pos=\"{f"0 0 {prev_half_len + current_half_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + offset4}<joint name=\"joint {i}\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_half_len}"}\"/>\n'

        output_prefix += f'{current_space + offset4}<geom name=\"geom edge {i}\" \
type=\"cylinder\" size=\"{f"0.05 {current_half_len}"}\" material=\"red\"/>\n'

        if i == joints - 1:
            output_prefix += f'{current_space + offset4}<site name=\"tip\" size=\"0.1\" \
pos=\"{f"0 0 {current_half_len}"}\"/>\n'

        output_suffix.append(f'{current_space}</body>\n')
        current_space += offset4

    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'

    return output


def manipulator_shade_autogen(joints, lengths: list):
    output = ''
    current_space = offset8
    output_prefix = ''
    output_suffix = []

    for i in range(joints):

        if i == 0:
            euler = '\"0 90 0\"'
            prev_half_len = 0
            current_half_len = lengths[i] / 2
        else:
            euler = '\"0 0 0\"'
            prev_half_len = lengths[i - 1] / 2
            current_half_len = lengths[i] / 2

        if i == 0:
            output_prefix += f'{current_space}<body name=\"edge {i} shade\" \
pos=\"{f"{prev_half_len + current_half_len} 0 0.1"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"edge {i} shade\" \
pos=\"{f"0 0 {prev_half_len + current_half_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + offset4}<joint name=\"joint {i} shade\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_half_len}"}\"/>\n'

        output_prefix += f'{current_space + offset4}<geom name=\"geom edge {i} shade\" \
type=\"cylinder\" size=\"{f"0.05 {current_half_len}"}\" material=\"green\"/>\n'

        output_suffix.append(f'{current_space}</body>\n')
        current_space += offset4

    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'
    return output


def obstacles_autogen(file_path):
    obstacles = 0
    output = ''
    data = open(file_path).read()
    data = data.split('\n')
    for s in data:
        if s == '':
            continue
        output += f'        <body name=\"obstacle {obstacles}\">\n'
        output += f'            <geom name=\"geom obstacle {obstacles}\" {s} material=\"blue\"/>\n'
        output += f'        </body>\n'
        obstacles += 1
    return {'obstacles':output, 'obstacles_counter': obstacles}

gen_obstacles = obstacles_autogen(path_to_obstacles)
 
with open(f'tools/{joints}-dof_{gen_obstacles["obstacles_counter"]}-obs_manipulator.xml', "w+") as f:
    f.write(open(path_to_header, 'r').read())
    f.write('\n')
    f.write(manipulator_autogen(joints, length))
    f.write(manipulator_shade_autogen(joints, length))
    f.write(gen_obstacles['obstacles'])
    f.write('\n')
    f.write('    </worldbody>\n\n')
    f.write('    <contact>\n')
    f.write(collision_section(joints, gen_obstacles['obstacles_counter']))    
    f.write('    </contact>\n')
    f.write(open(path_to_footer, 'r').read())
