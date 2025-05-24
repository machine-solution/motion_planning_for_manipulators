# TODO MOVE FLOOR DOWN

joint_length = 0.5

manipulators = [
    (
        [joint_length, joint_length, joint_length],
        (-1, -1, 0),
    ),
    (
        [joint_length, joint_length, joint_length],
        ( 1, -1, 0),
    ),
    (
        [joint_length, joint_length, joint_length],
        ( 1, 1, 0),
    ),
    (
        [joint_length, joint_length, joint_length],
        ( -1, 1, 0),
    ),
] 
# first element - list of lengths, second element - offset
# all manipulators must have same dof

arms = len(manipulators) # do not change
joints = len(manipulators[0][0]) # do not change

path_to_obstacles = 'maps/free_0.txt'
path_to_result = f'model/{arms}-arm/{joints}-dof/manipulator_0.xml'


offset4 = ' ' * 4 # do not change
offset8 = ' ' * 8 # do not change
path_to_header = 'tools/const/header.txt' # do not change
path_to_sphere = 'tools/const/sphere.txt' # do not change
path_to_footer = 'tools/const/footer.txt' # do not change


def collision_section(joints, arms, obstacles):
    output = ''
    # shades collisions
    for a in range(arms):
        for i in range(joints - 1):
            output += offset8 + f'<exclude body1=\"link {a}-{i}\" body2=\"link {a}-{i + 1}\"/>\n'

    # # all shades with all links
    # for a in range(arms):
    #     for b in range(arms):
    #         for i in range(joints):
    #             for j in range(joints):
    #                 output += offset8 + f'<exclude body1=\"link {a}-{i} shade\" body2=\"link {b}-{j}\"/>\n'
    # # all shades with all obstacles
    # for a in range(arms):
    #     for i in range(joints):
    #         for j in range(obstacles):
    #             output += offset8 + f'<exclude body1=\"link {a}-{i} shade\" body2=\"obstacle {j}\"/>\n'
    return output


def manipulator_autogen(num, joints, lengths: list, base: tuple):
    output = ''
    output_suffix = []
    output_prefix = ''
    current_space = offset8

    x, y, z = base

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
            output_prefix += f'{current_space}<body name=\"link {num}-{i}\" \
pos=\"{f"{prev_half_len + current_half_len + x} {0 + y} {0.1 + z}"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"link {num}-{i}\" \
pos=\"{f"0 0 {prev_half_len + current_half_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + offset4}<joint name=\"joint {num}-{i}\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_half_len}"}\"/>\n'

        output_prefix += f'{current_space + offset4}<geom name=\"geom link {num}-{i}\" \
type=\"cylinder\" size=\"{f"0.05 {current_half_len}"}\" material=\"red\"/>\n'

#         if i == joints - 1:
#             output_prefix += f'{current_space + offset4}<site name=\"tip {num}\" size=\"0.1\" \
# pos=\"{f"{0 + x} {0 + y} {current_half_len + z}"}\"/>\n'

        output_suffix.append(f'{current_space}</body>\n')
        current_space += offset4

    output += output_prefix + ''.join(output_suffix[-1:-len(output_suffix)-1:-1]) + '\n'

    return output


def manipulator_shade_autogen(num, joints, lengths: list, base: tuple):
    output = ''
    current_space = offset8
    output_prefix = ''
    output_suffix = []

    x, y, z = base

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
            output_prefix += f'{current_space}<body name=\"link {num}-{i} shade\" \
pos=\"{f"{prev_half_len + current_half_len + x} {0 + y} {0.1 + z}"}\" euler={euler}>\n'
        else:
            output_prefix += f'{current_space}<body name=\"link {num}-{i} shade\" \
pos=\"{f"0 0 {prev_half_len + current_half_len}"}\" euler={euler}>\n'

        output_prefix += f'{current_space + offset4}<joint name=\"joint {num}-{i} shade\" type=\"hinge\" \
axis=\"-1 0 0\" pos=\"{f"0 0 {-current_half_len}"}\"/>\n'

        output_prefix += f'{current_space + offset4}<geom name=\"geom link {num}-{i} shade\" \
type=\"cylinder\" size=\"{f"0.02 {current_half_len}"}\" material=\"green\" contype=\"0\" conaffinity=\"0\" />\n'

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


def footer_gen(arms):
    output = '\n'
    output += '    <sensor>\n'
    for num in range(arms):
        output += f'        <framepos objtype="site {num}" objname="tip"/>\n'
    output += '    </sensor>\n'
    return output


gen_obstacles = obstacles_autogen(path_to_obstacles)

with open(path_to_result, "w+") as f:
    f.write(open(path_to_header, 'r').read())
    f.write('\n')
    for num, lengths_base in enumerate(manipulators):
        lengths, base = lengths_base
        f.write(manipulator_autogen(num, joints, lengths, base))
    f.write('\n')
    for num, lengths_base in enumerate(manipulators):
        lengths, base = lengths_base
        f.write(manipulator_shade_autogen(num, joints, lengths, base))
    f.write(open(path_to_sphere, 'r').read())
    f.write(gen_obstacles['obstacles'])
    f.write('\n')
    f.write('    </worldbody>\n')
    f.write('    <contact>\n')
    f.write(collision_section(joints, arms, gen_obstacles['obstacles_counter']))    
    f.write('    </contact>\n')
    # f.write(footer_gen(arms))
    f.write(open(path_to_footer, 'r').read())
