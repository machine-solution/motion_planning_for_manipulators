joints = 2
obstacles = 6

file_path_to_edges = 'tools/edges.txt'
file_path_to_obstacles = 'tools/obstacles.txt'

with open(f'tools/{joints}-dof_{obstacles}-obs_manipulator.xml', "w+") as f:
    f.write(open('tools/const/header.txt', 'r').read())
    f.write('\n')
    f.write(open(file_path_to_edges, 'r').read())
    f.write(open(file_path_to_obstacles, 'r').read())
    f.write('\n')
    f.write('    </worldbody>\n\n')
    f.write('    <contact>\n')
    
    for i in range(joints):
        for j in range(obstacles):
            f.write(' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom obstacle {j}"/>\n')
    for i in range(joints):
        for j in range(joints):
            if j - i > 1:
                f.write(' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom edge {j}"/>\n')
    
    f.write('    </contact>\n')
    f.write(open('tools/const/footer.txt', 'r').read())

