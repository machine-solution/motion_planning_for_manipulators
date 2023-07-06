joints = 2
obstacles = 6

with open("tools/collision_section.txt", "w+") as f:
    for i in range(joints):
        for j in range(obstacles):
            f.write(' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom obstacle {j}"/>\n')
    for i in range(joints):
        for j in range(joints):
            if j - i > 1:
                f.write(' ' * 8 + f'<pair geom1="geom edge {i}" geom2="geom edge {j}"/>\n')

with open(f"tools/{joints}-dof/manipulator.xml", "w+") as f:
    f.write(open('tools/header.txt', 'r').read())
    f.write('\n')
    f.write(open('tools/edges.txt', 'r').read())
    f.write(open('tools/obstacles.txt', 'r').read())
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
    f.write(open('tools/footer.txt', 'r').read())
