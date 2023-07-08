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
