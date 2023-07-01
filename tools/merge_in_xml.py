dof = 2
obstacles = 6
with open(f"tools/{dof}-dof/manipulator.xml", "w+") as f:
    f.write(open('tools/header.txt', 'r').read())
    f.write('\n')
    f.write(open('tools/edges.txt', 'r').read())
    f.write(open('tools/obstacles.txt', 'r').read())
    f.write('\n')
    f.write('    </worldbody>\n\n')
    f.write('    <contact>\n')
    f.write(open('tools/collision_section.txt', 'r').read())
    f.write('    </contact>\n')
    f.write(open('tools/footer.txt', 'r').read())
