arms = 8

arm_path = 'model/iiwa14_template.xml'
arm_shade_path = 'model/iiwa14_template_shade.xml'
collide_path = 'model/iiwa14_template_collide.xml'

arm_path_out = 'model/iiwa14_{i}.xml'
arm_shade_path_out = 'model/iiwa14_shade_{i}.xml'
collide_path_out = 'model/iiwa14_collide_{i}.xml'

template = open(arm_path).read()
template_shade = open(arm_shade_path).read()
collide = open(collide_path).read()

for i in range(arms):
    with open(arm_path_out.format(i=i), "w+") as f:
        f.write(template.format(id=i))
    with open(arm_shade_path_out.format(i=i), "w+") as f:
        f.write(template_shade.format(id=f'{i}_shade'))
    with open(collide_path_out.format(i=i), "w+") as f:
        f.write(collide.format(id=i))