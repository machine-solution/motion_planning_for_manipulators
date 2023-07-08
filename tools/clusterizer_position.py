name = "scenaries/4_2-dof_pos"

input_file = name + "_scen.log"
output_easy = name + "_easy.scen"
output_medium = name + "_medium.scen"
output_hard = name + "_hard.scen"

medium_threshold = 1
hard_threshold = 2
dofs = 2
dim = 2


with open(output_easy, "w+") as easy:
    with open(output_medium, "w+") as medium:
        with open(output_hard, "w+") as hard:
            with open(input_file, "r") as inp:
                easy.write(f"{dofs}\n")
                medium.write(f"{dofs}\n")
                hard.write(f"{dofs}\n")
                inp.readline()
                for line in inp.readlines():
                    elems = line.split(",")
                    if float(elems[-1]) < medium_threshold:
                        outp = easy
                    elif float(elems[-1]) < hard_threshold:
                        outp = medium
                    else:
                        outp = hard

                    # if path cost is equal 0
                    if float(elems[dofs + dim]) < 1e-3:
                        continue

                    for i in range(dofs):
                        outp.write(elems[i].rjust(6, " "))
                    for i in range(dim):
                        outp.write(elems[i + dofs].rjust(14, " "))
                    outp.write(elems[dofs + dim].rjust(14, " ") + "\n")


