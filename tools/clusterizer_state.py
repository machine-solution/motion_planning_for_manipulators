name = "scenaries/4_4-dof"

input_file = name + "_scen.log"
output_easy = name + "_easy.scen"
output_medium = name + "_medium.scen"
output_hard = name + "_hard.scen"

medium_threshold = 0 # 0.15
hard_threshold = 0 # 0.4
dofs = 4


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

                    for i in range(dofs):
                        outp.write(elems[i].rjust(6, " "))
                    for i in range(dofs):
                        outp.write(elems[i + dofs].rjust(6, " "))
                    outp.write(elems[dofs * 2].rjust(14, " ") + "\n")


