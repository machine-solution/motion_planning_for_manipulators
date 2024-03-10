input_file = "scenaries/31/scen_dof=5_w=10.0.log"
output_file = "scenaries/31/dof=5_u=64.scen"

min_threshold = 0
max_threshold = 600
tasks_out = 100
dofs = 5


with open(output_file, "w+") as outp:
    with open(input_file, "r") as inp:
        outp.write(f"{dofs}\n")
        inp.readline()
        lines = []
        for line in inp.readlines():
            elems = line.split(",")
            if float(elems[-1]) < min_threshold:
                continue
            elif float(elems[-1]) > max_threshold:
                continue
            else:
                lines.append((line, float(elems[-1])))

        sorted_lines = sorted(lines, key=lambda x: -x[1])

        for num in range(min(len(sorted_lines), tasks_out)):
            line, runtime = sorted_lines[num]
            elems = line.split(",")
            for i in range(dofs):
                outp.write(elems[i].rjust(6, " "))
            for i in range(dofs):
                outp.write(elems[i + dofs].rjust(6, " "))
            outp.write(elems[dofs * 2].rjust(14, " ") + "\n")


