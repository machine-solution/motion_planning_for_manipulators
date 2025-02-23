scen_name = "scenaries/kuka_2/tasks_1.log"
stats_name = "pyplot/kuka_2/stats_1.log"
taskset_name = "scenaries/tasksets/kuka_2.txt"

number = 50
dof = 7
arms = 2

test_size = 3 + 2 * arms

with open(scen_name, "r") as scen:
    scen_list = scen.readlines()
with open(stats_name, "r") as stats:
    stats_list = stats.readlines()


with open(taskset_name, "w+") as outp:
    outp.write(f"{dof} {arms}\n\n")
    for i in range(1, len(stats_list)):
        if number == 0:
            break
        if stats_list[i].strip().split(',')[-1] == "0":
            number -= 1
            for j in range(test_size):
                outp.write(scen_list[2 + i * test_size + j])



