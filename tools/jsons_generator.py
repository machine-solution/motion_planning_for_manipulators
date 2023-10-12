import json

output_folder = 'parameters/launch/'
ws = [1.0, 1.1, 1.2, 1.5, 2.0, 4.0, 10.0, 30.0, 100.0]

for w in ws:
    with open(output_folder + f'config_w={w}.json', "w+") as outfile:
        config = {
            "model_filename": "model/2-dof/manipulator_5.xml",
            "algorithm": {
                "time_limit": 3.0,
                "weight": w,
                "type": 1
            },
            "taskset": {
                "use_random_tasks": True,
                "task_number": 10000,
                "task_type": 0,
                "taskset_filename": "scenaries/4_2-dof_pos_hard.scen"
            },
            "output": {
                "profiling": f"pyplot/0/runtime_hard_w={w}.log",
                "statistics": f"pyplot/0/stats_hard_w={w}.log",
                "taskset": "scenaries/scen.log"
            },
            "display_motion": True
        }
        json.dump(config, outfile, indent='\t')
