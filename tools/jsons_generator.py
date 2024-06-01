import json

output_folder = 'parameters/launch/'
# ws = [1.0, 1.1, 1.2, 1.5, 2.0, 4.0, 10.0, 30.0, 100.0]
nums = range(4, 12)

w = 1.0
# for w in ws:
for num in nums:
    with open(output_folder + f'config_{num}.json', "w+") as outfile:
        config = {
            "model_filename": f"model/2-dof/manipulator_{num}.xml",
            "algorithm": {
                "time_limit": 3.0,
                "type": 1,
                "heuristic": {
                    "weight": w
                }
            },
            "preprocess": {
                "type": 0,
                "clusters": 0,
            },
            "taskset": {
                "use_random_tasks": True,
                "task_number": 100,
                "random_seed": 12345,
                "task_type": 0,
                "taskset_filename": "scenaries/4_2-dof_pos_hard.scen"
            },
            "output": {
                "profiling": f"pyplot/0/runtime_hard_w={w}.log",
                "statistics": f"pyplot/0/stats_hard_w={w}.log",
                "taskset": "scenaries/scen.log",
                "configuration_space": f"maps/c_space_{num}.map",
                "paths_folder": f"maps/paths_{num}/"
            },
            "display_motion": False
        }
        json.dump(config, outfile, indent='\t')
