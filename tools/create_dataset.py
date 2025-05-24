import pandas as pd

file_template = 'pyplot/kuka_{arms}/stats_norm_{t}.log'
trivial_file_template = 'pyplot/kuka_{arms}/stats_trivial_{t}.log'
base_file_template = 'pyplot/kuka_{arms}/stats_norm_1.log'

all_data = pd.DataFrame()

arms_list = [2,3,4,6,8]
t_list = [1,11,51,101,201,501,1001]

tasks_count = 50

eps = 1e-3

easy_tasks = dict()

for arms in arms_list:
    arm_data = None
    for t in t_list:
        try:
            data = pd.read_csv(file_template.format(arms=arms, t=t))
            base_data = pd.read_csv(base_file_template.format(arms=arms))
        except Exception:
            continue
        if arm_data is None:
            arm_data = data['pathFound']
        else:
            arm_data += data['pathFound']
    if arm_data is not None:
        easy_tasks[arms] = arm_data
            

for arms in arms_list:
    for t in t_list:
        try:
            data = pd.read_csv(file_template.format(arms=arms, t=t))
            base_data = pd.read_csv(base_file_template.format(arms=arms))
            trivial_data = pd.read_csv(trivial_file_template.format(arms=arms, t=t))
        except Exception:
            print(arms, t, "not found")
            continue
        print(arms, t, "    found")
        data['test_id'] = range(1, 51)
        data['arms'] = arms
        data['constraint_interval'] = t
        data['pathCostNorm'] = data['pathCost'] / trivial_data['pathTrivialCost']
        data['taskSolved'] = easy_tasks[arms]
        all_data = pd.concat([all_data, data])
all_data.to_csv('pyplot/all_data.csv')
