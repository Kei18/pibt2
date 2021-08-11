import os
import re
import csv
import subprocess
import tqdm
from itertools import product
from datetime import datetime, timedelta, timezone

def get_date_str():
    return datetime.now(timezone(timedelta(hours=+9), "JST")).strftime("%Y-%m-%d-%H-%M-%S")


def read_result(filename):
    r_solved= re.compile(r"solved=(\d+)")
    r_service_time = re.compile(r"service_time=(.+)")
    r_makespan = re.compile(r"makespan=(\d+)")
    r_comp_time = re.compile(r"comp_time=(\d+)")

    solved, service_time, makespan, comp_time = -1, -1, -1, -1
    with open(result_file, 'r') as f_read:
        for row in f_read:
            m = re.match(r_solved, row)
            if m:
                solved = int(m.group(1))
            m = re.match(r_service_time, row)
            if m:
                service_time = float(m.group(1))
            m = re.match(r_makespan, row)
            if m:
                makespan = int(m.group(1))
            m = re.match(r_comp_time, row)
            if m:
                comp_time = int(m.group(1))
    return solved, service_time, makespan, comp_time


MAX_TIMESTEP = 3000
MAX_COMP_TIME = 30000
SOLVERS = [ "PIBT", "TP", ]
MAP_NAMES = [ "warehouse.map", ]
TASK_FREQS = [ 0.2, 0.5, 1, 2, 5, 10 ]
NUM_AGENTS = [ 10, 20, 30, 40, 50 ]
TASK_NUM = 500
REPEAT_NUM = 100

if __name__ == '__main__':
    ins_file = "local/ins.txt"
    result_file = "local/result.txt"

    command = os.path.join("..", "build", "mapd")
    output_dir = os.path.join("..", "..", "data", get_date_str())
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    output_file = os.path.join(output_dir, "result.csv")

    # write header
    with open(output_file, "w") as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow([
            "map_name", "seed", "num_agents", "solver", "task_frequency", "task_num",
            "solved", "service_time", "makespan", "comp_time",
        ])

    for solver, map_name, task_frequency, num_agents, seed in tqdm.tqdm(product(
            SOLVERS, MAP_NAMES, TASK_FREQS, NUM_AGENTS, range(REPEAT_NUM)
    )):
        with open(ins_file, 'w') as f_write:
            f_write.write(f'map_file={map_name}\n')
            f_write.write(f'agents={num_agents}\n')
            f_write.write(f'seed={seed}\n')
            f_write.write(f'task_frequency={task_frequency}\n')
            f_write.write(f'task_num={TASK_NUM}\n')
            f_write.write(f'max_timestep={MAX_TIMESTEP}\n')
            f_write.write(f'max_comp_time={MAX_COMP_TIME}\n')
            f_write.write('specify_pickup_deliv_locs=1\n')

        # solve
        subprocess.run(f"{command} -i {ins_file} -o {result_file} -s {solver} -d -L", shell=True)

        # read result
        res = read_result(result_file)

        # write result to csv
        with open(output_file, "a") as f:
            writer = csv.writer(f, lineterminator='\n')
            writer.writerow([map_name, seed, num_agents, solver,
                             task_frequency, TASK_NUM, *res])
