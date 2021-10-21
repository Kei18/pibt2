import os
import re
import glob
import csv
import subprocess
from datetime import datetime, timedelta, timezone

def get_date_str():
    return datetime.now(timezone(timedelta(hours=+9), "JST")).strftime("%Y-%m-%d-%H-%M-%S")

def read_result(filename):
    r_solved= re.compile(r"solved=(\d+)")
    r_soc = re.compile(r"soc=(\d+)")
    r_lb_soc = re.compile(r"lb_soc=(\d+)")
    r_makespan = re.compile(r"makespan=(\d+)")
    r_lb_makespan = re.compile(r"lb_makespan=(\d+)")
    r_comp_time = re.compile(r"comp_time=(\d+)")
    r_preprocessing_comp_time = re.compile(r"preprocessing_comp_time=(\d+)")

    solved, soc, lb_soc, makespan, lb_makespan, comp_time, preprocessing_comp_time = -1, -1, -1, -1, -1, -1, -1
    with open(result_file, 'r') as f_read:
        for row in f_read:
            m = re.match(r_solved, row)
            if m:
                solved = int(m.group(1))
            m = re.match(r_soc, row)
            if m:
                soc = int(m.group(1))
            m = re.match(r_lb_soc, row)
            if m:
                lb_soc = int(m.group(1))
            m = re.match(r_makespan, row)
            if m:
                makespan = int(m.group(1))
            m = re.match(r_lb_makespan, row)
            if m:
                lb_makespan = int(m.group(1))
            m = re.match(r_comp_time, row)
            if m:
                comp_time = int(m.group(1))
            m = re.match(r_preprocessing_comp_time, row)
            if m:
                preprocessing_comp_time = int(m.group(1))

    return solved, soc, lb_soc, makespan, lb_makespan, comp_time, preprocessing_comp_time


MAX_TIMESTEP = 100
MAX_COMP_TIME = 300000
SOLVER = "PIBT"
MAP_NAME = "orz900d.map"
ARR_NUM_AGENTS = [ 2000, 4000, 6000, 8000, 10000, ]
REPETITION = 100

if __name__ == '__main__':
    ins_file = "local/ins.txt"
    result_file = "local/result.txt"
    command = os.path.join("..", "build", "mapf")
    output_dir = os.path.join("..", "..", "data", get_date_str())
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    output_file = os.path.join(output_dir, "result.csv")

    # write header
    with open(output_file, "w") as f:
        writer = csv.writer(f, lineterminator='\n')
        writer.writerow([
            "map_name", "scen_num", "num_agents", "solver",
            "solved", "soc", "lb_soc", "makespan", "lb_makespan",
            "comp_time", "preprocessing_comp_time",
        ])

    for num_agents in ARR_NUM_AGENTS:
        for seed in range(REPETITION):
            # create instance
            with open(ins_file, 'w') as f_write:
                f_write.write(f'map_file={MAP_NAME}\n')
                f_write.write(f'agents={num_agents}\n')
                f_write.write(f'seed={seed}\n')
                f_write.write(f'random_problem=1\n')
                f_write.write(f'max_timestep={MAX_TIMESTEP}\n')
                f_write.write(f'max_comp_time={MAX_COMP_TIME}\n')

            # solve
            subprocess.run(f"{command} -i {ins_file} -o {result_file} -s {SOLVER} -L", shell=True)

            # read result
            res = read_result(result_file)

            # write result to csv
            with open(output_file, "a") as f:
                writer = csv.writer(f, lineterminator='\n')
                writer.writerow([MAP_NAME, seed, num_agents, SOLVER, *res])
