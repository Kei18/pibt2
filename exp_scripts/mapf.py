import os
import re
import glob
import csv
import subprocess
import tqdm
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

    solved, soc, lb_soc, makespan, lb_makespan, comp_time = -1, -1, -1, -1, -1, -1
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
    return solved, soc, lb_soc, makespan, lb_makespan, comp_time


MAX_TIMESTEP = 1000
# MAX_TIMESTEP = 2000  # for brc202d.map
MAX_COMP_TIME = 30000
SOLVERS = [
    "PIBT",
    "HCA",
    "PIBT_PLUS",
    "PushAndSwap",
    "PushAndSwap --no-compress"
]
MAP_NAMES = [
    # small
    "empty-8-8.map",
    "random-32-32-20.map",

    # large
    "empty-48-48.map",
    "random-64-64-20.map",
    "warehouse-20-40-10-2-2.map",
    "Berlin_1_256.map",
    "Paris_1_256.map",
    "den520d.map",
    "ost003d.map",

    # set step -> 2000
    # "brc202d.map",
]

if __name__ == '__main__':
    r_scen = re.compile(r"\d+\t.+\.map\t\d+\t\d+\t(\d+)\t(\d+)\t(\d+)\t(\d+)\t.+")
    r_scen_file= re.compile(r"benchmark/(.+)\-random\-(\d+)\.scen")

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
            "solved", "soc", "lb_soc", "makespan", "lb_makespan", "comp_time",
        ])

    for solver in SOLVERS:
        for scen_file in tqdm.tqdm(glob.glob("benchmark/*.scen")):
            # extract map name and scenario number
            m = re.match(r_scen_file, scen_file)
            if m:
                map_name = f"{str(m.group(1))}.map"
                scen_num = int(m.group(2))

            if map_name not in MAP_NAMES:
               continue

            # extract starts and goals
            starts_goals_str = []
            with open(scen_file, 'r') as f:
                for row in f:
                    m_scen = re.match(r_scen, row)
                    if m_scen:
                        y_s = int(m_scen.group(1))
                        x_s = int(m_scen.group(2))
                        y_g = int(m_scen.group(3))
                        x_g = int(m_scen.group(4))
                        starts_goals_str.append(f"{y_s},{x_s},{y_g},{x_g}")

            print(f"solver={solver}, map_name={map_name}, scen_num={scen_num},"
                  f" max_num_agents={len(starts_goals_str)}")

            # increments number of agents
            for num_agents in range(10, len(starts_goals_str) + 1, 10):
                # create instance
                with open(ins_file, 'w') as f_write:
                    f_write.write(f'map_file={map_name}\n')
                    f_write.write(f'agents={num_agents}\n')
                    f_write.write(f'seed=0\n')
                    f_write.write(f'random_problem=0\n')
                    f_write.write(f'max_timestep={MAX_TIMESTEP}\n')
                    f_write.write(f'max_comp_time={MAX_COMP_TIME}\n')
                    config = "\n".join(starts_goals_str[:num_agents])
                    f_write.write(f'{config}\n')

                # solve
                subprocess.run(f"{command} -i {ins_file} -o {result_file} -s {solver} -L", shell=True)

                # read result
                res = read_result(result_file)

                # write result to csv
                with open(output_file, "a") as f:
                    writer = csv.writer(f, lineterminator='\n')
                    writer.writerow([map_name, scen_num, num_agents, solver, *res])
