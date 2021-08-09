#include "../include/pibt_plus.hpp"

#include <fstream>
#include <memory>

#include "../include/pibt.hpp"
#include "../include/push_and_swap.hpp"

const std::string PIBT_PLUS::SOLVER_NAME = "PIBT_PLUS";

PIBT_PLUS::PIBT_PLUS(MAPF_Instance* _P) : MAPF_Solver(_P)
{
  solver_name = SOLVER_NAME;
  comp_time_complement = 0;
}

void PIBT_PLUS::run()
{
  // find lower bound of makespan
  int LB_makespan = 0;
  for (int i = 0; i < P->getNum(); ++i) {
    if (pathDist(i) > LB_makespan) LB_makespan = pathDist(i);
  }

  // solve by PIBT
  auto _P = MAPF_Instance(P, P->getConfigStart(), P->getConfigGoal(),
                          max_comp_time, LB_makespan);
  auto init_solver = std::make_unique<PIBT>(&_P);
  init_solver->setDistanceTable(
      (distance_table_p == nullptr) ? &distance_table : distance_table_p);
  info(" ", "run PIBT until timestep", LB_makespan);
  init_solver->solve();
  solution = init_solver->getSolution();

  if (init_solver->succeed()) {  // PIBT success
    solved = true;

  } else {  // PIBT failed

    auto t_complement = Time::now();

    // solved by Push & Swap
    auto _Q = MAPF_Instance(P, solution.last(), P->getConfigGoal(),
                            getRemainedTime(), max_timestep - LB_makespan);
    auto comp_solver = std::make_shared<PushAndSwap>(&_Q);

    // set solver options
    comp_solver->setDistanceTable(
        (distance_table_p == nullptr) ? &distance_table : distance_table_p);

    info(" ", "elapsed:", getSolverElapsedTime(), ", use",
         comp_solver->getSolverName(), "to complement the remain");

    // solve
    comp_solver->solve();
    solution += comp_solver->getSolution();
    if (comp_solver->succeed()) solved = true;

    comp_time_complement = getElapsedTime(t_complement);
  }
}

void PIBT_PLUS::printHelp()
{
  std::cout << PIBT_PLUS::SOLVER_NAME << "\n"
            << "  (none)" << std::endl;
}

void PIBT_PLUS::makeLog(const std::string& logfile)
{
  std::ofstream log;
  log.open(logfile, std::ios::out);
  makeLogBasicInfo(log);

  // print additional info
  log << "comp_time_complement=" << comp_time_complement << "\n";

  makeLogSolution(log);
  log.close();
}
