#include <getopt.h>

#include <default_params.hpp>
#include <hca.hpp>
#include <iostream>
#include <pibt.hpp>
#include <pibt_plus.hpp>
#include <problem.hpp>
#include <push_and_swap.hpp>
#include <random>
#include <vector>

void printHelp();
std::unique_ptr<MAPF_Solver> getSolver(const std::string solver_name,
                                       MAPF_Instance* P, bool verbose, int argc,
                                       char* argv[]);

int main(int argc, char* argv[])
{
  std::string instance_file = "";
  std::string output_file = DEFAULT_OUTPUT_FILE;
  std::string solver_name;
  bool verbose = false;
  char* argv_copy[argc + 1];
  for (int i = 0; i < argc; ++i) argv_copy[i] = argv[i];

  struct option longopts[] = {
      {"instance", required_argument, 0, 'i'},
      {"output", required_argument, 0, 'o'},
      {"solver", required_argument, 0, 's'},
      {"verbose", no_argument, 0, 'v'},
      {"help", no_argument, 0, 'h'},
      {"time-limit", required_argument, 0, 'T'},
      {"log-short", no_argument, 0, 'L'},
      {"make-scen", no_argument, 0, 'P'},
      {0, 0, 0, 0},
  };
  bool make_scen = false;
  bool log_short = false;
  int max_comp_time = -1;

  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:o:s:vhPT:L", longopts,
                            &longindex)) != -1) {
    switch (opt) {
      case 'i':
        instance_file = std::string(optarg);
        break;
      case 'o':
        output_file = std::string(optarg);
        break;
      case 's':
        solver_name = std::string(optarg);
        break;
      case 'v':
        verbose = true;
        break;
      case 'h':
        printHelp();
        return 0;
      case 'P':
        make_scen = true;
        break;
      case 'L':
        log_short = true;
        break;
      case 'T':
        max_comp_time = std::atoi(optarg);
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapf -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  // set problem
  auto P = MAPF_Instance(instance_file);

  // set max computation time (otherwise, use param in instance_file)
  if (max_comp_time != -1) P.setMaxCompTime(max_comp_time);

  // create scenario
  if (make_scen) {
    P.makeScenFile(output_file);
    return 0;
  }

  // solve
  auto solver = getSolver(solver_name, &P, verbose, argc, argv_copy);
  solver->setLogShort(log_short);
  solver->solve();
  if (solver->succeed() && !solver->getSolution().validate(&P)) {
    std::cout << "error@mapf: invalid results" << std::endl;
    return 0;
  }
  solver->printResult();

  // output result
  solver->makeLog(output_file);
  if (verbose) {
    std::cout << "save result as " << output_file << std::endl;
  }

  return 0;
}

std::unique_ptr<MAPF_Solver> getSolver(const std::string solver_name,
                                       MAPF_Instance* P, bool verbose, int argc,
                                       char* argv[])
{
  std::unique_ptr<MAPF_Solver> solver;
  if (solver_name == "PIBT") {
    solver = std::make_unique<PIBT>(P);
  } else if (solver_name == "HCA") {
    solver = std::make_unique<HCA>(P);
  } else if (solver_name == "PIBT_PLUS") {
    solver = std::make_unique<PIBT_PLUS>(P);
  } else if (solver_name == "PushAndSwap") {
    solver = std::make_unique<PushAndSwap>(P);
  } else {
    std::cout << "warn@mapf: "
              << "unknown solver name, " + solver_name + ", continue by PIBT"
              << std::endl;
    solver = std::make_unique<PIBT>(P);
  }
  solver->setParams(argc, argv);
  solver->setVerbose(verbose);
  return solver;
}

void printHelp()
{
  std::cout << "\nUsage: ./mapf [OPTIONS] [SOLVER-OPTIONS]\n"
            << "\n**instance file is necessary to run MAPF simulator**\n\n"
            << "  -i --instance [FILE_PATH]     instance file path\n"
            << "  -o --output [FILE_PATH]       ouptut file path\n"
            << "  -v --verbose                  print additional info\n"
            << "  -h --help                     help\n"
            << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
            << "  -T --time-limit [INT]         max computation time (ms)\n"
            << "  -L --log-short                use short log"
            << "  -P --make-scen                make scenario file using "
               "random starts/goals"
            << "\n\nSolver Options:" << std::endl;
  // each solver
  PIBT::printHelp();
  HCA::printHelp();
  PIBT_PLUS::printHelp();
  PushAndSwap::printHelp();
}
