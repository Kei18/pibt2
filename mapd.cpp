#include <getopt.h>

#include <default_params.hpp>
#include <iostream>
#include <pibt_mapd.hpp>
#include <problem.hpp>
#include <random>
#include <tp.hpp>
#include <vector>

void printHelp();
std::unique_ptr<MAPD_Solver> getSolver(const std::string solver_name,
                                       MAPD_Instance* P, bool verbose, int argc,
                                       char* argv[], bool use_distance_table);

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
      {"use-distance-table", no_argument, 0, 'd'},
      {0, 0, 0, 0},
  };
  bool log_short = false;
  int max_comp_time = -1;
  bool use_distance_table = false;

  // command line args
  int opt, longindex;
  opterr = 0;  // ignore getopt error
  while ((opt = getopt_long(argc, argv, "i:o:s:vhT:Ld", longopts,
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
      case 'L':
        log_short = true;
        break;
      case 'T':
        max_comp_time = std::atoi(optarg);
        break;
      case 'd':
        use_distance_table = true;
        break;
      default:
        break;
    }
  }

  if (instance_file.length() == 0) {
    std::cout << "specify instance file using -i [INSTANCE-FILE], e.g.,"
              << std::endl;
    std::cout << "> ./mapd -i ../instance/sample.txt" << std::endl;
    return 0;
  }

  // set problem
  auto P = MAPD_Instance(instance_file);

  // set max computation time (otherwise, use param in instance_file)
  if (max_comp_time != -1) P.setMaxCompTime(max_comp_time);

  // solve
  auto solver =
      getSolver(solver_name, &P, verbose, argc, argv_copy, use_distance_table);
  solver->setLogShort(log_short);
  solver->solve();
  if (solver->succeed() && !solver->getSolution().validate(&P)) {
    std::cout << "error@mapd: invalid results" << std::endl;
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

std::unique_ptr<MAPD_Solver> getSolver(const std::string solver_name,
                                       MAPD_Instance* P, bool verbose, int argc,
                                       char* argv[], bool use_distance_table)
{
  std::unique_ptr<MAPD_Solver> solver;
  if (solver_name == "PIBT") {
    solver = std::make_unique<PIBT_MAPD>(P, use_distance_table);
  } else if (solver_name == "TP") {
    solver = std::make_unique<TP>(P, use_distance_table);
  } else {
    std::cout << "warn@mapd: "
              << "unknown solver name, " + solver_name + ", continue by PIBT"
              << std::endl;
    solver = std::make_unique<PIBT_MAPD>(P, use_distance_table);
  }
  solver->setParams(argc, argv);
  solver->setVerbose(verbose);
  return solver;
}

void printHelp()
{
  std::cout
      << "\nUsage: ./mapd [OPTIONS] [SOLVER-OPTIONS]\n"
      << "\n**instance file is necessary to run MAPD simulator**\n\n"
      << "  -i --instance [FILE_PATH]     instance file path\n"
      << "  -o --output [FILE_PATH]       ouptut file path\n"
      << "  -v --verbose                  print additional info\n"
      << "  -h --help                     help\n"
      << "  -d --use-distance-table       use pre-computed distance table\n"
      << "  -s --solver [SOLVER_NAME]     solver, choose from the below\n"
      << "  -T --time-limit [INT]         max computation time (ms)\n"
      << "  -L --log-short                use short log\n"
      << "\nSolver Options:" << std::endl;
  // each solver
  PIBT_MAPD::printHelp();
  TP::printHelp();
}
