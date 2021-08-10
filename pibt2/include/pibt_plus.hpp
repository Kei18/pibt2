/*
 * Implementation of PIBT+
 */

#pragma once
#include "solver.hpp"

class PIBT_PLUS : public MAPF_Solver
{
private:
  // time required to complement plan, default zero
  double comp_time_complement;

public:
  static const std::string SOLVER_NAME;

  void run();

public:
  PIBT_PLUS(MAPF_Instance* _P);
  ~PIBT_PLUS() {}

  void makeLog(const std::string& logfile);
  static void printHelp();
};
