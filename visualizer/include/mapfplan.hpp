#pragma once
#include "../../third_party/grid-pathfinding/graph/include/graph.hpp"

using Config = std::vector<Node*>;
using Configs = std::vector<Config>;

struct MAPFPlan {
  int num_agents;         // number of agents
  Grid* G;                // grid
  std::string solver;     // solver name
  bool solved;            // success or not
  int soc;                // sum of cost
  int makespan;           // makespan
  int comp_time;          // computation time
  Config config_s;        // start configuration
  Config config_g;        // goal configuration
  Configs transitions;    // plan
  // for MAPD
  float service_time;     // service_time
  Configs targets;        // targets
  std::vector<std::vector<bool>> assigned;  // as

  ~MAPFPlan() { delete G; }
};
