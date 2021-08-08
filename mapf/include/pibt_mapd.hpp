#pragma once
#include "solver.hpp"

class PIBT_MAPD : public MAPD_Solver
{
public:
  static const std::string SOLVER_NAME;

private:
  // PIBT agent
  struct Agent {
    int id;
    Node* v_now;        // current location
    Node* v_next;       // next location
    Node* g;            // goal
    int elapsed;        // eta
    float tie_breaker;  // epsilon, tie-breaker
    Task* task;
    Task* target_task;
  };
  using Agents = std::vector<Agent*>;

  // <node-id, agent>, whether the node is occupied or not
  // work as reservation table
  Agents occupied_now;
  Agents occupied_next;

  // result of priority inheritance: true -> valid, false -> invalid
  bool funcPIBT(Agent* ai);
  // plan next node
  Node* planOneStep(Agent* a);
  // chose one node from candidates, used in planOneStep
  Node* chooseNode(Agent* a);

  // main
  void run();

public:
  PIBT_MAPD(MAPD_Instance* _P);
  ~PIBT_MAPD() {}

  static void printHelp();
};
