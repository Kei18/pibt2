#pragma once
#include <graph.hpp>

static int TASK_ID_CNT = 0;

struct Task {
  int id;
  Node* loc_pickup;
  Node* loc_delivery;
  Node* loc_current;
  int timestep_appear;
  int timestep_finished;
  bool assigned;

  static constexpr int NIL = -1;

  Task(Node* loc_p, Node* loc_d, int t)
    : id(TASK_ID_CNT++),
      loc_pickup(loc_p),
      loc_delivery(loc_d),
      loc_current(loc_p),
      timestep_appear(t),
      timestep_finished(NIL),
      assigned(false) {};

  void println() {
    std::cout << "task-" << id << ", "
              << loc_pickup->id << " -> " << loc_delivery->id << ", "
              << "loc_current=" << loc_current->id << ", "
              << "assigned=" << assigned << ","
              << "t_appear=" << timestep_appear << ", "
              << "t_finished=" << timestep_finished << ", "
              << std::endl;
  }
};

using Tasks = std::vector<Task*>;
