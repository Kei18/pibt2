#include "../include/tp.hpp"

const std::string TP::SOLVER_NAME = "TP";

TP::TP(MAPD_Instance* _P, bool _use_distance_table)
    : MAPD_Solver(_P, _use_distance_table)
{
  solver_name = TP::SOLVER_NAME;
}

void TP::run()
{
  std::vector<Path> TOKEN(P->getNum());
  Agents A;

  // initialize conflict table
  {
    CONFLICT_TABLE.push_back(std::vector<int>(G->getNodesSize(), NIL));
  }

  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Agent* a = new Agent{
        i,        // id
        s,        // current location
        nullptr,  // task
        false     // load_task
    };
    A.push_back(a);
    // line 2, initialize
    TOKEN[i].push_back(s);
  }
  solution.add(P->getConfigStart());

  // line 3
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", timestep:", P->getCurrentTimestep() + 1,
         ", open_tasks:", P->getOpenTasks().size(),
         ", closed_tasks:", P->getClosedTasks().size(),
         ", task_num:", P->getTaskNum());

    // line 4, get unassigned tasks
    Tasks unassigned_tasks;
    for (auto task : P->getOpenTasks()) {
      if (!task->assigned) unassigned_tasks.push_back(task);
    }

    Config targets(P->getNum());        // for history
    Tasks tasks(P->getNum(), nullptr);  // for history

    for (auto a : A) {  // line 5
      if ((int)TOKEN[a->id].size() - 1 != P->getCurrentTimestep()) {
        if (a->task != nullptr) {  // assigned agent
          tasks[a->id] = a->task;
          targets[a->id] =
              a->load_task ? a->task->loc_delivery : a->task->loc_pickup;
        } else {  // agents moving to endpoints
          targets[a->id] = *(TOKEN[a->id].end() - 1);
        }
        continue;
      }

      // line 7, pickup assignable tasks
      Tasks selected_tasks;
      for (auto task : unassigned_tasks) {
        if (task->assigned) continue;  // line 11

        bool cond = std::all_of(A.begin(), A.end(), [&](Agent* b) {
          if (b == a) return true;
          auto v = *(TOKEN[b->id].end() - 1);
          return v != task->loc_pickup && v != task->loc_delivery;
        });
        if (cond) selected_tasks.push_back(task);
      }

      if (!selected_tasks.empty()) {  // line 8
        // line 9, pickup one task
        auto task =
            *std::min_element(selected_tasks.begin(), selected_tasks.end(),
                              [&](Task* task1, Task* task2) {
                                return pathDist(a->v_now, task1->loc_pickup) <
                                       pathDist(a->v_now, task2->loc_pickup);
                              });

        // line 10, assign
        a->task = task;
        task->assigned = true;
        tasks[a->id] = task;
        targets[a->id] = task->loc_pickup;
        info("   ", "assign task-", task->id, ": agent-", a->id, "at",
             a->v_now->id, ",", task->loc_pickup->id, " -> ",
             task->loc_delivery->id);

        // line 12
        updatePath1(a->id, task, TOKEN);

        // line 13
      } else if (std::all_of(unassigned_tasks.begin(), unassigned_tasks.end(),
                             [&](Task* task) {
                               return task->assigned ||
                                      task->loc_delivery != a->v_now;
                             })) {
        // line 14
        TOKEN[a->id].push_back(a->v_now);

        // update conflict table
        {
          while (CONFLICT_TABLE.size() - 1 < TOKEN[a->id].size() - 1) {
            CONFLICT_TABLE.push_back(std::vector<int>(G->getNodesSize(), NIL));
          }
          CONFLICT_TABLE[P->getCurrentTimestep() + 1][a->v_now->id] = a->id;
        }

        targets[a->id] = a->v_now;

        // line 15
      } else {
        // line 16
        updatePath2(a->id, TOKEN, unassigned_tasks);

        targets[a->id] = *(TOKEN[a->id].end() - 1);
      }
    }

    hist_targets.push_back(targets);
    hist_tasks.push_back(tasks);

    Config config(P->getNum(), nullptr);
    for (auto a : A) {
      auto v_next = TOKEN[a->id][P->getCurrentTimestep() + 1];
      config[a->id] = v_next;
      a->v_now = v_next;

      if (a->task != nullptr) {
        // update task location
        if (a->load_task) {
          a->task->loc_current = a->v_now;
        }

        // pickup loc -> delivery loc
        if (a->load_task && a->v_now == a->task->loc_delivery &&
            P->getCurrentTimestep() + 1 == (int)TOKEN[a->id].size() - 1) {
          // finish task
          info("   ", "finish task-", a->task->id, ": agent-", a->id, ", ",
               a->task->loc_pickup->id, " -> ", a->task->loc_delivery->id);
          a->task = nullptr;
          a->load_task = false;

          // start loc -> pickup loc
        } else if (!a->load_task && a->v_now == a->task->loc_pickup) {
          a->load_task = true;
        }

      } else {
        // free agent
        targets[a->id] = *(TOKEN[a->id].end() - 1);
      }
    }

    // update plan
    solution.add(config);

    // increment timestep
    P->update();

    // failure
    if (P->getCurrentTimestep() >= max_timestep || overCompTime()) {
      solved = false;
      break;
    }

    // success
    if ((int)P->getClosedTasks().size() == P->getTaskNum()) {
      solved = true;
      break;
    }
  }

  // align target history and plan
  {
    Nodes targets;
    Tasks tasks;
    for (auto a : A) {
      targets.push_back(*(TOKEN[a->id].end() - 1));
      tasks.push_back(a->task);
    }
    hist_targets.push_back(targets);
    hist_tasks.push_back(tasks);
  }

  // memory clear
  for (auto a : A) delete a;
}

void TP::updatePath1(int i, Task* task, std::vector<Path>& TOKEN)
{
  info("   ", "updatePath1, agent-", i);

  // find path: loc -> pickup location
  updatePath(i, task->loc_pickup, TOKEN);
  // find path: pickup location -> delivery location
  updatePath(i, task->loc_delivery, TOKEN);
}

void TP::updatePath2(int i, std::vector<Path>& TOKEN, Tasks& unassigned_tasks)
{
  info("   ", "updatePath2, agent-", i);

  Node* target;
  Node* loc = TOKEN[i][P->getCurrentTimestep()];
  int estimated_cost = G->getNodesSize();
  for (auto p : P->getEndpoints()) {
    /*
     * the delivery locations of all tasks in the task set
     * are different from the chosen endpoint
     */
    bool cond1 = std::all_of(
        unassigned_tasks.begin(), unassigned_tasks.end(),
        [&](Task* task) { return task->assigned || task->loc_delivery != p; });
    if (!cond1) continue;

    /*
     * no path of other agents in the token ends in the chosen endpoint
     */
    bool cond2 = true;
    for (int j = 0; j < P->getNum(); ++j) {
      if (j == i) continue;
      if (*(TOKEN[j].end() - 1) == p) {
        cond2 = false;
        break;
      }
    }
    if (!cond2) continue;

    // update target
    int d = pathDist(loc, p);
    if (d < estimated_cost) {
      target = p;
      estimated_cost = d;
    }
  }

  if (target == nullptr) halt("this is not well-formed instance");

  // find path: loc -> target
  updatePath(i, target, TOKEN);
}

void TP::updatePath(int i, Node* g, std::vector<Path>& TOKEN)
{
  auto s = *(TOKEN[i].end() - 1);
  const int current_timestep = (int)TOKEN[i].size() - 1;

  // someone may use g
  int max_constraint_time = current_timestep;
  for (int j = 0; j < P->getNum(); ++j) {
    for (int t = (int)TOKEN[j].size() - 1; t > max_constraint_time; --t) {
      if (j != i && TOKEN[j][t] == g) {
        max_constraint_time = std::max(t, max_constraint_time);
        break;
      }
    }
  }

  AstarHeuristics fValue = [&](AstarNode* n) {
    return n->g + pathDist(n->v, g);
  };

  CheckAstarFin checkAstarFin = [&](AstarNode* n) {
    return n->v == g && n->g + current_timestep > max_constraint_time;
  };

  std::vector<int> token_endpoints(G->getNodesSize(), NIL);
  for (int j = 0; j < P->getNum(); ++j) {
    if (j == i) continue;
    token_endpoints[(*(TOKEN[j].end() - 1))->id] = j;
  }

  CheckInvalidAstarNode checkInvalidAstarNode = [&](AstarNode* m) {
    auto t = current_timestep + m->g;
    // avoid endpoints
    auto k = token_endpoints[m->v->id];
    if (k != NIL && (int)TOKEN[k].size() - 1 < t) return true;
    // avoid conflicts
    if ((int)CONFLICT_TABLE.size() - 1 >= t) {
      // check vertex conflicts
      if (CONFLICT_TABLE[t][m->v->id] != NIL) return true;
      // check swap conflicts
      if (CONFLICT_TABLE[t][m->p->v->id] != NIL &&
          CONFLICT_TABLE[t - 1][m->v->id] == CONFLICT_TABLE[t][m->p->v->id])
        return true;
    }

    return false;
  };

  // get path
  auto path = getPathBySpaceTimeAstar(s, g, fValue, compareAstarNodeBasic,
                                      checkAstarFin, checkInvalidAstarNode,
                                      getRemainedTime());

  if (path.empty()) halt("failed");

  // update conflict table
  while (CONFLICT_TABLE.size() - 1 < current_timestep + path.size() - 1) {
    CONFLICT_TABLE.push_back(std::vector<int>(G->getNodesSize(), NIL));
  }

  // update TOKEN
  for (int _t = 1; _t < (int)path.size(); ++_t) {
    TOKEN[i].push_back(path[_t]);

    // update conflict table
    CONFLICT_TABLE[current_timestep + _t][path[_t]->id] = i;
  }
}

void TP::printHelp() { printHelpWithoutOption(SOLVER_NAME); }
