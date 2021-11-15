#include "../include/pibt_mapd.hpp"

const std::string PIBT_MAPD::SOLVER_NAME = "PIBT";

PIBT_MAPD::PIBT_MAPD(MAPD_Instance* _P, bool _use_distance_table)
    : MAPD_Solver(_P, _use_distance_table),
      occupied_now(Agents(G->getNodesSize(), nullptr)),
      occupied_next(Agents(G->getNodesSize(), nullptr))
{
  solver_name = PIBT_MAPD::SOLVER_NAME;
}

void PIBT_MAPD::run()
{
  // compare priority of agents
  auto compare = [](Agent* a, const Agent* b) {
    if (a->task != nullptr && b->task == nullptr) return true;
    if (a->task == nullptr && b->task != nullptr) return false;

    if (a->elapsed != b->elapsed) return a->elapsed > b->elapsed;
    // use initial distance
    return a->tie_breaker > b->tie_breaker;
  };
  Agents A;

  // initialize
  for (int i = 0; i < P->getNum(); ++i) {
    Node* s = P->getStart(i);
    Agent* a = new Agent{
        i,                         // id
        s,                         // current location
        nullptr,                   // next location
        s,                         // goal
        0,                         // elapsed
        getRandomFloat(0, 1, MT),  // tie-breaker
        nullptr,                   // task (assigned)
        nullptr,                   // target_task (if free)
    };
    A.push_back(a);
    occupied_now[s->id] = a;
  }
  solution.add(P->getConfigStart());

  auto assign = [&](Agent* a, Task* task) {
    a->task = task;
    a->target_task = nullptr;
    a->task->assigned = true;
    a->g = task->loc_delivery;  // update destination
    info("   ", "assign task-", a->task->id, ": agent-", a->id, ", ",
         a->task->loc_pickup->id, " -> ", a->task->loc_delivery->id);
  };

  // main loop
  while (true) {
    info(" ", "elapsed:", getSolverElapsedTime(),
         ", timestep:", P->getCurrentTimestep() + 1,
         ", open_tasks:", P->getOpenTasks().size(),
         ", closed_tasks:", P->getClosedTasks().size(),
         ", task_num:", P->getTaskNum());

    // target assignment
    {
      Tasks unassigned_tasks;
      for (auto task : P->getOpenTasks()) {
        if (!task->assigned) unassigned_tasks.push_back(task);
      }

      // for log
      Nodes targets;
      Tasks tasks;

      for (auto a : A) {
        // agent is already assigned task
        if (a->task != nullptr) {
          targets.push_back(a->g);
          tasks.push_back(a->task);
          continue;
        }

        // free agent, find min_distance pickup location

        // setup
        a->target_task = nullptr;
        a->g = a->v_now;
        int min_d = P->getG()->getNodesSize();

        std::shuffle(unassigned_tasks.begin(), unassigned_tasks.end(), *MT);
        for (auto itr = unassigned_tasks.begin(); itr != unassigned_tasks.end();
             ++itr) {
          auto task = *itr;
          int d = pathDist(a->v_now, task->loc_pickup);
          if (d == 0) {
            // special case, assign task directly
            assign(a, task);
            unassigned_tasks.erase(itr);  // remove from unassigned_tasks
            break;
          }

          if (d < min_d) {
            min_d = d;
            a->g = task->loc_pickup;
            a->target_task = task;
          }
        }

        targets.push_back(a->g);
        tasks.push_back(a->task);
      }

      hist_targets.push_back(targets);
      hist_tasks.push_back(tasks);
    }

    // planning
    {
      std::sort(A.begin(), A.end(), compare);
      for (auto a : A) {
        // if the agent has next location, then skip
        if (a->v_next == nullptr) {
          // determine its next location
          funcPIBT(a);
        }
      }
    }

    // acting
    Config config(P->getNum(), nullptr);
    for (auto a : A) {
      // clear
      if (occupied_now[a->v_now->id] == a) occupied_now[a->v_now->id] = nullptr;
      occupied_next[a->v_next->id] = nullptr;

      // set next location
      config[a->id] = a->v_next;
      occupied_now[a->v_next->id] = a;
      // update priority
      a->elapsed = (a->v_next == a->g) ? 0 : a->elapsed + 1;
      // reset params
      a->v_now = a->v_next;
      a->v_next = nullptr;

      // update task info
      if (a->task != nullptr) {  // assigned agent
        a->task->loc_current = a->v_now;

        // finish
        if (a->task->loc_current == a->task->loc_delivery) {
          info("   ", "finish task-", a->task->id, ": agent-", a->id, ", ",
               a->task->loc_pickup->id, " -> ", a->task->loc_delivery->id);

          a->task = nullptr;
        }

      } else if (a->target_task != nullptr) {  // free agent
        // assign
        if (a->target_task->loc_pickup == a->v_now) {
          assign(a, a->target_task);
        }
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
    Nodes targets(P->getNum(), nullptr);
    Tasks tasks(P->getNum(), nullptr);
    for (auto a : A) {
      targets[a->id] = a->g;
      tasks[a->id] = a->task;
    }
    hist_targets.push_back(targets);
    hist_tasks.push_back(tasks);
  }

  // memory clear
  for (auto a : A) delete a;
}

bool PIBT_MAPD::funcPIBT(Agent* ai, Agent* aj)
{
  // compare two nodes
  auto compare = [&](Node* const v, Node* const u) {
    int d_v = pathDist(v, ai->g);
    int d_u = pathDist(u, ai->g);
    if (d_v != d_u) return d_v < d_u;
    // tie break
    if (occupied_now[v->id] != nullptr && occupied_now[u->id] == nullptr)
      return false;
    if (occupied_now[v->id] == nullptr && occupied_now[u->id] != nullptr)
      return true;
    // randomize
    return false;
  };

  // get candidates
  Nodes C = ai->v_now->neighbor;
  C.push_back(ai->v_now);
  // randomize
  std::shuffle(C.begin(), C.end(), *MT);
  // sort
  std::sort(C.begin(), C.end(), compare);

  for (auto u : C) {
    // avoid conflicts
    if (occupied_next[u->id] != nullptr) continue;
    if (aj != nullptr && u == aj->v_now) continue;

    // reserve
    occupied_next[u->id] = ai;
    ai->v_next = u;

    auto ak = occupied_now[u->id];
    if (ak != nullptr && ak->v_next == nullptr) {
      if (!funcPIBT(ak, ai)) continue;  // replanning
    }
    // success to plan next one step
    return true;
  }

  // failed to secure node
  occupied_next[ai->v_now->id] = ai;
  ai->v_next = ai->v_now;
  return false;
}

void PIBT_MAPD::printHelp() { printHelpWithoutOption(SOLVER_NAME); }
