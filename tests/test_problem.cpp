#include <plan.hpp>
#include <problem.hpp>

#include "gtest/gtest.h"

TEST(MAPF_Instance, loading)
{
  auto P = MAPF_Instance("../tests/instances/toy_problem.txt");
  Graph* G = P.getG();

  ASSERT_EQ(P.getNum(), 2);
  ASSERT_EQ(P.getMaxTimestep(), 10);
  ASSERT_EQ(P.getMaxCompTime(), 1000);

  Config starts = P.getConfigStart();
  ASSERT_EQ(starts.size(), 2);
  ASSERT_EQ(starts[0], G->getNode(0, 0));
  ASSERT_EQ(starts[1], G->getNode(1, 1));

  Config goals = P.getConfigGoal();
  ASSERT_EQ(goals.size(), 2);
  ASSERT_EQ(goals[0], G->getNode(1, 0));
  ASSERT_EQ(goals[1], G->getNode(0, 1));
}

TEST(MAPF_Instance, plan)
{
  auto P = MAPF_Instance("../tests/instances/toy_problem.txt");
  Graph* G = P.getG();

  Plan plan0;
  Config c0_0 = {G->getNode(0, 0), G->getNode(1, 1)};
  Config c0_1 = {G->getNode(1, 0), G->getNode(0, 1)};
  plan0.add(c0_0);
  plan0.add(c0_1);
  ASSERT_TRUE(plan0.validate(&P));

  Plan plan1;
  Config c1_0 = {G->getNode(0, 0), G->getNode(1, 1)};
  Config c1_1 = {G->getNode(0, 1), G->getNode(1, 0)};
  plan1.add(c1_0);
  plan1.add(c1_1);
  ASSERT_FALSE(plan1.validate(&P));
}

TEST(MAPD_Instance, load)
{
  auto P = MAPD_Instance("../tests/instances/toy_mapd.txt");

  auto init_pos = P.getStart(0)->pos;

  ASSERT_TRUE(init_pos.x == 1 && init_pos.y == 1);
  ASSERT_TRUE(P.getTaskFrequency() == 1);
  ASSERT_TRUE(P.getTaskNum() == 10);
  ASSERT_TRUE(P.getCurrentTimestep() == 0);
  ASSERT_TRUE(int(P.getOpenTasks().size()) == 1);
  ASSERT_TRUE(int(P.getClosedTasks().size()) == 0);

  for (int t = 0; t < P.getTaskNum() + 1; ++t) {
    // update
    P.update();
    ASSERT_TRUE(P.getCurrentTimestep() == t + 1);
    ASSERT_TRUE(int(P.getOpenTasks().size()) <= P.getTaskNum());
  }
}
