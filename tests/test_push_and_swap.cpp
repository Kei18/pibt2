#include <push_and_swap.hpp>

#include "gtest/gtest.h"

TEST(PushAndSwap, ins_tree)
{
  auto P = MAPF_Instance("../tests/instances/tree.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_corners)
{
  auto P = MAPF_Instance("../tests/instances/corners.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_tunnel)
{
  auto P = MAPF_Instance("../tests/instances/tunnel.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_string)
{
  auto P = MAPF_Instance("../tests/instances/string.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_loop_chain)
{
  auto P = MAPF_Instance("../tests/instances/loop-chain.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_connector)
{
  auto P = MAPF_Instance("../tests/instances/connector.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
