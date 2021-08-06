#include <push_and_swap.hpp>

#include "gtest/gtest.h"

TEST(PushAndSwap, ins_tree)
{
  Problem P = Problem("../tests/instances/tree.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_corners)
{
  Problem P = Problem("../tests/instances/corners.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_tunnel)
{
  Problem P = Problem("../tests/instances/tunnel.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_string)
{
  Problem P = Problem("../tests/instances/string.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_loop_chain)
{
  Problem P = Problem("../tests/instances/loop-chain.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}

TEST(PushAndSwap, ins_connector)
{
  Problem P = Problem("../tests/instances/connector.txt");
  auto solver = std::make_unique<PushAndSwap>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
