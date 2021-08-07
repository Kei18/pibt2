#include <pibt.hpp>

#include "gtest/gtest.h"

TEST(PIBT, solve)
{
  auto P = MAPF_Instance("../tests/instances/example.txt");
  auto solver = std::make_unique<PIBT>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
