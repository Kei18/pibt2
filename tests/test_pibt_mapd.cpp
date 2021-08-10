#include <pibt_mapd.hpp>

#include "gtest/gtest.h"

TEST(PIBT_MAPD, solve)
{
  auto P = MAPD_Instance("../tests/instances/test_mapd_pibt_ins.txt");
  auto solver = std::make_unique<PIBT_MAPD>(&P);
  solver->solve();

  ASSERT_TRUE(solver->succeed());
  ASSERT_TRUE(solver->getSolution().validate(&P));
}
