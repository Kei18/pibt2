#include <tp.hpp>

#include "gtest/gtest.h"

TEST(TP, solve)
{
  auto P = MAPD_Instance("../tests/instances/tp_mapd.txt");
  auto solver = std::make_unique<TP>(&P);
  solver->setVerbose(true);
  solver->solve();

  // ASSERT_TRUE(solver->succeed());
  // ASSERT_TRUE(solver->getSolution().validate(&P));
}
