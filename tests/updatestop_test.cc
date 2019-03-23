// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/vbd.h"

#include <gtest/gtest.h>

TEST(UpdateStopTest, TowNodes) {
  auto a = vbd::MakeVariable<double>("a", 10.5);
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) {
        if (a == 5.0)
          throw vbd::Exception();
        return a * 3;
      });
  ASSERT_EQ(a->Value(), 10.5);
  ASSERT_EQ(b->Value(), 5.0);
  *a = 3;
  ASSERT_EQ(a->Value(), 3.0);
  ASSERT_EQ(b->Value(), 9.0);
  ASSERT_THROW(*a = 5, vbd::Exception);
  ASSERT_EQ(a->Value(), 3.0);
  ASSERT_EQ(b->Value(), 9.0);
}
