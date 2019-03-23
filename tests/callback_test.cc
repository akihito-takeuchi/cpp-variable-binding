// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/vbd.h"

#include <gtest/gtest.h>

TEST(CallbackTest, SingleNode) {
  auto a = vbd::MakeVariable<double>("a", 10.5);
  // Callback that round up the value
  a->SetCallback([](auto v) { return std::ceil(v); });
  ASSERT_EQ(a->Value(), 10.5);
  a->ExecCallback();
  ASSERT_EQ(a->Value(), 11.0);
}

TEST(CallbackTest, TwoNodesCbFirst) {
  auto a = vbd::MakeVariable<double>("a", 10.5);
  // Callback that round up the value
  a->SetCallback([](auto v) { return std::ceil(v); });
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) { return a * 3; });
  ASSERT_EQ(a->Value(), 10.5);
  ASSERT_EQ(b->Value(), 5.0);
  *a = 3.5;
  ASSERT_NE(a->Value(), 3.5);
  ASSERT_EQ(a->Value(), 4.0);
  ASSERT_EQ(b->Value(), 12.0);
}

TEST(CallbackTest, TwoNodesCbSecond) {
  auto a = vbd::MakeVariable<double>("a", 10.5);
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) { return a * 3; });
  // Callback that round up the value
  b->SetCallback([](auto v) { return std::ceil(v); });
  ASSERT_EQ(a->Value(), 10.5);
  ASSERT_EQ(b->Value(), 5.0);
  *a = 3.5;
  ASSERT_EQ(a->Value(), 3.5);
  ASSERT_EQ(b->Value(), 11.0);
}

TEST(CallbackTest, TwoNodesCbBoth) {
  auto a = vbd::MakeVariable<double>("a", 10.5);
  // Callback that round up the value
  a->SetCallback([](auto v) { return std::ceil(v); });
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) { return a * 3; });
  // Callback that round down the value to integer multiples of 5
  b->SetCallback([](auto v) { return (int(v) / 5) * 5; });
  ASSERT_EQ(a->Value(), 10.5);
  ASSERT_EQ(b->Value(), 5.0);
  *a = 3.5;
  ASSERT_EQ(a->Value(), 4.0);
  ASSERT_EQ(b->Value(), 10.0);
}
