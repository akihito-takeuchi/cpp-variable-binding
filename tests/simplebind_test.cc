// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/vbd.h"

#include <fstream>
#include <gtest/gtest.h>

TEST(SimpleBind, Unidirectional) {
  auto a = vbd::MakeVariable<double>("a", 10);
  ASSERT_EQ(a->Value(), 10.0);
  ASSERT_FALSE(a->IsReadonly());
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) { return a * 3; });
  ASSERT_TRUE(b->IsReadonly());
  ASSERT_FALSE(a->IsReadonly());
  ASSERT_EQ(b->Value(), 5.0);
  *a = 8;
  ASSERT_EQ(b->Value(), 24.0);
  ASSERT_THROW(*b = 1, vbd::Exception);
  std::ofstream out("unidirectional.dot");
  vbd::CreateGraphViz("unidirectional", out);
}

TEST(SimpleBind, Bidirectional) {
  auto a = vbd::MakeVariable<double>("a", 10);
  ASSERT_EQ(a->Value(), 10.0);
  ASSERT_FALSE(a->IsReadonly());
  auto b = vbd::MakeVariable<double, double>(
      "b", 5, a, [](auto /*self*/,
                    auto a) { return a * 3; });
  ASSERT_TRUE(b->IsReadonly());
  ASSERT_FALSE(a->IsReadonly());
  a->Listen(b, [](auto /*self*/, auto b) { return b / 3;});
  ASSERT_FALSE(a->IsReadonly());
  ASSERT_EQ(b->Value(), 5.0);

  *a = 8;
  ASSERT_EQ(a->Value(), 8.0);
  ASSERT_EQ(b->Value(), 24.0);

  *b = 30;
  ASSERT_EQ(a->Value(), 10.0);
  ASSERT_EQ(b->Value(), 30.0);
  std::ofstream out("bidirectional.dot");
  vbd::CreateGraphViz("bidirectional", out);
}

TEST(SimpleBind, BindError) {
  auto a = vbd::MakeVariable<double>(10);
  ASSERT_EQ(a->Value(), 10.0);
  ASSERT_FALSE(a->IsReadonly());
  auto b = vbd::MakeVariable<double, double>(
      5, a, [](auto /*self*/,
               auto a) { return a * 3; });
  // b is already listening to a. Should throw exception
  ASSERT_THROW(b->Listen(a, [](auto, auto) { return 0; }),
               vbd::Exception);
}

TEST(SimpleBind, ThreeNodes) {
  auto a = vbd::MakeVariable<double>(10);
  ASSERT_EQ(a->Value(), 10.0);
  ASSERT_FALSE(a->IsReadonly());
  auto b = vbd::MakeVariable<double, double>(
      5, a, [](auto /*self*/,
               auto a) { return a * 3; });
  ASSERT_FALSE(a->IsReadonly());
  ASSERT_TRUE(b->IsReadonly());
  ASSERT_EQ(b->Value(), 5.0);
  auto c = vbd::MakeVariable<double, double>(
      1, b, [](auto /*self*/,
               auto b) { return b * 4; });
  ASSERT_FALSE(a->IsReadonly());
  ASSERT_TRUE(b->IsReadonly());
  ASSERT_TRUE(c->IsReadonly());
  ASSERT_EQ(c->Value(), 1.0);
  *a = 2;
  // a = 2
  // b = a * 3
  // c = b * 4
  ASSERT_EQ(a->Value(), 2.0);
  ASSERT_EQ(b->Value(), 6.0);
  ASSERT_EQ(c->Value(), 24.0);
  std::ofstream out("threenodes.dot");
  vbd::CreateGraphViz("threenodes", out);
}
