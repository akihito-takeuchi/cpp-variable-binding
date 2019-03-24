// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbdqt/bindqobject.h"

#include <QApplication>
#include <QLineEdit>
#include <gtest/gtest.h>

TEST(BindQtTest, SimpleWidget) {
  int argc = 0;
  char** argv = nullptr;
  QApplication app(argc, argv);
  auto a = vbd::MakeVariable<QString>("Test string");
  QLineEdit lbl(a->Value());
  vbd::qt::BindQObject(a, &lbl, "text", &QLineEdit::textChanged);
  ASSERT_EQ(a->Value(), QString("Test string"));
  ASSERT_EQ(lbl.text(), QString("Test string"));
  *a = "Updated";
  ASSERT_EQ(a->Value(), QString("Updated"));
  ASSERT_EQ(lbl.text(), QString("Updated"));
  lbl.setText("Modified");
  ASSERT_EQ(a->Value(), QString("Modified"));
  ASSERT_EQ(lbl.text(), QString("Modified"));
}
