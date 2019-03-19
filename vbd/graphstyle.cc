// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/graphstyle.h"

namespace vbd {

GraphStyle::GraphStyle() = default;

GraphStyle::~GraphStyle() = default;

GraphStyle::StyleValueMap GraphStyle::GetStyle() const {
  return StyleValueMap({});
}

}  // namespace vbd
