// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#pragma once

#include <string>
#include <map>

namespace vbd {

class GraphStyle {
 public:
  using StyleValueMap = std::map<std::string, std::string>;
  GraphStyle();
  virtual ~GraphStyle();
  virtual StyleValueMap GetStyle() const;
};

// Style example

// StyleValueMap style_values = {
//   {"graph.charset", "UTF-8"},
//   {"graph.fontsize", "18"},
//   {"node.fontsize", "14"},
//   {"edge.color", "black"}};
//

}  // namespace vbd
