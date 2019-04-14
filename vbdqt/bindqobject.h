// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/vbd.h"

#include <QObject>
#include <QVariant>

namespace vbd {

class qt {
 public:
  template<typename T, typename QObjType, typename FuncType>
  static void BindObjectProperty(const VPtr<T>& v, QObjType* obj,
                                 const char* property_name,
                                 FuncType signal) {
    v->SetCallback([obj, property_name](const T& value) {
        obj->setProperty(property_name, value);
        return value;
      });
    QObject::connect(obj, signal, [v, obj, property_name]() {
        *v = qvariant_cast<T>(obj->property(property_name));
      });
  }
};

}  // namespace vbd
