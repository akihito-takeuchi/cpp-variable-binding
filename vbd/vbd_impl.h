// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#pragma once

#include <mutex>
#include <forward_list>
#include <unordered_map>

namespace vbd {

enum class Access {
  kNotInitialized = 0,
  kReadOnly = 1,
  kWritable = 2
};

class VbdEngine;
class FunctionNode;
using FunctionPtr = std::shared_ptr<FunctionNode>;
using VoidFuncType = std::function<void()>;

class VariableBase {
 public:
  virtual ~VariableBase();
  bool IsReadonly() const;
  template<typename T>
  T operator=(const T& new_value);
  template<typename T>
  T Get() const;
  virtual void Legalize() = 0;
  void SetName(const std::string& name) { name_ = name; }
  std::string Name() const { return name_; }
 protected:
  VariableBase();
  VariableBase(const std::string& name);
  template<typename T>
  void SetValue(const T& new_value);
  void RegisterAssignOperation(VariableBase* variable,
                               const VoidFuncType& func);
  void UnregisterAssignOperation(VariableBase* variable);
  void UpdateGroupID(VariableBase* listener,
                     VariableBase* broadcaster);
  bool HasPath(const VariableBase* listener,
               const VariableBase* broadcaster) const;
  FunctionPtr CreateFunctionNode(const VoidFuncType& func,
                                 VariableBase* listener,
                                 const std::vector<VariableBase*>&& broadcaster);
  std::vector<FunctionPtr> to_listener_funcs_;
  std::vector<FunctionPtr> to_broadcaster_funcs_;
  std::string name_;
  boost::any value_;
  unsigned int group_id_;
  Access access_;
  friend class VbdEngine;
};

template<typename T>
T VariableBase::operator=(const T& new_value) {
  if (IsReadonly())
    throw Exception("Variable is readonly");
  SetValue(new_value);
  return new_value;
}

template<typename T>
void VariableBase::SetValue(const T& new_value) {
  auto org_value = boost::any_cast<T>(value_);
  if (new_value != org_value)
    RegisterAssignOperation(
        this, [this, new_value = new_value](){
          value_ = new_value;
        });
  else
    UnregisterAssignOperation(this);
}

template<typename T>
T VariableBase::Get() const {
  return boost::any_cast<T>(value_);
}

template<typename T>
class Variable : public VariableBase {
 public:
  using ValueType = T;
  Variable() : VariableBase() {};
  Variable(const T& initial_value) : VariableBase() {
    value_ = initial_value;
  };
  Variable(const std::string& name, const T& initial_value) : VariableBase(name) {
    value_ = initial_value;
  }
  T operator=(const T& new_value) {
    return VariableBase::operator=(new_value);
  }
  T Value() const {
    return VariableBase::Get<T>();
  }
  void Legalize() {
    if (legalizer_)
      value_ = legalizer_(Value());
  }
  template<typename VT>
  void Listen(const VT& broadcaster,
              const BindFuncType1<T, typename VT::element_type::ValueType>& f) {
    if (HasPath(this, broadcaster.get()))
      throw Exception("Multiple path is not allowed");
    auto func = [this, broadcaster, f]() {
      SetValue(f(Value(), broadcaster->Value())); };
    auto func_node = CreateFunctionNode(func, this, {broadcaster.get()});
    to_broadcaster_funcs_.push_back(func_node);
    broadcaster->to_listener_funcs_.push_back(func_node);
    UpdateGroupID(this, broadcaster.get());
  }
 private:
  LegalizerFuncType<T> legalizer_;
};

void InitVariable(const std::shared_ptr<VariableBase>& v);

template<typename T>
VPtr<T> MakeVariable(const T& v) {
  auto variable = std::make_shared<Variable<T>>(v);
  InitVariable(variable);
  return variable;
}

template<typename S, typename T1>
VPtr<S> MakeVariable(
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const BindFuncType1<S, T1>& bind_func) {
  auto variable = std::make_shared<Variable<S>>(initial_value);
  InitVariable(variable);
  variable->Listen(broadcaster1, bind_func);
  return variable;
}

template<typename S, typename T1, typename T2>
VPtr<S> MakeVariable(
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const VPtr<T2>& broadcaster2,
    const BindFuncType2<S, T1, T2>& bind_func) {
  auto variable = std::make_shared<Variable<S>>(initial_value);
  InitVariable(variable);
  variable->Listen(broadcaster1, broadcaster2, bind_func);
  return variable;
}

template<typename T>
VPtr<T> MakeVariable(const std::string& name, const T& v) {
  auto variable = std::make_shared<Variable<T>>(name, v);
  InitVariable(variable);
  return variable;
}

template<typename S, typename T1>
VPtr<S> MakeVariable(
    const std::string& name,
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const BindFuncType1<S, T1>& bind_func) {
  auto variable = std::make_shared<Variable<S>>(name, initial_value);
  InitVariable(variable);
  variable->Listen(broadcaster1, bind_func);
  return variable;
}

template<typename S, typename T1, typename T2>
VPtr<S> MakeVariable(
    const std::string& name,
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const VPtr<T2>& broadcaster2,
    const BindFuncType2<S, T1, T2>& bind_func) {
  auto variable = std::make_shared<Variable<S>>(name, initial_value);
  InitVariable(variable);
  variable->Listen(broadcaster1, broadcaster2, bind_func);
  return variable;
}

}  // namespace vbd
