// Copyright (c) 2018 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#pragma once

#include <string>
#include <memory>
#include <ostream>
#include <functional>
#include <vector>
#include <unordered_set>
#include <tuple>
#include <boost/any.hpp>

#include "vbd/graphstyle.h"

namespace vbd {

class Exception : public std::exception {
 public:
  Exception() noexcept {};
  Exception(const std::string& msg) noexcept : msg_(msg) {}
  virtual ~Exception() {}
  virtual const char* what() const noexcept { return msg_.c_str(); }
 private:
  std::string msg_;
};

template<typename S, typename T1>
using BindFuncType1 = std::function<
  S (const S& self, const T1& broadcaster1)>;

template<typename S, typename T1, typename T2>
using BindFuncType2 = std::function<
  S (const S& self, const T1& broadcaster1, const T2& broadcaster2)>;

template<typename T>
using CallbackFuncType = std::function<T (const T& current)>;

using TransactionFuncType = std::function<void ()>;

template<typename T>
class Variable;

template<typename T>
using VPtr = std::shared_ptr<Variable<T>>;

template<typename S>
VPtr<S> MakeVariable(const S& initial_value);

template<typename S, typename T1>
VPtr<S> MakeVariable(
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const BindFuncType1<S, T1>& bind_func);

template<typename S, typename T1, typename T2>
VPtr<S> MakeVariable(
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const VPtr<T2>& broadcaster2,
    const BindFuncType2<S, T1, T2>& bind_func);

template<typename S>
VPtr<S> MakeVariable(const std::string& id, const S& initial_value);

template<typename S, typename T1>
VPtr<S> MakeVariable(
    const std::string& id,
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const BindFuncType1<S, T1>& bind_func);

template<typename S, typename T1, typename T2>
VPtr<S> MakeVariable(
    const std::string& id,
    const S& initial_value,
    const VPtr<T1>& broadcaster1,
    const VPtr<T2>& broadcaster2,
    const BindFuncType2<S, T1, T2>& bind_func);

void Transaction(const TransactionFuncType& func);

void ConstructTransaction(const TransactionFuncType& func);

void ExecTransaction();

void CreateGraphViz(const std::string& graph_name,
                    std::ostream& os,
                    std::shared_ptr<GraphStyle> style = nullptr);

}  // namespace vbd

#include "vbd/vbd_impl.h"
