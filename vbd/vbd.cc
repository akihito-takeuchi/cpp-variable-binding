// Copyright (c) 2019 Akihito Takeuchi
// Distributed under the MIT License : http://opensource.org/licenses/MIT

#include "vbd/vbd.h"
#include "vbd/vbd_impl.h"
#include "vbd/graphstyle.h"

#include <iostream>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/predicate.hpp>
#include <boost/algorithm/string/join.hpp>

namespace vbd {

namespace {

const std::string kGraphNameKey = "graph_name";
const std::string kGraphSectionName = "graph";
const std::string kNodeSectionName = "node";
const std::string kEdgeSectionName = "edge";

void WriteGraphVizHeader(const std::string& graph_name,
                         std::ostream& os,
                         const std::shared_ptr<GraphStyle>& style) {
  auto values = style->GetStyle();
  os << "digraph " << graph_name << " {\n";
  for (auto& sec_name : {kGraphSectionName,
                         kNodeSectionName,
                         kEdgeSectionName}) {
    std::vector<std::string> style_lines;
    for (auto& item : values) {
      if (!boost::algorithm::starts_with(item.first, sec_name + "."))
        continue;
      style_lines.push_back(
          (boost::format("%1% = \"%2%\"")
           % item.first.substr(sec_name.size() + 1) % item.second).str());
    }
    if (style_lines.empty())
      continue;
    os << "  " << sec_name << " [\n";
    os << "    " << boost::algorithm::join(style_lines, ",\n    ");
    os << "\n  ];\n";
  }
}

void WriteGraphVizFooter(std::ostream& os) {
  os << "}\n";
}

std::string GetVarName(
    VariableBase* v, std::unordered_map<VariableBase*, std::string>& name_map) {
  std::string var_name;
  do {
    auto itr = name_map.find(v);
    if (itr != name_map.end()) {
      var_name = itr->second;
      break;
    }
    std::string name_base = v->Name();
    if (name_base.empty())
      name_base = "Var";
    var_name = name_base;
    size_t name_idx = 0;
    std::unordered_set<std::string> used_names;
    for (auto& pair : name_map)
      used_names.insert(pair.second);
    while (used_names.find(var_name) != used_names.cend())
      var_name = name_base + "_" + boost::lexical_cast<std::string>(name_idx ++);
  } while (false);
  name_map[v] = var_name;
  return var_name;
}

}  // namespace

class FunctionNode {
 public:
  FunctionNode() = default;
  FunctionNode(const VoidFuncType& func,
               VariableBase* listener,
               const std::vector<VariableBase*>& broadcasters)
      : func_(func), listener_variable_(listener),
        broadcaster_variables_(broadcasters) {}
  VariableBase* Listener() const { return listener_variable_; }
  const std::vector<VariableBase*>& Broadcasters() const {
    return broadcaster_variables_;
  }
  void Apply() {
    func_();
  }
 private:
  VoidFuncType func_;
  VariableBase* listener_variable_ = nullptr;
  std::vector<VariableBase*> broadcaster_variables_;
  friend class VbdEngine;
};

struct PropergateTreeNode {
  PropergateTreeNode() = default;
  PropergateTreeNode(FunctionNode* f) : func(f) {}
  FunctionNode* func = nullptr;
  std::vector<PropergateTreeNode> next_nodes;
};

class VbdEngine {
 public:
  VbdEngine(const VbdEngine&) = delete;
  VbdEngine& operator=(const VbdEngine&) = delete;
  static VbdEngine& Instance();
  void InitVariable(const std::shared_ptr<VariableBase>& v);
  void UpdateAccess(VariableBase* v) const;
  bool HasPath(const VariableBase* listener,
               const VariableBase* broadcaster) const;
  void UpdateGroupID(VariableBase* listener,
                     VariableBase* broadcaster);
  void StartTransaction();
  bool TryStartTransaction();
  void EndTransaction();
  void RegisterAssignOperation(VariableBase* variable,
                               const VoidFuncType& func);
  void UnregisterAssignOperation(VariableBase* variable);
  void Exec();
  bool Executing() { return executing_; }
  void CreateGraphViz(const std::string& graph_name,
                      std::ostream& os,
                      const std::shared_ptr<GraphStyle>& style);

 private:
  VbdEngine();
  void UpdateAccess_(VariableBase* v,
                     std::forward_list<VariableBase*>& visited) const;
  bool HasPath_(std::unordered_set<const VariableBase*> listeners,
                const VariableBase* broadcaster) const;
  void Assign_();
  void BuildPropergationFlow_(PropergateTreeNode& tree);
  void TraceTree_(PropergateTreeNode& tree,
                  std::unordered_set<FunctionNode*>& visited);
  void Propergate_(PropergateTreeNode& tree);
  void ProcessPropergateNode_(PropergateTreeNode& tree);
  void Rollback_();
  static void Init();
  static std::once_flag init_flag_;
  static std::unique_ptr<VbdEngine> instance_;

  std::unordered_set<std::shared_ptr<VariableBase>> variables_;
  std::unordered_map<unsigned int,
                     std::unordered_set<VariableBase*>> variables_by_group_;
  std::vector<std::pair<VariableBase*, VoidFuncType>> assign_info_list_;
  unsigned int group_id_next_ = 0;
  bool in_transaction_ = false;
  bool executing_ = false;
};

VbdEngine::VbdEngine() = default;

std::once_flag VbdEngine::init_flag_;
std::unique_ptr<VbdEngine> VbdEngine::instance_ = nullptr;

VbdEngine& VbdEngine::Instance() {
  std::call_once(init_flag_, Init);
  return *instance_;
}

void VbdEngine::Init() {
  instance_ = std::unique_ptr<VbdEngine>(new VbdEngine);
}

void VbdEngine::InitVariable(const std::shared_ptr<VariableBase>& v) {
  v->group_id_ = group_id_next_ ++;
  variables_by_group_[v->group_id_].emplace(v.get());
  variables_.emplace(v);
}

void VbdEngine::StartTransaction() {
  if (in_transaction_)
    throw Exception(
        "VbdEngine::StartTransaction : "
        "Transaction has already been started.");
  if (!assign_info_list_.empty())
    throw Exception(
        "VbdEngine::StartTransaction : "
        "A transaction has already been constructed and wait execution");
  in_transaction_ = true;
}

bool VbdEngine::TryStartTransaction() {
  if (in_transaction_)
    return false;
  StartTransaction();
  return true;
}

void VbdEngine::EndTransaction() {
  if (!in_transaction_)
    throw Exception(
        "VbdEngine::EndTransaction : Transaction has not been started.");
  in_transaction_ = false;
}

void VbdEngine::RegisterAssignOperation(VariableBase* variable,
                                        const VoidFuncType& func) {
  UnregisterAssignOperation(variable);
  assign_info_list_.emplace_back(variable, func);
}

void VbdEngine::UnregisterAssignOperation(VariableBase* variable) {
  assign_info_list_.erase(
      std::remove_if(assign_info_list_.begin(), assign_info_list_.end(),
                     [&variable](auto p) { return p.first == variable; }),
      assign_info_list_.end());
}

void VbdEngine::Exec() {
  executing_ = true;
  PropergateTreeNode start_node;
  BuildPropergationFlow_(start_node);
  try {
    Propergate_(start_node);
  } catch (const Exception& e) {
    Rollback_();
    assign_info_list_.clear();
    executing_ = false;
    throw;
  }
  executing_ = false;
}

void VbdEngine::Assign_() {
  for (auto& info : assign_info_list_) {
    info.second();
    info.first->ExecCallback();
  }
  assign_info_list_.clear();
}

void VbdEngine::BuildPropergationFlow_(PropergateTreeNode& tree) {
  for (auto& info : assign_info_list_) {
    for (auto next_func : info.first->to_listener_funcs_) {
      auto next_func_ptr = next_func.get();
      std::unordered_set<FunctionNode*> visited{next_func_ptr};
      tree.next_nodes.emplace_back(next_func_ptr);
      TraceTree_(tree.next_nodes.back(), visited);
    }
  }
}

void VbdEngine::TraceTree_(PropergateTreeNode& tree,
                           std::unordered_set<FunctionNode*>& visited) {
  for (auto next_func : tree.func->listener_variable_->to_listener_funcs_) {
    auto next_func_ptr = next_func.get();
    if (visited.find(next_func_ptr) == visited.end()) {
      visited.insert(next_func_ptr);
      tree.next_nodes.emplace_back(next_func_ptr);
      TraceTree_(tree.next_nodes.back(), visited);
    }
  }
}

void VbdEngine::ProcessPropergateNode_(PropergateTreeNode& node) {
  node.func->Apply();
  Assign_();
  for (auto& next_node : node.next_nodes)
    ProcessPropergateNode_(next_node);
}

void VbdEngine::Propergate_(PropergateTreeNode& tree) {
  Assign_();
  for (auto& node : tree.next_nodes)
    ProcessPropergateNode_(node);
  Assign_();
}

void VbdEngine::Rollback_() {
}

void VbdEngine::UpdateAccess(VariableBase* v) const {
  std::forward_list<VariableBase*> chain{v};
  UpdateAccess_(v, chain);
}

void VbdEngine::UpdateAccess_(
    VariableBase* v, std::forward_list<VariableBase*>& chain) const {
  if (v->to_broadcaster_funcs_.size() == 0) {
    // v is not a listener
    v->access_ = Access::kWritable;
    return;
  }

  for (auto& broadcaster_func : v->to_broadcaster_funcs_) {
    for (auto& broadcaster : broadcaster_func->Broadcasters()) {
      // If subtree is already initialized, there should be no cyclic reference
      if (broadcaster->access_ != Access::kNotInitialized) {
        v->access_ = Access::kReadOnly;
        continue;
      }
    
      auto itr = std::find(chain.begin(), chain.end(), broadcaster);
      // If broadcaster is already in chain, it has cyclic reference
      if (itr != chain.end()) {
        itr ++;
        std::for_each(chain.begin(), itr,
                      [](auto& v) { v->access_ = Access::kWritable; });
        continue;
      }

      chain.push_front(broadcaster);
      UpdateAccess_(broadcaster, chain);
      chain.pop_front();
    }
  }
}

bool VbdEngine::HasPath(const VariableBase* listener,
                              const VariableBase* broadcaster) const {
  return HasPath_({listener}, broadcaster);
}

bool VbdEngine::HasPath_(
    std::unordered_set<const VariableBase*> listeners,
    const VariableBase* broadcaster) const {
  if (listeners.empty())
    return false;
  if (listeners.find(broadcaster) != listeners.end())
    return true;
  std::unordered_set<const VariableBase*> next_listeners;
  for (auto& listener : listeners)
    for (auto& broadcaster_func : listener->to_broadcaster_funcs_)
      next_listeners.insert(broadcaster_func->Broadcasters().begin(),
                            broadcaster_func->Broadcasters().end());
  return HasPath_(next_listeners, broadcaster);
}

void VbdEngine::UpdateGroupID(VariableBase* listener,
                                    VariableBase* broadcaster) {
  std::vector<unsigned int> group_ids_to_reset;
  group_ids_to_reset.emplace_back(listener->group_id_);
  if (listener->group_id_ != broadcaster->group_id_) {
    group_ids_to_reset.emplace_back(broadcaster->group_id_);
    listener->group_id_ = broadcaster->group_id_;
    variables_by_group_[listener->group_id_].erase(listener);
    variables_by_group_[broadcaster->group_id_].emplace(listener);
  }
  for (auto group_id : group_ids_to_reset)
    for (auto v : variables_by_group_[group_id])
      v->access_ = Access::kNotInitialized;
}

void VbdEngine::CreateGraphViz(const std::string& graph_name,
                               std::ostream& os,
                               const std::shared_ptr<GraphStyle>& style) {
  WriteGraphVizHeader(graph_name, os, style);
  std::unordered_map<VariableBase*, std::string> name_map;
  std::vector<std::string> edges;
  os << "  // nodes\n";
  for (auto& node : variables_) {
    auto name_from = GetVarName(node.get(), name_map);
    os << "  " << name_from << ";\n";
    for (auto& listener_func : node->to_listener_funcs_) {
      auto name_to = GetVarName(listener_func->listener_variable_, name_map);
      edges.push_back(name_from + " -> " + name_to);
    }
  }
  os << "  // edges\n";
  for (auto& edge : edges)
    os << "  " << edge << ";\n";
  WriteGraphVizFooter(os);
}
  
VariableBase::VariableBase()
    : access_(Access::kNotInitialized) {
}

VariableBase::VariableBase(const std::string& name)
    : name_(name), access_(Access::kNotInitialized) {
}

VariableBase::~VariableBase() = default;

bool VariableBase::IsReadonly() const {
  if (access_ == Access::kNotInitialized) {
    VbdEngine::Instance().UpdateAccess(
        const_cast<VariableBase*>(this));
  }
  return access_ != Access::kWritable;
}

void VariableBase::RegisterAssignOperation(VariableBase* variable,
                                           const VoidFuncType& func) {
  auto& engine = VbdEngine::Instance();
  if (engine.Executing()) {
    engine.RegisterAssignOperation(variable, func);
    return;
  }

  auto new_transaction = engine.TryStartTransaction();
  engine.RegisterAssignOperation(variable, func);
  if (new_transaction) {
    engine.EndTransaction();
    engine.Exec();
  }
}

void VariableBase::UnregisterAssignOperation(VariableBase* variable) {
  auto& engine = VbdEngine::Instance();
  engine.UnregisterAssignOperation(variable);
}

void VariableBase::UpdateGroupID(VariableBase* listener,
                                 VariableBase* broadcaster) {
  VbdEngine::Instance().UpdateGroupID(listener, broadcaster);
}

bool VariableBase::HasPath(const VariableBase* listener,
                           const VariableBase* broadcaster) const {
  return VbdEngine::Instance().HasPath(listener, broadcaster);
}

FunctionPtr VariableBase::CreateFunctionNode(
    const VoidFuncType& func,
    VariableBase* listener,
    const std::vector<VariableBase*>&& broadcaster) {
  return std::make_shared<FunctionNode>(func, listener, broadcaster);
}

void InitVariable(const std::shared_ptr<VariableBase>& v) {
  VbdEngine::Instance().InitVariable(v);
}

void CreateGraphViz(const std::string& graph_name,
                    std::ostream& os,
                    std::shared_ptr<GraphStyle> style) {
  if (!style)
    style = std::make_shared<GraphStyle>();
  VbdEngine::Instance().CreateGraphViz(graph_name, os, style);
}

void Transaction(const TransactionFuncType& func) {
  ConstructTransaction(func);
  ExecTransaction();
}

void ConstructTransaction(const TransactionFuncType& func) {
  auto& engine = VbdEngine::Instance();
  engine.StartTransaction();
  func();
  engine.EndTransaction();
}

void ExecTransaction() {
  VbdEngine::Instance().Exec();
}


}  // namespace vbd
