"""Independent CPU control-flow checks of the exact corrected DRM C++ bodies.

The extracted bodies are unchanged. Only plant structures and the CUDA oracle
are mocked. This tests graph logic, not physical collision geometry or GPU speed.
"""
from __future__ import annotations
import hashlib,json,subprocess,sys,tempfile
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT))
from benchmarks.generic_motion_v1.runners.native_drm_corrections import correct_source,reconcile_query_source
from benchmarks.generic_motion_v1.runners.native_drm_query import SOURCE,patched_source

PRELUDE=r'''
#include <Eigen/Dense>
#include <fmt/core.h>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
namespace csdecomp {
using Voxels=Eigen::MatrixXf;
struct Options {int max_nodes_to_expand=1000,max_iterations_steering_to_node=3,max_num_blocked_edges_before_discard_node=10,max_number_planning_attempts=2;float online_edge_step_size=.01f,voxel_padding=.01f;bool try_shortcutting=false;};
struct Inspector {std::vector<int> robot_geometry_ids;};
struct Graph {std::unordered_map<int,Eigen::VectorXf> id_to_node_map;std::unordered_map<int,std::vector<int>> node_adjacency_map;};
struct Call {std::vector<float> values;float radius;};
std::vector<Call> calls;
std::function<bool(float)> free_point=[](float){return true;};
std::vector<uint8_t> checkCollisionFreeVoxelsCuda(const Eigen::MatrixXf* q,const Voxels*,float radius,const int*,const std::vector<int>&) {
  Call call;call.radius=radius;std::vector<uint8_t> result;
  for(int i=0;i<q->cols();++i){float x=(*q)(0,i);call.values.push_back(x);result.push_back(free_point(x));}
  calls.push_back(call);return result;
}
class DrmPlanner {
public:
  Graph drm_;Options options_;Inspector inspector_;int mplant_=0,num_joints_=1;bool is_initialized_=true,drm_loaded_=true,quiet_mode_=true;
  std::unordered_set<int> geometry_invalid_nodes_,environment_collision_nodes_,collision_set_;
  std::set<std::vector<int>> forbidden_edge_set_;
  std::unordered_map<int,int> attempts_to_leave_node;
  double Distance(const Eigen::VectorXf&,const Eigen::VectorXf&);
  bool CheckPathExists(int,int,const std::vector<int>&,std::vector<int>*);
  bool FindPath(int,const Eigen::VectorXf&,int,std::vector<Eigen::VectorXf>*,std::vector<int>*);
  bool IsEdgeBlocked(int,int) const;
  void PopulatePath(int,const Eigen::VectorXf&,int,const std::unordered_map<int,int>&,std::vector<Eigen::VectorXf>*,std::vector<int>*);
  bool Plan(const Eigen::VectorXf&,const Eigen::VectorXf&,const Voxels&,float,std::vector<Eigen::VectorXf>*);
  int _plan(const Eigen::VectorXf&,const Eigen::VectorXf&,const Voxels&,float,std::vector<Eigen::VectorXf>*);
  bool PlanStatePath(const std::vector<Eigen::VectorXf>&,const std::vector<int>&,bool,const Voxels&,const float&,std::vector<Eigen::VectorXf>*);
  bool SteerFunctionIsSafe(const Eigen::VectorXf&,const Eigen::VectorXf&,const Voxels&,const float&);
};
template<typename T,typename PriorityType> struct PriorityQueue {
  struct Compare {bool operator()(std::pair<PriorityType,T>a,std::pair<PriorityType,T>b){return a.first>b.first;}};
  std::priority_queue<std::pair<PriorityType,T>,std::vector<std::pair<PriorityType,T>>,Compare> elements;
  bool Empty(){return elements.empty();}void Put(T item,PriorityType priority){elements.emplace(priority,item);}T Get(){auto e=elements.top();elements.pop();return e.second;}
};
'''
MAIN=r'''
Eigen::VectorXf q(float x){return Eigen::VectorXf::Constant(1,x);}
DrmPlanner graph(){DrmPlanner p;p.drm_.id_to_node_map={{0,q(0)},{1,q(1)},{2,q(2)}};p.drm_.node_adjacency_map={{0,{}},{1,{2}},{2,{1}}};return p;}
bool run(DrmPlanner&p,float a,float b,std::vector<Eigen::VectorXf>* path){calls.clear();try{return p.Plan(q(a),q(b),Voxels(3,0),.04f,path);}catch(const std::runtime_error&){return false;}}
int checks=0;
void check(bool ok,const char* name){if(!ok){std::cerr<<"FAIL "<<name<<std::endl;std::exit(2);}++checks;std::cout<<"PASS "<<name<<std::endl;}
}
int main(){using namespace csdecomp;std::vector<Eigen::VectorXf> path;
  {auto p=graph();p.geometry_invalid_nodes_={0};p.environment_collision_nodes_={1};p.collision_set_={0,1};p.forbidden_edge_set_={{-1,1},{1,2}};free_point=[](float x){return x!=2;};run(p,1,1,&path);
   check(p.collision_set_==std::unordered_set<int>({0,2}),"cache_false_positive_removed_false_negative_added_geometry_invalid_retained");
   check(calls.front().values.size()==3,"all_nodes_queried_in_exact_batch");
   check(p.forbidden_edge_set_.empty(),"old_cached_edges_and_endpoint_tags_cleared");
   p.attempts_to_leave_node[1]=999;p.collision_set_.insert(1);p.forbidden_edge_set_={{-1,1},{1,2}};free_point=[](float){return true;};
   check(run(p,1,1,&path),"second_query_can_reuse_previously_excluded_free_node");
   check(p.collision_set_==std::unordered_set<int>({0}),"second_query_retains_only_permanent_geometry_invalid");
   check(p.attempts_to_leave_node.empty(),"second_query_clears_outgoing_attempts");}
  {auto p=graph();free_point=[](float){return true;};check(run(p,0,2,&path),"alternative_visible_attachment_connects_components");
   check(path.front()[0]==0&&path.back()[0]==2,"alternative_attachment_preserves_fixed_endpoints");
   bool alt=false;for(const auto& c:calls)if(c.values.size()>2&&c.values.front()==0&&c.values.back()==1)alt=true;
   check(alt,"alternative_attachment_steered_before_acceptance");}
  {auto p=graph();p.options_.max_iterations_steering_to_node=1;free_point=[](float){return true;};check(!run(p,0,2,&path),"one_candidate_budget_cannot_use_unchecked_alternative");
   check(calls.size()==4,"budget_one_checks_exact_batch_endpoints_and_one_per_endpoint_only");}
  {auto p=graph();p.options_.max_iterations_steering_to_node=2;free_point=[](float x){return !(x>1.45f&&x<1.55f);};
   check(!run(p,0,2,&path),"alternate_graph_connection_does_not_bypass_lazy_edge_check");
   check(p.forbidden_edge_set_.count({1,2})&&p.forbidden_edge_set_.count({2,1}),"failed_internal_steer_records_both_edge_directions");}
  {auto p=graph();p.drm_.node_adjacency_map={{0,{1}},{1,{0}},{2,{}}};p.forbidden_edge_set_={{0,1}};
   check(!p.CheckPathExists(0,1,{},nullptr),"reachability_honors_forward_forbidden_edge");
   check(p.CheckPathExists(1,0,{},nullptr),"reverse_direction_is_independent_when_only_forward_forbidden");
   std::vector<int> ids;check(!p.FindPath(0,q(0),1,&path,&ids),"astar_agrees_with_blocked_reachability");
   check(p.FindPath(1,q(1),0,&path,&ids),"astar_agrees_with_reverse_reachability");
   check(p.CheckPathExists(2,2,{},nullptr),"isolated_same_node_is_reachable");p.collision_set_.insert(2);
   check(!p.CheckPathExists(2,2,{},nullptr),"colliding_same_node_is_not_reachable");}
  {auto p=graph();free_point=[](float x){return !(x>.45f&&x<.55f);};p.options_.max_iterations_steering_to_node=3;
   check(!run(p,0,2,&path),"all_visible_alternatives_rejected_never_count_success");
   check(!p.collision_set_.count(1),"failed_endpoint_edge_does_not_invalidate_intermediate_node");
   check(p.forbidden_edge_set_.count({-1,1}),"failed_endpoint_edge_uses_local_tag");}
  {auto p=graph();free_point=[](float x){return x!=0;};check(!run(p,0,2,&path),"colliding_fixed_endpoint_is_rejected");check(p.collision_set_.count(0),"uncached_colliding_node_added");}
  {auto p=graph();free_point=[](float x){return x<.5f;};calls.clear();check(!p.SteerFunctionIsSafe(q(0),q(.5f),Voxels(3,0),.04f),"steering_checks_final_endpoint");check(calls.back().values.back()==.5f,"interpolated_endpoint_exact");}
  std::cout<<"TOTAL "<<checks<<std::endl;
}
'''

def body(source, name):
    start=source.index(name)
    brace=source.index('{',start)
    depth=1;i=brace+1
    while depth:
        depth += (source[i]=='{')-(source[i]=='}');i+=1
    return source[start:i]

def main():
    original=SOURCE.read_text()
    corrected=reconcile_query_source(correct_source(patched_source(original)))
    names=['double DrmPlanner::Distance','bool DrmPlanner::CheckPathExists','bool DrmPlanner::FindPath','bool DrmPlanner::IsEdgeBlocked','void DrmPlanner::PopulatePath','bool DrmPlanner::Plan(','int DrmPlanner::_plan','bool DrmPlanner::PlanStatePath','bool DrmPlanner::SteerFunctionIsSafe']
    cpp=PRELUDE+'\n'.join(body(corrected,n) for n in names)+MAIN
    out=ROOT/'results/trend_diagnostics/drm_consistent_independent_review.json'
    log=out.with_suffix('.log')
    result={'scope':'Exact corrected native C++ control-flow bodies with mocked CUDA oracle and minimal plant/graph structures; no GPU calls and not a geometry validity test','gpu_calls':0,'source_sha256':hashlib.sha256(corrected.encode()).hexdigest(),'extracted_methods':names,'test_script_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
    external=ROOT/'reproduction/vendor/paperdate_bazel_output/external'
    with tempfile.TemporaryDirectory(prefix='drm-control-review-') as tmp:
        source=Path(tmp)/'review.cpp';binary=Path(tmp)/'review';source.write_text(cpp)
        cmd=['g++','-std=c++17','-O2','-DFMT_HEADER_ONLY','-I'+str(external/'eigen'),'-I'+str(external/'fmt/include'),str(source),'-o',str(binary)]
        compile_result=subprocess.run(cmd,capture_output=True,text=True)
        result['compile_returncode']=compile_result.returncode
        if compile_result.returncode:
            log.write_text(compile_result.stdout+compile_result.stderr);out.write_text(json.dumps(result,indent=2)+'\n');raise RuntimeError('CPU harness compilation failed')
        run=subprocess.run([str(binary)],capture_output=True,text=True)
        log.write_text(run.stdout+run.stderr)
        result.update(returncode=run.returncode,checks=[line[5:] for line in run.stdout.splitlines() if line.startswith('PASS ')],failed=[line[5:] for line in run.stderr.splitlines() if line.startswith('FAIL ')])
    build=ROOT/'benchmarks/generic_motion_v1/native/build/drm_consistent_query'
    receipt=json.loads((build/'build.json').read_text())
    result['built_profile']=receipt['profile']
    result['compiled_source_matches_reviewed_generation']=hashlib.sha256((build/'drm_planner.cpp').read_bytes()).hexdigest()==result['source_sha256']==receipt['patched_source_sha256']
    result['binary_matches_receipt']=hashlib.sha256(Path(receipt['binary']).read_bytes()).hexdigest()==receipt['binary_sha256']
    fixture=json.loads((ROOT/'results/trend_diagnostics/drm_query_fixture/fixture.json').read_text())
    result['prior_real_gpu_fixture']=fixture
    result['budget_semantics']='max_iterations_steering_to_node is per endpoint per _plan attempt, preserved from upstream; full Plan may attempt up to max_number_planning_attempts. Exact reconciliation is once per Plan and charged within online query.'
    result['limits']=['Mock collision oracle tests control flow only; prior saved tiny canonical fixture supplies limited real GPU evidence.','No new GPU calls or performance measurement.','Native finite edge sampling is not continuous certification; common final validator remains necessary.']
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='extracted_methods'},indent=2))
    assert result['returncode']==0 and not result['failed']
    assert result['compiled_source_matches_reviewed_generation'] and result['binary_matches_receipt']
if __name__=='__main__':main()
