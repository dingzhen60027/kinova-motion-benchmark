import json,time
from pathlib import Path
import numpy as np
from pydrake.all import BsplineBasis,BsplineTrajectory,KnotVectorType,KinematicTrajectoryOptimization,IpoptSolver
out=Path('/tmp/nto_time_parameterization_20261101'); out.mkdir(exist_ok=True)
report={'scope':'CPU analytical seven-dimensional rest-to-rest motion without obstacles, only joint0 displacement2.0; isolates parameterization under identical KTO derivative constraints, no benchmark solve','continuous_rest_to_rest_lower_bound_s':3.0,'four_control_hull_lower_bound_s':6.0,'runs':[]}
base=np.zeros((4,7));base[2:,0]=2.0
for count in [4,16]:
 initial=BsplineTrajectory(BsplineBasis(4,4,KnotVectorType.kClampedUniform,0.,4.),[q[:,None] for q in base])
 original=initial.vector_values(np.linspace(0,4,1001))
 while len(initial.control_points())<count:
  unique=np.unique(initial.basis().knots()); i=np.argmax(np.diff(unique)); initial.InsertKnots([float((unique[i]+unique[i+1])/2)])
 error=float(np.max(np.abs(original-initial.vector_values(np.linspace(0,4,1001)))))
 kto=KinematicTrajectoryOptimization(initial);kto.AddDurationCost(1.0);kto.AddDurationConstraint(.05,100.)
 kto.AddPathPositionConstraint(base[0],base[0],0.);kto.AddPathPositionConstraint(base[-1],base[-1],1.)
 kto.AddVelocityBounds(-np.ones(7),np.ones(7));kto.AddAccelerationBounds(-np.ones(7),np.ones(7))
 kto.AddPathVelocityConstraint(np.zeros(7),np.zeros(7),0.);kto.AddPathVelocityConstraint(np.zeros(7),np.zeros(7),1.)
 solver=IpoptSolver();prog=kto.get_mutable_prog()
 for key,value in dict(max_iter=2000,tol=1e-6,print_level=0,file_print_level=5,output_file=str(out/f'ipopt_{count}.log')).items():prog.SetSolverOption(solver.solver_id(),key,value)
 tic=time.perf_counter();result=solver.Solve(prog);elapsed=time.perf_counter()-tic
 traj=kto.ReconstructTrajectory(result);vv=traj.MakeDerivative(1);aa=traj.MakeDerivative(2);t=np.linspace(0,traj.end_time(),10001)
 record=dict(control_count=count,solver_success=bool(result.is_success()),status=str(result.get_solution_result()),duration_s=float(traj.end_time()),initial_shape_deviation_rad=error,max_velocity_dense=float(np.max(np.abs(vv.vector_values(t)))),max_acceleration_dense=float(np.max(np.abs(aa.vector_values(t)))),max_velocity_control_hull=float(np.max(np.abs(vv.control_points()))),max_acceleration_control_hull=float(np.max(np.abs(aa.control_points()))),endpoint_velocity=float(np.max(np.abs(vv.vector_values([0.,traj.end_time()])))),solve_seconds=elapsed)
 report['runs'].append(record);np.savez_compressed(out/f'candidate_{count}.npz',controls=np.asarray(traj.control_points()),knots=traj.basis().knots(),duration=traj.end_time())
print(json.dumps(report,indent=2));(out/'result.json').write_text(json.dumps(report,indent=2)+'\n')
