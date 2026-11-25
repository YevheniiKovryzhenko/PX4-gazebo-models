#!/usr/bin/env python3
"""Refresh the committed numerical snapshot from a local MATLAB aircraft model.

Offline only: requires MATLAB, NumPy and SciPy. The generated SDF has none of
these dependencies. Source data is sampled read-only, in a temporary directory.
"""
import argparse
import json
import hashlib
import math
import os
import subprocess
import tempfile
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import nnls
root = Path(__file__).resolve().parent
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('source', type=Path, help='S2F-X200 MATLAB model folder')
parser.add_argument('--matlab', default='matlab', help='MATLAB executable')
args = parser.parse_args()
source = args.source.resolve()
quote = lambda s: str(s).replace("'", "''")
with tempfile.TemporaryDirectory(prefix='x200-reference-') as work:
    destination = Path(work) / 'samples.json'
    command = (f"addpath('{quote(root)}'); "
               f"export_reference('{quote(source)}','{quote(destination)}')")
    subprocess.run([args.matlab, '-sd', work, '-batch', command], check=True,
                   env={**os.environ, 'MATLAB_PREFDIR': str(Path(work) / 'prefs')})
    ref = json.loads(destination.read_text())
d = json.loads((root / 'matlab_data.json').read_text())
loads = np.array(ref['rotor_loads'])
frd_to_flu = np.diag([1, -1, -1])
d['mass_kg'] = ref['massprop']['Mass']
d['cg_flu_m'] = (frd_to_flu @ ref['massprop']['r_cg']).tolist()
d['inertia_flu_kg_m2'] = (frd_to_flu @ ref['massprop']['MOI'] @ frd_to_flu).tolist()
d['wing_span_m'] = ref['wing_span_m']
d['wing_area_m2'] = ref['wing_area_m2']
for key in ['E1212', 'N0012']:
    d.pop(key, None)
for i, rotor in enumerate(d['rotors']):
    p = ref['propulsion']
    rotor['position_flu_m'] = [p['x_loc'][i], -p['y_loc'][i], -p['z_loc'][i]]
    rotor['roll_deg'] = p['ph_hub'][i]
    rotor['spin_frd'] = p['SpinDir'][i]
    rotor['radius_m'] = (p['D_LP'][i] if i < 4 else p['D_MP']) / 39.37 / 2
d['source_sha256']=hashlib.sha256((source/'Vehicle.mat').read_bytes()).hexdigest()
d['source_files_sha256']={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [source/'Vehicle.mat',*[source/'AircraftDynamicsModel'/s for s in ['CALC_FM.m','DynamicInflowModel.m','BEMT_Model.m','MotorDynamics.m','Strip_Aero_Model.m','NLS_Coeff_Aero_Model_Generated.m','MP_Propeller_Model.m']]]}
d['reference_density_kg_m3']=1.225
d['rotor_reference_samples']={'columns':['rpm','lift_fx_N','lift_fy_N','lift_fz_N','lift_mx_Nm','lift_my_Nm','lift_mz_Nm','cruise_thrust_N','cruise_torque_Nm'],'values':loads.tolist()}
d['control_reference_per_deg']={'speed_m_s':20,'columns':['Fx_frd_N','Fy_frd_N','Fz_frd_N','Mx_frd_Nm','My_frd_Nm','Mz_frd_Nm'],'values':ref['control_derivatives_per_deg']}
d['aero_reference']={'columns':['u','v','w','p','q','r','alpha_deg','Fx_frd_N','Fy_frd_N','Fz_frd_N','Mx_frd_Nm','My_frd_Nm','Mz_frd_Nm'],'values':ref['aero_reference']}
d['esc_reference']={'gains':ref['esc'],'rpm_start':5000,'rpm_step':250,'torque_limit_Nm':1.271,'power_limit_W':1375}
# Match the active BEM lift model, including CALC_FM's download correction.
download=(1-2*math.atan(.85/ref['propulsion']['D_LP'][0])/(2*math.pi))**(1/3)
w2=(loads[:,0]*math.pi/30)**2;mask=loads[:,0]>=3000
for i,r in enumerate(d['rotors']):
 t=-loads[:,3]*download if i<4 else loads[:,7];q=abs(loads[:,6]) if i<4 else loads[:,8]
 kt=float(w2[mask]@t[mask]/(w2[mask]@w2[mask]));kq=float(w2[mask]@q[mask]/(w2[mask]@w2[mask]));r['thrust_coefficient']=kt;r['moment_ratio_m']=kq/kt;r['prop_inertia_kg_m2']=float(ref['prop_inertia'][i]);r['rotor_velocity_slowdown']=10
 name='LP' if i<4 else 'MP';kp=ref['esc']['KP_'+name];ki=ref['esc']['KI_'+name]
 def response(delta):
  rpm0=5000;target=rpm0+delta;q0=np.interp(rpm0,loads[:,0],q);pw0=q0*rpm0*math.pi/30
  def fun(_,x):
   rpm,e=x;omega=max(rpm,1)*math.pi/30;err=target-rpm
   pw=pw0+np.clip(kp*err+ki*e,-1375,1375-pw0)
   torque=np.clip(pw/omega,-1.271,1.271)
   return [(torque-np.interp(rpm,loads[:,0],q))/r['prop_inertia_kg_m2']*30/math.pi,err]
  def event(_,x):return (x[0]-rpm0)/delta-(1-math.exp(-1))
  event.terminal=True
  sol=solve_ivp(fun,[0,2],[rpm0,0],events=event,max_step=.0005,rtol=1e-9,atol=1e-9)
  return float(sol.t_events[0][0])
 r['time_constant_up_s']=response(250);r['time_constant_down_s']=response(-250)
 r['fit_max_relative_error_thrust']=float(max(abs(kt*w2[mask]/t[mask]-1)))
 r['fit_max_relative_error_torque']=float(max(abs(kq*w2[mask]/q[mask]-1)))
# Active strip-model coordinate corrections, before FRD -> FLU.
g=np.array(ref['strip_geometry']);g[:128,13]+=(1.49-.5)/39.37;g[:128,15]-=(-.65748+.5)/39.37
g[128:182,13]-=(-25.201+24.708)/39.37;g[128:182,15]-=(-1.4764+2.565)/39.37
g[182:,14:16]=np.column_stack([g[182:,15],-g[182:,14]])
g[182:,13]-=(-24.114+24.208)/39.37;g[182:,15]-=(-2.1024+2.815)/39.37
# Existing seven groups partition all strips; split wing at midspan.
groups=[np.where((np.arange(201)<64)&(abs(g[:,14])>.5))[0],np.where((np.arange(201)<64)&(abs(g[:,14])<=.5))[0],np.where((np.arange(201)>=64)&(np.arange(201)<128)&(abs(g[:,14])<=.5))[0],np.where((np.arange(201)>=64)&(np.arange(201)<128)&(abs(g[:,14])>.5))[0],np.arange(128,155),np.arange(155,182),np.arange(182,201)]
for i,(s,indices) in enumerate(zip(d['surfaces'],groups)):
 area=float(g[indices,4].sum());cp=(g[indices,13:16]*g[indices,4,None]).sum(axis=0)/area;cp*=np.array([1,-1,-1]);s['area_m2']=area;s['cp_flu_m']=cp.tolist();s['chord_m']=float(np.average(g[indices,3],weights=g[indices,4]))
 table=np.array(ref['profiles'][s['profile']]);a=np.deg2rad(table[:,0]);small=abs(table[:,0])<=3;slope,intercept=np.polyfit(a[small],table[small,1],1)
 ar=(d['wing_span_m']**2/g[:128,4].sum()) if i<4 else ((.55**2/g[128:182,4].sum()) if i<6 else (.25**2/area))
 cla=float(slope/(1+slope/(math.pi*ar)));scale=.75 if i<4 else 1
 a0=float(intercept/slope);incidence=6 if i<4 else 0
 fit_a=np.deg2rad(np.linspace(-3,10,100) if i<4 else np.linspace(-8,8,100));effective=fit_a+a0+math.radians(incidence)
 cl=cla*effective;section_a=cl/slope-a0
 cd=np.interp(section_a,a,table[:,2])+cl**2/(math.pi*ar);cm=np.interp(section_a,a,table[:,3])*s['chord_m']
 cda=float(abs(effective)@cd/(effective@effective));cma=float(effective@cm/(effective@effective))
 peak=np.argmax(table[:,1]);stall=float(np.deg2rad(table[peak,0])+a0)
 ctrl=np.array(ref['control_derivatives_per_deg'][i]);qbar=245
 # Gazebo's horizontal lift is +Z FLU, rudder lift is +Y FLU.
 lift_per_deg=-ctrl[2] if i<6 else -ctrl[1]
 control_cl=lift_per_deg*180/math.pi/(qbar*area)
 # Fit residual pitch moment in addition to lift acting at the panel CP.
 cm_delta=(ctrl[4]+cp[0]*ctrl[2])*180/math.pi/(qbar*area) if i<6 else 0
 s['aerodynamics']={'cla':cla*scale,'a0_rad':a0+math.radians(incidence),'stall_rad':stall,'cda':cda,'cma_m':cma,'cla_stall':-3.85*scale,'cda_stall':-.92,'cma_stall_m':0,'control_joint_rad_to_cl':float(control_cl),'cm_delta_m_rad':float(cm_delta),'aspect_ratio':ar,'incidence_deg':incidence}
# Fit total pre-stall drag and pitching moment to the active strip + NLS
# model near level forward flight. Keep the distributed force application and
# its local rotational-flow damping, rather than inventing body drag constants.
rows = [row for row in ref['aero_reference'] if abs(row[6]) <= 3]
drag_matrix = []
drag_target = []
for row in rows:
    alpha = math.radians(row[6])
    drag_matrix.append([sum(245 * s['area_m2'] * abs(alpha + s['aerodynamics']['a0_rad'])
                            for s in d['surfaces'][start:end]) for start, end in [(0, 4), (4, 6)]])
    drag_target.append(-row[7] * math.cos(alpha) - row[9] * math.sin(alpha))
drag_coefficients, _ = nnls(np.array(drag_matrix), np.array(drag_target))
for i, surface in enumerate(d['surfaces'][:6]):
    surface['aerodynamics']['cda'] = float(drag_coefficients[0 if i < 4 else 1])
moment_matrix = []
moment_target = []
for row in rows:
    alpha = math.radians(row[6])
    lever_moment = 0
    for surface in d['surfaces'][:6]:
        aero = surface['aerodynamics']
        effective = alpha + aero['a0_rad']
        lift = 245 * surface['area_m2'] * aero['cla'] * effective
        drag = 245 * surface['area_m2'] * aero['cda'] * abs(effective)
        fx = lift * math.sin(alpha) - drag * math.cos(alpha)
        fz = -lift * math.cos(alpha) - drag * math.sin(alpha)
        x, _, z_flu = surface['cp_flu_m']
        lever_moment += -z_flu * fx - x * fz
    moment_matrix.append([sum(245 * s['area_m2'] * (alpha + s['aerodynamics']['a0_rad'])
                              for s in d['surfaces'][start:end]) for start, end in [(0, 4), (4, 6)]])
    moment_target.append(row[11] - lever_moment)
moment_coefficients = np.linalg.lstsq(moment_matrix, moment_target, rcond=None)[0]
for i, surface in enumerate(d['surfaces'][:6]):
    surface['aerodynamics']['cma_m'] = float(moment_coefficients[0 if i < 4 else 1])
d['aero_fit'] = {'speed_m_s': 20, 'alpha_deg': [-3, 0, 3],
                 'method': 'finite-wing section lift; effective drag and pitch fit to Strip_Aero_Model including NLS'}
(root/'matlab_data.json').write_text(json.dumps(d,indent=2)+'\n')
print('Updated', root / 'matlab_data.json')
subprocess.run([os.sys.executable, str(root / 'generate_model.py')], check=True)
