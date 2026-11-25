function export_reference(source, destination)
% Offline numerical export; never changes the source aircraft or Vehicle.mat.
% Called by update_reference.py. Requires MATLAB, but Gazebo does not.

addpath(fullfile(source,'AircraftDynamicsModel'));
load(fullfile(source,'Vehicle.mat'),'Vehicle');
FltCon.rho=1.225; FltCon.SpeedOfSound=340; FltCon.DynVisc=1.7894e-5;
X=zeros(12,1); U=zeros(12,1);
aero_reference=zeros(5,13); k=0;
for j=1:7
 loads=zeros(2,6);
 for s=1:2
  X(1)=20; U(:)=0;U(j)=(-1)^s*2;
  [F,M]=Strip_Aero_Model(X,Vehicle,FltCon,U,[],0);
  loads(s,:)=[F',M'];
 end
 control_derivatives(j,:)=(loads(2,:)-loads(1,:))/4;
end
for a=[-6,-3,0,3,6]
 X=zeros(12,1);X(1)=20*cosd(a);X(3)=20*sind(a);U(:)=0;
 [F,M]=Strip_Aero_Model(X,Vehicle,FltCon,U,[],0);
 k=k+1; aero_reference(k,:)=[X(1:6)',a,F',M'];
end
p=Vehicle.Propulsion;
rp=[p.x_loc(1:4),p.y_loc(1:4),p.z_loc(1:4)];
rpm=(2000:500:8000)';
loads=zeros(length(rpm),9);
X=zeros(12,1);
for i=1:length(rpm)
 [F,Q]=DynamicInflowModel(X,Vehicle,FltCon,rpm(i)*ones(4,1),[],[], ...
     90*ones(4,1),-p.SpinDir(1:4),p.ph_hub(1:4),zeros(4,1),rp,1,[]);
 [Tmp,Qmp]=MP_Propeller_Model(X,Vehicle,FltCon,rpm(i),0,p.SpinDir(5),[]);
 loads(i,:)=[rpm(i),F(:,1)',Q(:,1)',Tmp,Qmp];
end
ref.rotor_loads=loads;
ref.massprop=Vehicle.MassProp;
ref.propulsion=struct('x_loc',p.x_loc,'y_loc',p.y_loc,'z_loc',p.z_loc, ...
    'SpinDir',p.SpinDir,'ph_hub',p.ph_hub,'D_LP',p.D_LP,'D_MP',p.D_MP);
ref.wing_span_m=Vehicle.Aero.b;
ref.wing_area_m2=Vehicle.Aero.S;
ref.control_derivatives_per_deg=control_derivatives;
ref.aero_reference=aero_reference(1:k,:);
ref.profiles.E1212=[Vehicle.Aero.E1212_Strip_AOA_EXT(:),Vehicle.Aero.E1212_Strip_CL_EXT(:),Vehicle.Aero.E1212_Strip_CD_EXT(:),Vehicle.Aero.E1212_Strip_Cm_EXT(:)];
ref.profiles.N0012=[Vehicle.Aero.N0012_Strip_AOA_EXT(:),Vehicle.Aero.N0012_Strip_CL_EXT(:),Vehicle.Aero.N0012_Strip_CD_EXT(:),Vehicle.Aero.N0012_Strip_Cm_EXT(:)];
ref.strip_geometry=Vehicle.Aero.StripData;
ref.esc=Vehicle.Controls.ESC;
ref.prop_inertia=Vehicle.Propulsion.I;
fid=fopen(destination,'w');
assert(fid>=0,'Cannot create reference export');
cleanup=onCleanup(@() fclose(fid));
fwrite(fid,jsonencode(ref));
end

