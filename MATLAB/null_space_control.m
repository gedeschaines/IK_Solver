function [dH] = null_space_control(P3D,et,vt,w,p,dq,q,u)

% function [dH] = null_space_control(P3D,et,vt,w,p,dq,q,u) : computes and returns
%                                                            null space control vector
%
%   NOTE: This function is taiored specifically to hardcoded n-link
%         chains defined in IK_Solver_nlink.py
%
%   P3D = Plot3D
%   et  = position of target in world space
%   vt  = velocity of target in world space
%   w   = set of n joint rotation direction vectors in world space
%   p   = set of n+1 position vectors (n joints + end-effector) in world space
%   dq  = vector of n joint rotation rates (radians/sec)
%   q   = vector of joint rotations (radians)
%   u   = set of joint rotation axis unit vectors

global  dpr;

nq = length(dq);   % number of link chain joints
dH = zeros(1,nq);  % zero control vector

if P3D == 0
  %%fprintf('dq  = %8.5f %8.5f %8.5f %8.5f\n',...
  %%         dq(1),dq(2),dq(3),dq(5))
  avT1 = avelT_wrt_jointm(et,vt,w,p,dq,1);
  avT2 = avelT_wrt_jointm(et,vt,w,p,dq,2);
  avT3 = avelT_wrt_jointm(et,vt,w,p,dq,3);
  avT5 = avelT_wrt_jointm(et,vt,w,p,dq,5);
  avT  = [avT1(3), avT2(3), avT3(3), 0.0, avT5(3)];
  %%fprintf('avT = %8.5f %8.5f %8.5f %8.5f\n',...
  %%         avT(1),avT(2),avT(3),avT(5))
  avE1 = avelE_wrt_jointm(w,p,dq,1);
  avE2 = avelE_wrt_jointm(w,p,dq,2);
  avE3 = avelE_wrt_jointm(w,p,dq,3);
  avE5 = avelE_wrt_jointm(w,p,dq,5);
  avE  = [avE1(3), avE2(3), avE3(3), 0.0, avE5(3)];
  %%fprintf('avE = %8.5f %8.5f %8.5f %8.5f\n',...
  %%         avE(1),avE(2),avE(3),avE(5))
  dH(1) = avE(1);
  dH(2) = avE(2);
  dH(3) = avE(3);
  %dH(5) = avE(5);

  phiZ = get_endeff_rot(nq,q,u);
  %%fprintf('phiZ = %8.5f\n', phiZ*dpr)
  dH(nq) = phiZ;
else
  %%fprintf('dq  = %8.5f %8.5f %8.5f\n', dq(1),dq(2),dq(3))
  avT1 = avelT_wrt_jointm(et,vt,w,p,dq,1);
  avT2 = avelT_wrt_jointm(et,vt,w,p,dq,2);
  avT3 = avelT_wrt_jointm(et,vt,w,p,dq,3);
  avT  = [avT1(3), avT2(2), avT3(2), 0.0, 0.0];
  %%fprintf('avT = %8.5f %8.5f %8.5f\n', avT(1),avT(2),avT(3))
  avE1 = avelE_wrt_jointm(w,p,dq,1);
  avE2 = avelE_wrt_jointm(w,p,dq,2);
  avE3 = avelE_wrt_jointm(w,p,dq,3);
  avE  = [avE1(3), avE2(2), avE3(2), 0.0, 0.0];
  %%fprintf('avE = %8.5f %8.5f %8.5f\n', avE(1),avE(2),avE(3))
  dH(1) = avE(1);
  dH(2) = avE(2);
  dH(3) = avE(3);

  [psi,theta,phi] = get_endeff_ypr(nq,q,u);
  %%fprintf('psi,theta,phi = %8.5f, %8.5f, %8.5f\n',...
  %%         psi*dpr,theta*dpr,phi*dpr)
  dH(nq-1) = -theta;
  dH(nq)   = -psi;
end

end
