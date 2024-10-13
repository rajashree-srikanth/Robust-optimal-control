function [K_t,P_t,phi_t]=twopbvp(T,t,a,b,q,r)
%%
% TWOPBVP  Two point boundary-value problem (LQ problem with finite horizon T
%          and null final state).
%
%          * K_t=twopbvp(T,t,A,B,Q,R) computes, at current time t (in [0, T[), 
%            the optimal gain K_t for the following LQ problem:
%
%            - System:
%                .
%                x = Ax + Bu  with negative state feedback u(t)=-K_t(t)*x(t)
%
%            - Performance index:
%                        /T
%                J = 0.5 |  {x'Qx + u'Ru} dt
%                       / 0
%            - Hard constraint:
%                x(T)=0
%
%          * [K_t,P_t]=twopbvp(T,t,A,B,Q,R) computes also the semi-definite
%            positive solution P_t at current time t of the associated
%            Riccati equation:
%                 .                             -1 
%                P_t = -P_t A - A' P_t + P_t B R  B'P_t - Q
%
%          * [K_t,P_t,phi_t]=twopbvp(T,t,A,B,Q,R) computes also the transition
%            matrix phi_t at current time t on the optimal trajectory, s.t.:
%
%                x(t)=phi_t x(0)
%
%
narginchk(6,6);
error(abcdchk(a,b)); %checks dimensional consistency of the a,b matrices
if isempty(a) || isempty(b)
  error('A and B matrices cannot be empty.')
end
if t>T
  disp('Input argument problem: t>T !!!');
  K_t=[];P_t=[];phi_t=[];
  return
end
%
n=size(a,1);
[mb,nb]=size(b);
[mq,nq]=size(q);
if (n~=mq) || (n~=nq) 
  error('A and Q must be the same size.');
end
[mr,nr]=size(r);
if (mr~=nr) || (nb~=mr)
  error('B and R must be consistent.');
end
%
H=[a -b*inv(r)*b';-q -a'];
eH_t=expm(H*(t));
eH_Tt=expm(H*(T-t));
eH_T = expm(H*(T));
% E11_t = eH_t(1:n, n+1:2*n);
% E11_T = eH_T(1:n, n+1:2*n);
% E11_Tt = eH_Tt(1:n, n+1:2*n);
E11_t = eH_t(1:n, 1:n);
E11_T = eH_T(1:n, 1:n);
E11_Tt = eH_Tt(1:n, 1:n);
E12_t = eH_t(1:n,n+1:2*n); %for any other t E12(t)
E12_T = eH_T(1:n,n+1:2*n); %for t=T E12(T)
E12_Tt = eH_Tt(1:n,n+1:2*n); %for t=T E12(T)
% % E21_t = eH_t(n+1:2*n, 1:n);
% % E22_t = eH_t(n+1:2*n, n+1:2*n);
P_t = - ((E12_Tt))\E11_Tt;
K_t = -inv(r)*b'*P_t;
phi_t = E11_t - E12_t*(E12_T)\(E11_T);
% P_t = eH_t;
% K_t = E11_t;
% phi_t = E12_t;

