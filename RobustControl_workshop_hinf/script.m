%############################################################
%  MATLAB WORKSHOP - CONTROL OF AN UNSTABLE AEROSPACE VEHICLE
%############################################################
%
clear variables
close all
%
% First requirement: the transfer between the reference input and the error must
% be small at low frequency to have a low steady-state error (see Figure 1). The
% controller K(s) must be computed so that ||W1*S||_inf < 1, where W1(s) =
% 1/Sdesired(s) = (s+1)/s. A weighting functions cannot have a pole on the
% imaginary axis, so a regularization is performed: W1(s) = (s+1)/(s+0.001).
%
% Second requirement: robustness w.r.t. an additive uncertainty must be ensured
% (see Figure 2). The controller K(s) must be computed so that ||Wa*K*S||_inf < 1,
% where Wa(s) = 1/KSdesired(s) = (s+3)/300. A transfer function cannot have less
% poles than zeros, so a regularization is performed: Wa(s) = (s+3)/(s+300).
%
% CASE WHERE ONLY Y IS AVAILABLE
%%
% weighted standard form
[A,B,C,D]=linmod('standard_form_1');  % exogenous inputs w and outputs z first
P=ss(A,B,C,D);
damp(P)
% => the order of P is 4 (2 for G + 1 for W1 + 1 for W2)
%
% full-order controller design with hinfsyn
% the Hinf norm of the transfer w->[z1;z2] is minimized
[K1,CL,gamma]=hinfsyn(P,1,1);  % 1 measured output and 1 control input
% => gamma=1.49
damp(K1)
% => the order of K1 is 4
norm(CL(1,1),'inf') % transfer between w and z1 => Hinf norm = 1.49
norm(CL(2,1),'inf') % transfer between w and z2 => Hinf norm = 1.10
% => the requirements are not satisfied
%
% closed-loop simulation
K=K1;
sim('closed_loop_1');
% => big overshoot and low steady-state error
%%
% analysis
G=tf(1,[1 0 -1]);
W1=tf([1 1],[1 0.001]);
Wa=tf([1 3],[1 300]);
% 1. first requirement: sensitivity function
S=inv(1+G*K1);
figure,hold on
sigma(1/W1) % template Sdesired in blue
sigma(S)    % sensitivity function S in red
grid on
figS=gcf;
% => S > Sdesired for some frequencies because gamma>1
figure,hold on
sigma(W1*S)
sigma(W1*S)
grid on
figWS=gcf;
% => the maximum difference over frequency is equal to norm(CL(1,1),'inf')=1.49 (=3.46dB)
% 2. second requirement: additive uncertainty rejection
figure,hold on
sigma(1/Wa)  % template KSdesired in blue
sigma(K1*S)  % K*S in red
grid on
figKS=gcf;
% => K*S > KSdesired for some frequencies because gamma>1
figure,hold on
sigma(Wa*K*S)
sigma(Wa*K*S)
grid on
figWKS=gcf;
% => the maximum difference over frequency is equal to norm(CL(2,1),'inf')=1.10 (=0.83dB)
%
% reduced-order design with hinfstruct
% the Hinf norm of the transfer w->[z1;z2] is minimized
% 1. static controller
K0=ltiblock.gain('K0',1,1);
opt=hinfstructOptions('randomstart',3);
[Kf,gamma]=hinfstruct(P,K0,opt);
% => fail to enforce closed-loop stability
% 2. first-order controller
K0=ltiblock.ss('K0',1,1,1);
opt=hinfstructOptions('randomstart',3);
[Kf,gamma]=hinfstruct(P,K0,opt);
% => stabilizes the system but gamma is very high (33.4)
K=ss(Kf);
sim('closed_loop_1');
% => non-negligible steady-state error (~3.5%) => bad performance
% => many oscillations => bad robustness
% 3. second-order controller
K0=ltiblock.ss('K0',2,1,1);
opt=hinfstructOptions('randomstart',3);
[Kf,gamma]=hinfstruct(P,K0,opt);
% => gamma is still very high (14.6) and much larger than in the full-order case
K=ss(Kf);
sim('closed_loop_1');
% => bad performance and robustness
% 4. third-order controller
K0=ltiblock.ss('K0',3,1,1);
opt=hinfstructOptions('randomstart',3);
[Kf,gamma]=hinfstruct(P,K0,opt);
% => same value of gamma as in the full-order case (1.49)
K2=ss(Kf);
K=K2;
sim('closed_loop_1');
% => same behavior as in the full-order case, but order 3 instead of 4
%
% analysis
% 1. first requirement: sensitivity function 
S=inv(1+G*K2);
figure(figS)
sigma(S)    % sensitivity function S in orange
figure(figWS)
sigma(W1*S)
% => very close to the full-order case
% 2. second requirement: additive uncertainty rejection
figure(figKS)
sigma(K2*S)  % K*S in orange
figure(figWKS)
sigma(Wa*K*S)
% => very close to the full-order case except at high frequency
%
% It could be guessed that the controller order must be at least equal to 3:
%  * ||S||_inf < s/(s+1) means that S(jw)~jw (derivator) at low frequency => K(s) must have
%    an integrator, i.e. K(s)=Kt(s)/s. This ensures that S(s) = 1/(1+K(s)*G(s)) =
%    s/(s+Kt(s)*G(s)) is equivalent to s/(Kt(0)*G(0)) when s tends to 0 (low frequency).
%  * The system is very unstable, since G(s) is open-loop unstable and there is an
%    integrator in K(s). A first way to improve stability is to add a proportional
%    integral controller (1+tau1*s)/s to K(s) instead of just an integrator. A second way
%    is to add a derivative feedback which has a stabilizing effect. But ydot is not
%    measured, so a pseudo-derivator s/(1+tau*s) of y should be added to K(s). Both
%    contributions are required here to stabilize the system and have a satisfactory
%    phase margin. So at this stage, K(s) is a linear combination of (1+tau1*s)/s and 
%    s/(1+tau*s), i.e. the structure of K(s) is (a_2*s^2+a1*s+a0)/(s*(1+tau*s)).
%  * S(jw)~1 at high frequency, so ||KS||_inf < 300/(s+3) means that deg(den(K(s)) = 
%    deg(numK)+1 = 3 (roll-off). A third pole must thus be added to K(s), which is
%    finally a transfer function with degree 2 in the numerator and 3 in the denominator.
%
% confirmation with hinfstruct
K0=ltiblock.tf('K0',2,3);  % K(s) has 2 zeros and 3 poles => relative degree of 1
K0.den.Value(end)=0;  % the constant term of the denominator is set to zero => integrator
K0.den.Free(end)=false;  % this term is considered as non tunable
opt=hinfstructOptions('randomstart',3);
[Kf,gamma]=hinfstruct(P,K0,opt);
% => same value of gamma as in the full-order case (1.50)
K3=ss(Kf);
tf(K3)
K=K3;
sim('closed_loop_1');
% => same behavior as in the full-order case
%
% reduced-order design with hinfstruct
% the Hinf norm of both transfers w->z1 and w->z2 is minimized (less conservative than
% minimizing the Hinf norm of the single transfer w->[z1;z2])
CL0=lft(P,K0);
CL1=CL0(1,1);           % mixed-sensitivity      W_1*S
CL2=CL0(2,1);           % uncertainty rejection  W_a*K*S
CL0f=blkdiag(CL1,CL2);  % concatenation of both Hinf constraints
options=hinfstructOptions('RandomStart',3);
[CL,gamma]=hinfstruct(CL0f,options);
% => better value of gamma than in the full-order case (1.39)
norm(CL(1,1),'inf') % transfer between w and z1 => Hinf norm = 1.39
norm(CL(2,2),'inf') % transfer between w and z2 => Hinf norm = 1.38
% => the requirements are not satisfied, but there is a better balance
K4=ss(CL.Blocks.K0);
tf(K4)
K=K4;
sim('closed_loop_1');
% => same behavior as in the full-order case
%
% analysis
% 1. first requirement: sensitivity function 
S=inv(1+G*K4);
figure(figS)
sigma(S)    % sensitivity function S in purple
figure(figWS)
sigma(W1*S)
% => better than in the full-order case
% 2. second requirement: additive uncertainty rejection
figure(figKS)
sigma(K4*S)  % K*S in purple
figure(figWKS)
sigma(Wa*K*S)
% => worse than in the full-order case
%
% standard form without the weighting functions
[A,B,C,D]=linmod('standard_form_2');  % exogenous inputs w and outputs z first
P0=ss(A,B,C,D);
damp(P0)
% reduced-order design with systune
% the Hinf norm of both transfers w->z1 and w->z2 is minimized
CL0=lft(P0,K0);
CL0.u='w'; 
CL0.y={'z1','z2'};
% requirements
% instead to including the weighting functions W1=1/Sdesired and W2=1/KSdesired in
% the standard form, the templates Sdesired and KSdesired are directly specified
% when defining the requirements => regularization is no longer required
Req1=TuningGoal.Gain('w','z1',tf([1 0],[1 1]));
Req2=TuningGoal.Gain('w','z2',tf(300,[1 3]));
% design
[CL,fBest,gBest]=systune(CL0,[],[Req1 Req2]); % two hard goals and no soft goal
norm(W1*CL(1,1),'inf') % transfer between w and z1 => Hinf norm = 1.39
norm(Wa*CL(2,1),'inf') % transfer between w and z2 => Hinf norm = 1.38
% => the above results are recovered
K5=ss(CL.Blocks.K0);
damp(K5)
K=K5;
sim('closed_loop_1');
% => same behavior as in the full-order case
%
% analysis
figure
viewSpec([Req1 Req2],CL)
%
% CASE WHERE BOTH Y AND YDOT ARE AVAILABLE
%
% standard form without the weighting functions
[A,B,C,D]=linmod('standard_form_3');  % exogenous inputs w and outputs z first
P0=ss(A,B,C,D);
%
% full-order controller design with hinfsyn
% the Hinf norm of the transfer w->[z1;z2] is minimized
% weighted standard form
P=[W1 0 0 0;0 Wa 0 0;0 0 1 0;0 0 0 1]*P0;
% call to the design routine
[K6,CL,gamma]=hinfsyn(P,2,1);  % 2 measured outputs and 1 control input
% => gamma=1.12
damp(K6)
% => the order of K6 is 4
%
% design of a structured controller with systune, composed of:
%  * a derivative feedback throught a gain (D) on ydot,
%  * a PI controller (PI) on the error signal,
%  * a first-order filter (Filter) on the control signal (roll-off)
PI0=ltiblock.tf('PI',1,1);  % one pole and one zero
PI0.den.Value(end)=0;  % the constant term of the denominator is set to zero => integrator
PI0.den.Free(end)=false;  % this term is considered as non tunable
D0=ltiblock.gain('D',1,1);  % static gain
Filter0=ltiblock.tf('Filter',0,1);  % one pole and no zero => 1/(1+tau*s)
K0=Filter0*[PI0 D0];  % second-order controller with 2 inputs and 1 output
CL0=lft(P0,K0);
CL0.u='w';
CL0.y={'z1','z2'};
[CL,fBest,gBest]=systune(CL0,[],[Req1 Req2]);
% => gamma=1.09 => not bad, requirements almost satisfied
% => better than full-order controller with only two poles instead of 4
PI=tf(CL.blocks.PI);
D=tf(CL.blocks.D);
Filter=tf(CL.blocks.Filter);
sim('closed_loop_2');
damp(CL)
% => very slow step response due to a very slow pole at -0.07
% => addition of a constraint on the closed-loop dynamics
Req3=TuningGoal.Poles(1,0,Inf);
[CL,fBest,gBest]=systune(CL0,[],[Req1 Req2 Req3]);
% => gamma=1.17 => slightly larger than before but still better than without ydot
PI=tf(CL.blocks.PI);
D=tf(CL.blocks.D);
Filter=tf(CL.blocks.Filter);
sim('closed_loop_2');
damp(CL)
% => the settling time is restored and the overshoot is reduced (20% instead of 60%)
