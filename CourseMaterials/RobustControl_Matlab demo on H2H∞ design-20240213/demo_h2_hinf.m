%####################################
%  MATLAB DEMO - MIXED H2/HINF DESIGN
%####################################
%
clear variables
close all
%
% data
k=1;   % spring constant
m1=1;  % mass 1
m2=2;  % mass 2
A=10;  % bound for disturbance rejection
%
%#############
% First tuning
%#############
%
% standard problem
[a,b,c,d]=linmod('standard_form_1');
P=ss(a,b,c,d);
%
% H2/Hinf design
% second order controller
K=ltiblock.ss('K',2,1,1); % closed-loop cannot be stabilized with 1st order controller
CL=lft(P,K);
CL.y={'z1','z2'};
CL.u={'w1','w2'} ; 
% requirements
ReqH2=TuningGoal.Variance('w1','z1',1); % soft constraint: minimize H2 norm
ReqHinf=TuningGoal.Gain('w2','z2',1);   % hard constraint: Hinf norm must be < 1
% design
opt=systuneOptions('RandomStart',3);
[CLopt,fBest,gBest]=systune(CL,ReqH2,ReqHinf,opt);
% analysis
Kopt=ss(CLopt.Blocks.K);
zpk(Kopt)                   % high frequency zero (-145.9) out of the system bandwidth
norm(CLopt('z1','w1'),2)    % 0.163
norm(CLopt('z2','w2'),inf)  % 0.999
damp(CLopt)                 % 2 poles very badly damped, 2 poles badly damped
figure,hold on,grid on
bode(Kopt)
sisotool(P(3,3),-Kopt)      % gain margin = 3.6dB, phase margin = 20.6deg
%
%#################################################
% Second tuning with the modified standard problem
%#################################################
%
% modified standard problem (fast pole of pseudo-derivator removed)
[a,b,c,d]=linmod('standard_form_2');
P=ss(a,b,c,d);
%
% H2/Hinf desin
% second order controller
K=ltiblock.tf('K',1,2); % to ensure that D_K=0 and remove the high frequency zero
CL=lft(P,K);
CL.y={'z1','z2'};
CL.u={'w1','w2'};
% requirements
ReqH2=TuningGoal.Variance('w1','z1',1); % soft
ReqHinf=TuningGoal.Gain('w2','z2',1);   % hard 
% design
opt=systuneOptions('RandomStart',3);
[CLopt,fBest,gBest]=systune(CL,ReqH2,ReqHinf,opt);
% analysis
Kopt=ss(CLopt.Blocks.K);
zpk(Kopt)                   % almost same controller as before but no high frequency zero
norm(CLopt('z1','w1'),2)    % 0.164
norm(CLopt('z2','w2'),inf)  % 0.999
damp(CLopt)                 % 2 poles very badly damped, 2 poles badly damped
bode(Kopt)                  % better attenuation at high frequency
sisotool(P(3,3),-Kopt)      % gain margin = 3.6dB, phase margin = 20.6deg
%
%############################################
% Third tuning with better gain/phase margins
%############################################
%
% modified standard problem (fast pole of pseudo-derivator removed)
[a,b,c,d]=linmod('standard_form_2');
P=ss(a,b,c,d);
%
% H2/Hinf desin
% second order controller
K=ltiblock.tf('K',1,2);    % to ensure that D_K=0 (not better with order 3)
AP=AnalysisPoint('input'); % single-channel analysis point
CL=lft(P*mdiag(1,1,AP),K); % margins should be computed at the plant input (u)
CL.y={'z1','z2'};
CL.u={'w1','w2'};
% requirements
ReqH2=TuningGoal.Variance('w1','z1',1); % soft
ReqHinf=TuningGoal.Gain('w2','z2',1);   % hard 
ReqMargins=TuningGoal.Margins('input',5,35);   % hard (does not work with 40deg)
% design
opt=systuneOptions('RandomStart',3);
[CLopt,fBest,gBest]=systune(CL,ReqH2,[ReqHinf ReqMargins],opt);
% analysis
Kopt=ss(CLopt.Blocks.K);
norm(CLopt('z1','w1'),2)    % 0.297
norm(CLopt('z2','w2'),inf)  % 0.999
damp(CLopt)                 % 2 poles very badly damped, 2 poles badly damped
sisotool(P(3,3),-Kopt)      % gain margin = 6.5dB, phase margin = 35.2deg
%
%##################################
% Fourth tuning with better damping
%##################################
%
% modified standard problem (fast pole of pseudo-derivator removed)
[a,b,c,d]=linmod('standard_form_2');
P=ss(a,b,c,d);
%
% H2/Hinf desin
% third order controller
K=ltiblock.tf('K',2,3);    % to ensure that D_K=0 (not better with order 4)
AP=AnalysisPoint('input'); % single-channel analysis point
CL=lft(P*mdiag(1,1,AP),K); % margins should be computed at the plant input (u)
CL.y={'z1','z2'};
CL.u={'w1','w2'};
% requirements
ReqH2=TuningGoal.Variance('w1','z1',1); % soft
ReqHinf=TuningGoal.Gain('w2','z2',1);   % hard 
ReqMargins=TuningGoal.Margins('input',5,30);   % hard (slighty less than before)
ReqPoles=TuningGoal.Poles(0,0.27,Inf);  % hard
% design
opt=systuneOptions('RandomStart',3);
[CLopt,fBest,gBest]=systune(CL,ReqH2,[ReqHinf ReqMargins ReqPoles],opt);
% analysis
Kopt=ss(CLopt.Blocks.K);
norm(CLopt('z1','w1'),2)    % 0.709
norm(CLopt('z2','w2'),inf)  % 0.998
damp(CLopt)                 % 2x2 poles with damping 0.27 => less oscillations
sisotool(P(3,3),-Kopt)      % gain margin = 5.0dB, phase margin = 32.2deg
