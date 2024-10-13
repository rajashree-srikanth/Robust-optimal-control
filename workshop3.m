%% Workshop 3
KA = [2.1608, 0.1027, 0.0357, 0.5607, 2.1999, 3.8429, 2.0283, 1];
KB = [3.4329, 0.1627, 0.1139, 0.2539, 1.7993, 1.8078, 1.8201, 1];
KC = [2.1608, 0.1027, 0.0357, 0.5607, 1.7993, 3.8429, 1.8201, 0.1];
% [K1, K2, K3, K4, K5, K6, K7, 1/J]
% A = [-KA(7)*KA(8)+KA(5)*KA(8), KA(4)*KA(8); -KA(3), -KA(2)];
% B = [0, KA(6)*KA(8); KA(1), 0];
% C = eye(2);
% D = zeros(0,0)
% Msys = ss(A,B,C,D)
G = [0.0081, 0.0872, 0.1586, -0.1202; 0.0187, 0.1826, 0.0848, -0.0224];
d1=gss('delta1',[],[2.1608, 3.4329])
d2=gss('delta2',[],[-0.1627,-0.1027])
d3=gss('delta3',[],[-0.1139,-0.0357])
d4=gss('delta4',[], [0.2539, 0.5607])
d5=gss('delta5', [], [-0.2293, 0.38])
d6 = gss('delta6', [], [1.8078, 3.8429])
d7 = gss('delta7', [], [0.1, 1])

%% 1.1
A = [d5*d7, d4*d7; d3, d2]
B = [0, d6*d7; d1, 0];
C = eye(2);
D = zeros(2,2)
sysOL = ss(A,B,C,D)

%% 1.2

[Ac, Bc, Cc, Dc] = linmod('closed_loop_model')
sys_control = ss(Ac,Bc,Cc,Dc)

%% 1.3
sysCL = feedback(sysOL, sys_control)
% order of M = 4 since there are 2 states in the plant + 2 states of the
% controller
sysCL_D = sysCL.D
% the parameter size refers to the no. of times each delta is repeated in
% the ss representation (the diagonalized representation of the
% uncertainity matrix
%% 2.1
% evaluating closed loop system at operating points A, B, C
deltaA = {KA(1), -KA(2), -KA(3), KA(4), KA(5)-KA(7) KA(6) KA(8)}
deltaB = {KB(1), -KB(2), -KB(3), KB(4), KB(5)-KB(7) KB(6) KB(8)}
deltaC = {KC(1), -KC(2), -KC(3), KC(4), KC(5)-KC(7) KC(6) KC(8)}
sysA = eval(sysCL, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, deltaA)
damp(sysA.A)
sysB = eval(sysCL, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, deltaB)
damp(sysB.A)
sysC = eval(sysCL, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, deltaC)
damp(sysC.A)

%% 2.2
[sys, sample] = dbsample(sysCL, 1000)
figure;
hold;
for i=1:1000
    ev = eig(sys{i});
    plot(real(ev), imag(ev), 'kx')
end

%% 2.3

sysCL_new = sysCL([],[]) %closed loop system without inputs and outputs

optub.lmi = 1; 
optub.tol = 0;
ubnd = muub(sysCL_new, [], optub);
[lbnd, wc, pert, iodesc] = mulb(sysCL_new);
%% 2.3(c)
delta_lbnd = {iodesc{1}.value, iodesc{2}.value, iodesc{3}.value, iodesc{4}.value, iodesc{5}.value, iodesc{6}.value, iodesc{7}.value}
sysCL_lbnd = eval(sysCL, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, delta_lbnd)
damp(sysCL_lbnd)
% mu analysis is better since it is not random
k_lbnd = 1/ubnd
k_ubnd = 1/lbnd

%% 3.1
% K5, K6 are changed

G = [0.0081, 0.0872, 0.1586, -0.1202; 0.0187, 0.1826, 0.0848, -0.0224];
d1=gss('delta1',[],[2.1608, 3.4329])
d2=gss('delta2',[],[-0.1627,-0.1027])
d3=gss('delta3',[],[-0.1139,-0.0357])
d4=gss('delta4',[], [0.2539, 0.5607])
d5=gss('delta5', [], [1.7993-2.0283, 2.0183-1.8201])
d6 = gss('delta6', [], [2.0247, 4.4962])
d7 = gss('delta7', [], [0.1, 1])

%
A = [d5*d7, d4*d7; d3, d2]
B = [0, d6*d7; d1, 0];
C = eye(2);
D = zeros(2,2)
sysOL = ss(A,B,C,D)

[Ac, Bc, Cc, Dc] = linmod('closed_loop_model')
sys_control_adjust = ss(Ac,Bc,Cc,Dc)
sysCL_adjust = feedback(sysOL, sys_control_adjust)

%% 3.2

sysCL_adjust_new = sysCL_adjust([],[]) %closed loop system without inputs and outputs

optub.lmi = 1; 
optub.tol = 0;
ubnd = muub(sysCL_adjust_new, [], optub);
[lbnd, wc, pert, iodesc] = mulb(sysCL_adjust_new);

%%

delta_lbnd = {iodesc{1}.value, iodesc{2}.value, iodesc{3}.value, iodesc{4}.value, iodesc{5}.value, iodesc{6}.value, iodesc{7}.value}
sysCL_lbnd = eval(sysCL_adjust_new, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, delta_lbnd)
damp(sysCL_lbnd)
k_lbnd_adjust = 1/ubnd
k_ubnd_adjst = 1/lbnd
k_lbnd
k_ubnd

%%  Q4
% 4.1
optbb.maxgap=0.05;
[lbnd,wc,pert,iodesc]=mubb(sysCL_adjust_new,[],optbb);
%%
delta_lbnd = {iodesc{1}.value, iodesc{2}.value, iodesc{3}.value, iodesc{4}.value, iodesc{5}.value, iodesc{6}.value, iodesc{7}.value}
sysCL_lbnd = eval(sysCL, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, delta_lbnd)
damp(sysCL_lbnd)
k_adbndjust_br = 1./lbnd
% since k_r >=1, the system is robustly stable

%% Q4.2

A = [d7*d5, d7*d4; d3, d2]
B = [0, d7*d6; d1, 0];
C = eye(2);
D = zeros(2,2)
sysOL_min = ss(A,B,C,D)

[Ac, Bc, Cc, Dc] = linmod('closed_loop_model')
sys_control_min = ss(Ac,Bc,Cc,Dc)
sysCL_min = feedback(sysOL_min, sys_control_min)
%%
sysCL_min = sysCL_min([],[]) %closed loop system without inputs and outputs

optub.lmi = 1; 
optub.tol = 0;
% ubnd = muub(sysCL_min, [], optub);
% [lbnd, wc, pert, iodesc] = mulb(sysCL_min);
optbb.maxgap=0.05;
[lbnd,wc,pert,iodesc]=mubb(sysCL_min,[],optbb);
%%

delta_lbnd = {iodesc{1}.value, iodesc{2}.value, iodesc{3}.value, iodesc{4}.value, iodesc{5}.value, iodesc{6}.value, iodesc{7}.value}
sysCL_lbnd = eval(sysCL_min, {'delta1', 'delta2', 'delta3', 'delta4', 'delta5', 'delta6', 'delta7'}, delta_lbnd)
damp(sysCL_lbnd)
k_ubnd_min = 1./lbnd