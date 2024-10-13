close all
clear
%% Exercise 1
A = [ -1.417 , 1 ; 2.86 , -1.183]
B = [0 ; -3.157]
%% 1. Compute the open loop poles of the natural aircraft
%-----------------------
lambda = eig(A)
%% 2. What are the values of matrices R and Q ?
%Check that Q = Q T 0 and R = R T > 0
%-----------------------
R = 5 %R = R^T > 0
Q = [0, 0; 0, 1] %Q = Q^T >= 0
%% 3. What is the value of matrix N which is defined by
%Q = N^T N
%-----------------------
N = [0 1]
Q = N'*N
%% 4. Check that (A, B) is controllable and that (A, N ) is observable
%-----------------------
%We may use the Kalman test to check the controllability of the system
rank(ctrb(A,B)) %rank([B, A*B])
%we may use the Kalman test to check the observability of the system
rank(obsv(A,N)) %rank([N; N*A])
%% 5.Compute the positive solution of the algebraic Ricatti
%equation P thanks to the eigenvectors of the Hamiltonian
%matrix as well the optimal gain K.
%Check that you get the same result either with scilab (command 'ricc')
%or matlab (command 'care')
%-----------------------
%Hamiltonian matrix
H = [A, -B*inv(R)*B'; -Q, -A']
[V,D] = eig(H)
%The eigenvalues in the matrix D are symmetric with respect
%to the imaginary axis
%The eigenvectors assiciated with the eigenvalues in the
%left half plane are the following:
X12=V(:,1:2)
X1=X12(1:2,:)
X2=X12(3:4,:)
P12=X2*inv(X1)
%P: solution of the algebraic Riccati equation
%L are the closed-loop eigenvalues
%K is the feebback gain matrix
[P,lambdaFTBF,K] = care(A,B,Q,R)
%P is the same than P12
%K is the same than B'*inv(R)*P
K
B'*inv(R)*P
%lambdaFTBF are the eigenvalues of A-B*K
%They are the same than the eigenvalues of the Hamiltonian
%matrix H in the left half plane
D
lambdaFTBF
eig(A-B*K)
%% 6.Plot the Bode diagram of K(sI A)^{-1} B ('open loop')
%and compute the phase margin
%-----------------------
%dot x = A x + B u; y = N x => G(s) = N (sI-A)^-1 B
%To obtain the Bode plot of K (sI-A)^-1 B
%we just change N by K
KPhiB = ss(A,B,K,0);
%[num,den] = ss2tf(A,B,N,0,1)
figure; bode(KPhiB); grid;
[Gm,Pm,Wcg,Wcp] = margin(KPhiB)
Gm_dB = 20*log10(Gm)
%% 7. Plot the Nyquist diagram of K (sI A)^{-1} B ('open loop')
%-----------------------
figure; nyquist(KPhiB);

%% Exercise 2
A = [-1.417 1 ; 2.86 -1.183];
B = [0; -3.157];

% Q1
R = 1000;
% q^2 = x'Q x
% q^2 = [alpha q]' [0 0; 0 1] [alpha q]
% Q = [0 0; 0 1];
N = [0 1]; %Q = N'*N
Q = N'*N;
[K, P, CLeig] = lqr(A,B,Q,R)

%%

%Q2. check Kalman Equality
   
%computation of N(S) and D(S)
[Ns, Ds] = ss2tf(A, B, N, 0)
[Nms, Dms] = ss2tf(-A, B, -N, 0)

%computation of beta(s)
[~, betas] = ss2tf(A-B*K, B, N, 0)
%computation of beta(-s)
[~, betams] = ss2tf(-(A-B*K), B, -N, 0)

%Kalman equality
conv(betas, betams) -conv(Ds, Dms) + 1/R*conv(Ns,Nms) %shall be zero!
%%
% Q3. compare poles of cl (A-BK) with poles of ol A
% which are the roots of det (sI − A + BK),
%and compare then with the poles of the open loop system, which are the roots
% of det (sI − A).
eig(A)
eig(A-B*K)
%%
% Q4. Chang-Letov design procedure
Gs = tf(Ns, Ds)
Gms = tf(Nms, Dms)
figure
rlocus(Gms*Gs)
%when R is very high, tends towards open loop poles
%to get dominant pole = -1, the corresponding gain k = -.681
kp = 0.681
[K,P,CLeig] = lqr(A,B,Q,1/kp)
%a robust methodology to place poles, better in this aspect in comparison
%to pole placement technique
%%

%Q5. 
R = 0.01;
[K,P,CLeig] = lqr(A,B,Q,R)
roots(Ns)
%if R v.small, tends towards zeros of open loop

% Q that makes poles faster than 2 rad/s
alpha = 2; %rad/s
R = 5; %no influence, since Q=0
[P, ~, ~] = care(A+alpha*eye(2,2),B,zeros(2,2),R)
K = inv(R)*B'*P
eig(A-B*K) %shall be to the left of -2!

%step response without prefilter
C = [0 1] %to get q as the output
[numF, denF] = ss2tf(A-B*K, B, C, 0)
F = tf(numF, denF)
figure
step(F)
hold on
% not very good response, since final value should be = 1
%prefilter
Pf = -inv(C*inv(A-B*K)*B)
[numFwithPf, denFwithPf] = ss2tf(A-B*K, B*Pf, C, 0)
FwithPf = tf(numFwithPf, denFwithPf)
step(FwithPf)

% Integral action
Ae = [A, zeros(2,1); -C, 0]
Be = [B; 0]
[Ppi, ~, ~] = care(Ae+alpha*eye(3,3), Be, zeros(3,3), R)
Kpi = inv(R)*Be'*Ppi
eig(Ae-Be*Kpi) %shall be to the left of -alpha!
Kp = Kpi(1:2)
Ki = Kpi(end)
[numFpi, denFpi] = ss2tf([A-B*Kp -B*Ki; -C 0], [zeros(2,1); 1], [C 0], 0)
Fpi = tf(numFpi, denFpi)
step(Fpi)
legend('F', 'FwithPF', 'Fint')