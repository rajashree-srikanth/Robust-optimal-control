%% Q1 - State space representation of the system
Torb = 5400; %in seconds
T = 0.25*Torb;
omega = (2*pi)/(Torb); %in rad/s
A = [[0 0 1 0]; [0 0 0 1]; [3*omega^2 0 0 -2*omega]; [0 0 2*omega 0]];
B = zeros(4,2);
B(3,1) = 1;
B(4,2) = 1;
A
B

%% Q2 - Controllability
n_allip = rank(ctrb(A,B)) %rank of matrix when both inputs are available
%n_allip = 4
B_phiz = B;
B_phiz(4,2) = 0;
n_phiz = rank(ctrb(A,B_phiz)) %rank of matrix is only phi_z is available
%n_phiz = 3
B_phix = B;
B_phix(3,1) = 0; %matrix that will be used for the computation henceforth
n_phix = rank(ctrb(A,B_phix)) %rank of matrix is only phi_x is available
%n_phix = 4
%assuming phi_x as the only control input!

%% Q3 - Optimal Control
trajX = zeros(4,max(size(1:T)));
trajU = zeros(4,max(size(1:T)));
t = 0; %initial time
a = A;
n = 4;
b = B_phix(:,2);
q = zeros(4,4);
r = 1;
%initial conditions
x0 = [0; 1000; 0; 0];
for t=2:2
    [K_t,P_t,phi_t]=twopbvp(T,t,a,b,q,r)
%     phi_t(1:n,1:n)
%     x_t = phi_t*x0;
%     trajX(:,t) = x_t;
%     u_t = K_t*x_t;
%     trajU(:,t) = u_t;
end
% phi_t
% plotresults(t,trajX,trajU)


