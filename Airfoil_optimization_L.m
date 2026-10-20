% =============================================================================
% AIRFOIL GEOMETRY + AERODYNAMIC ANALYSIS  (LABELED VERSION)
% =============================================================================
% Purpose:
%   1) Fit polynomial airfoil surfaces from LE radius, crest location, and TE slopes.
%   2) Compute section pressures, lift, and drag (subsonic thin-airfoil +
%      Prandtl-Glauert, or supersonic shock-expansion).
%   3) Sweep crest geometry to maximize L/D.
%   4) Check turbulent separation with Stratford's criterion (s_A vs s_B).
%
% Notes:
%   - Subsonic branch: M < 1.  Supersonic branch: M >= 1.
% =============================================================================

clc
clear
close all

%% ======================== 1. MISSION / FLOW INPUTS ========================
C_L=.1321;.5;                      % 3-D lift coefficient target
sweep=.9599;        % wing sweep [deg then rad]; last value used
sweep_xc=.5532;                    %Value of sweep at maximum airfoil thickness
e=.8534;                      % Oswald efficiency
AR=1.8236;                   % aspect ratio
C_d_0=0.008779;                        % profile / parasite Cd
i_max=200;                          % chordwise stations for force integration (make sure this ends in a zero)
xx_max=200;                         % stations for thin-airfoil integrals
y=1.4;                              % gamma (ratio of specific heats)
M=2;.78;.78;2;                  % Freestream Mach; first value used
a_inf=343;                          % Freestream air speed of sound
mu=1.789E-5;                        % Freestream air dynamic viscosity
rho=.3;              % Freestream air density 
T_inf=300;                          % Freestream air temperature
p_inf=.2*101300;                    % Freestream air pressure
Cp=1005;                            % Specific heat of air
c_root=27.66;                       % Root chord length
R=287.05;                           % Gas constant for air
D_f=3.1;                            % Fuselage diameter
b=25.52;                            % Wingspan
C_p_0=-1.2;                         %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil
x_cp0=.05;                          %chordwise fraction of zero sweep pressure coefficient
%% ======================== 2. AIRFOIL GEOMETRY CONTROLS ========================
%note: the inputs below represent dimensions relative to the chord length (x/c)
r_le = 0.017;                        % Wing leading-edge radius / chord
theta_le_u = 2*pi/180;3*pi/180;     % Upper surface leading edge angle
theta_te_u = -2*pi/180;-2*pi/180;   % Upper surface trailing edge angle
theta_le_l = -6*pi/180;-3*pi/180;   % Lower surface leading edge angle
theta_te_l = 2*pi/180;2*pi/180;     % Lower surface trailing edge angle

x_crest_u_min=0.25;                 %Lower bound best L/D max airfoil thickness x location (upper surface)
x_crest_u_max=0.4;                  %Upper bound best L/D max airfoil thickness x location (upper surface)
y_crest_u_min=0.005;                %Lower bound best L/D max airfoil thickness y location (upper surface) 
y_crest_u_max=.02;                  %Upper bound best L/D max airfoil thickness y location (upper surface)
x_crest_l_min=0.35;                 %Lower bound best L/D max airfoil thickness x location (lower surface)
x_crest_l_max=0.4;                  %Upper bound best L/D max airfoil thickness x location (lower surface)
y_crest_l_min=-0.01;               %Lower bound best L/D max airfoil thickness y location (lower surface)
y_crest_l_max=-.02;                 %Upper bound best L/D max airfoil thickness y location (lower surface)

%% ============================= For loop iteration parameters ======================== %%
N=20;                               % upper and lower surface iteration number for Prandtl-Meyer / shock relations
xxxx_max=15;                        % upper section iteration number for Prandtl-Meyer / shock relations
xxxxx_max=20;                       % lower section iteration number for Prandtl-Meyer / shock relations
xxxxxxx_max=15;                     % lower section iteration number for Prandtl-Meyer / shock relations
pp_max=7;                           % iteration number controlling x_crest_l
nn_max=7;                           % iteration number controlling y_crest_l
p_max=7;                            % iteration number controlling x_crest_u
n_max=7;                            % iteration number controlling y_crest_u


v_inf=1;                            % nondimensional freestream speed
dx=1/i_max;

CL_ww=[0 0.1 0.3 0.4 0.5 0.6 0.8]; %Raymer's method for induced drag
S_ww=[.91 .91 .925 .9275 .935 .935 .915]; %Raymer's method for induced drag

%% ======================== 3. CREST-GEOMETRY SEARCH LOOPS ========================
% Nested brute-force sweep of lower/upper crest (x,y) to maximize L/D.
% Loop order:  pp = x_crest_l , nn = y_crest_l , p = x_crest_u , n = y_crest_u
for pp=2:pp_max+2
x_crest_l=(x_crest_l_max-x_crest_l_min)*(pp-2)/pp_max+x_crest_l_min;    
% x_crest_l = .35;.2;.25;
for nn=2:nn_max+2
% y_crest_u = 0.0788;.005;
y_crest_l=(y_crest_l_max-y_crest_l_min)*(nn-2)/nn_max+y_crest_l_min;
% y_crest_l= -.015;-.01;-0.0423;-.02;
for p=2:p_max+2
x_crest_u=(x_crest_u_max-x_crest_u_min)*(p-2)/p_max+x_crest_u_min;
% x_crest_u=.4;.3;
for n=2:n_max+2
y_crest_u=(y_crest_u_max-y_crest_u_min)*(n-2)/n_max+y_crest_u_min;
% y_crest_u=.01;.005;.0788;.005;


%% ======================== 4. FIT POLYNOMIAL AIRFOIL SURFACES ========================
% Subsonic (M<1):  y = a1*x^(1/2) + a2*x + a3*x^2 + a4*x^3 + a5*x^4
%   a1 = +/-sqrt(2*r_le) enforces leading-edge radius.
%   Remaining coeffs match crest height, crest slope=0, TE height=0, TE slope.
% Supersonic (M>=1):  ordinary polynomial y = a1*x + ... + a5*x^5
%   (no square-root LE term; LE and TE slopes imposed directly).
if M<1

a1_u = sqrt(2*r_le);                % upper-surface LE radius term

% Matrix for unknowns [a2_u a3_u a4_u a5_u a6_u]'
M_u = [x_crest_u^(1), x_crest_u^(2), x_crest_u^(3), x_crest_u^(4);

    1, (2)*x_crest_u^(1), (3)*x_crest_u^(2), (4)*x_crest_u^(3);

    1, 1, 1, 1;
    
    1, 2, 3, 4];

MM_u = [y_crest_u - a1_u*sqrt(x_crest_u); -(1/2)*a1_u*x_crest_u^(-1/2); -a1_u; tan(theta_te_u) - (1/2)*a1_u];

% Solve
A_u = M_u \ MM_u;

a2_u = A_u(1);
a3_u = A_u(2);
a4_u = A_u(3);
a5_u = A_u(4);

% LOWER SURFACE

a1_l = -sqrt(2*r_le);

% Matrix for unknowns [a2_l a3_l a4_l a5_l]'
M_l = [x_crest_l^(1), x_crest_l^(2), x_crest_l^(3), x_crest_l^(4);

1, (2)*x_crest_l^(1), (3)*x_crest_l^(2), (4)*x_crest_l^(3);

1, 1, 1, 1;
    
1, 2, 3, 4];

MM_l = [y_crest_l - a1_l*sqrt(x_crest_l); -(1/2)*a1_l*x_crest_l^(-1/2); -a1_l; tan(theta_te_l) - (1/2)*a1_l];

% Solve
A_l = M_l \ MM_l;

a2_l = A_l(1);
a3_l = A_l(2);
a4_l = A_l(3);
a5_l = A_l(4);

else

% Matrix for unknowns [a2_u a3_u a4_u a5_u a6_u]'
M_u = [x_crest_u^(1), x_crest_u^(2), x_crest_u^(3), x_crest_u^(4), x_crest_u^(5);

1, (2)*x_crest_u^(1), (3)*x_crest_u^(2),(4)*x_crest_u^(3), (5)*x_crest_u^(4);

    1, 1, 1, 1, 1;
    
    1, 2, 3, 4, 5;
    
    1,0,0,0,0];

MM_u = [y_crest_u; 0; 0; tan(theta_te_u); tan(theta_le_u)];

% Solve
A_u = M_u \ MM_u;

a1_u = A_u(1);
a2_u = A_u(2);
a3_u = A_u(3);
a4_u = A_u(4);
a5_u = A_u(5);
% LOWER SURFACE

% Matrix for unknowns [a2_l a3_l a4_l a5_l]'
M_l = [ x_crest_l^(1), x_crest_l^(2), x_crest_l^(3), x_crest_l^(4), x_crest_l^(5);

    1, (2)*x_crest_l^(1), (3)*x_crest_l^(2), (4)*x_crest_l^(3), (5)*x_crest_l^(4);

    1, 1, 1, 1, 1;
    
    1, 2, 3, 4, 5;
    
    1, 0, 0, 0, 0];

MM_l = [ y_crest_l; 0; 0; tan(theta_te_l); tan(theta_le_l)];

% Solve
A_l = M_l \ MM_l;

a1_l = A_l(1);
a2_l = A_l(2);
a3_l = A_l(3);
a4_l = A_l(4);
a5_l = A_l(5);

end
    
%% ======================== 5. CHORDWISE INTEGRATION SETUP ========================
if M<1
beta=(1-M^2)^.5;
else
beta=(M^2-1)^.5;
end

if M<1


dx_xx=1/xx_max;
sum_u(1)=0;
sum_l(1)=0;
theta(1)=0;
sum2=0;
sum3=0;
sum4=0;
u(1)=0;
l(1)=0;
sum5(1)=0;
x(1)=0;
for xx=2:xx_max
x(xx)=1/xx_max*(xx-1);
u(xx)=a1_u*x(xx)^(1/2)+a2_u*x(xx)^(1)+a3_u*x(xx)^(2)+a4_u*x(xx)^(3)+a5_u*x(xx)^(4);
s_u(xx)=0.5*a1_u*x(xx).^(-1/2) + 1*a2_u*x(xx).^(0) + 2*a3_u*x(xx).^(1) + 3*a4_u*x(xx).^(2) + 4*a5_u*x(xx).^(3);
l(xx)=a1_l*x(xx).^(1/2)+a2_l*x(xx).^(1)+a3_l*x(xx).^(2)+a4_l*x(xx).^(3)+a5_l*x(xx).^(4);
s_l(xx)=0.5*a1_l*x(xx).^(-1/2)+1*a2_l*x(xx)^(0)+2*a3_l*x(xx)^(1)+3*a4_l*x(xx)^(2)+4*a5_l*x(xx)^(3);
s_camber(xx)=((u(xx)+l(xx))/2-(u(xx-1)+l(xx-1))/2)/dx_xx;
s_camber(xx) = 0.5*(s_u(xx) + s_l(xx));

theta(xx)=acos(1-2*x(xx));
d_theta=(theta(xx)-theta(xx-1));

dy2=s_camber(xx)*d_theta;
sum2=dy2+sum2;
dy3=s_camber(xx)*cos(theta(xx))*d_theta;
sum3=dy3+sum3;
dy4=s_camber(xx)*cos(2*theta(xx))*d_theta;
sum4=dy4+sum4;

% Thin-airfoil Fourier coefficients from camber-line slope
A1=2/pi*sum3;
A2=2/pi*sum4;
aoa_L0=1/pi*sum2-A1/2;              % zero-lift angle
M_n=M*cos(sweep);
L_s_2d = 2*pi/(1-M_n^2)^.5;            % 2-D lift slope with Prandtl-Glauert
eta_s = L_s_2d/(2*pi/beta);
F = 1.07*(1 + D_f/b)^2;
L_s_3d = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep)^2/beta^2)));
a_i=C_L/(pi*AR);
% L_s_3d=L_s_2d/(1+L_s_2d/(pi*AR*e)); % 3-D lift slope
aoa=C_L/L_s_3d+aoa_L0;              % geometric AoA needed for target CL
A0=aoa-1/pi*sum2;
% A0=C_L/L_s_3d-A1/2;                 % A0 overwritten to match target CL

if xx<xx_max
x_q(xx)=x(xx);
s_u_q(xx)=0.5*a1_u*x_q(xx).^(-1/2) + 1*a2_u*x_q(xx).^(0) + 2*a3_u*x_q(xx).^(1) + 3*a4_u*x_q(xx).^(2) + 4*a5_u*x_q(xx).^(3);
s_l_q(xx)=0.5*a1_l*x_q(xx).^(-1/2)+1*a2_l*x_q(xx)^(0)+2*a3_l*x_q(xx)^(1)+3*a4_l*x_q(xx)^(2)+4*a5_l*x_q(xx)^(3);
s_camber_q(xx) = 0.5*(s_u_q(xx) + s_l_q(xx));

s_t_u(xx)=(s_u_q(xx)-s_camber_q(xx));
q_u(xx)=v_inf*s_t_u(xx);

% if x_i(i)==x_q(xx)
% dsum5(xx)=0;
% else
% dsum5(xx)=(q_u(xx)/(x_i(i)-x_q(xx)))*dx_xx;
% end
% 
% sum5(xx)=dsum5(xx)+sum5(xx-1);
end

end

end

D_u=0;                              % running upper-surface "drag" integral (axial)
D_l=0;
L_u=0;                              % running upper-surface "lift" integral (normal)
L_l=0;
r(1)=0;
dx_i=1/i_max;
ppp=0;
v_n(1)=0;
p_u(1)=p_inf*(1+((y-1)/2)*M^2)^(y/(y-1));
p_u_B(1)=p_inf*(1+((y-1)/2)*M^2)^(y/(y-1));
% p_u_B(1)=p_inf;
Cpp(1)=1;
sum6(1)=0;
for i=2:(i_max-.01*i_max)
x_i(i)=1/i_max*(i-1)+.01;
% aoa=(aoa_high-aoa_low)/i_max*(i-2)+aoa_low;

%% ----- 5a. SUBSONIC BRANCH: thin-airfoil theory + Cp -----
if M<1

uu(i)=a1_u*x_i(i)^(1/2)+a2_u*x_i(i)^(1)+a3_u*x_i(i)^(2)+a4_u*x_i(i)^(3)+a5_u*x_i(i)^(4);
ll(i)=a1_l*x_i(i).^(1/2)+a2_l*x_i(i).^(1)+a3_l*x_i(i).^(2)+a4_l*x_i(i).^(3)+a5_l*x_i(i).^(4);
s_u(i)=0.5*a1_u*x_i(i).^(-1/2) + 1*a2_u*x_i(i).^(0) + 2*a3_u*x_i(i).^(1) + 3*a4_u*x_i(i).^(2) + 4*a5_u*x_i(i).^(3);
s_l(i)=0.5*a1_l*x_i(i).^(-1/2)+1*a2_l*x_i(i)^(0)+2*a3_l*x_i(i)^(1)+3*a4_l*x_i(i)^(2)+4*a5_l*x_i(i)^(3);
s_camber_i(i) = 0.5*(s_u(i) + s_l(i));

theta_B(i)=acos(1-2*x_i(i));
% Vortex-sheet strength (thin-airfoil theory)
gamma = (2*v_inf*( A0*(1+cos(theta_B(i)))/sin(theta_B(i)) + A1*sin(theta_B(i)) + A2*sin(2*theta_B(i)) ))/(1-M^2)^.5;
v_u(i)=v_inf+gamma/2;               % upper-surface edge speed
% v_u(i)=aoa-s_u(i);
% v_xu(i)=v_inf*(aoa-s_camber_i(i));
% v_xu_t(i)=(v_inf+v_xu(i));
% V_xu_tt(i)=v_xu_t(i)*cos(aoa);
% v_yu(i)=v_xu_t(i)*tan(s_camber_i(i));
% v_u(i)=(v_xu_t(i)^2+v_yu(i)^2)^.5;

gamma_air = y;                      % 1.4
V_ratio_u = v_u(i) / v_inf;
T_ratio_u = 1 + 0.5*(gamma_air-1)*M^2*(1 - V_ratio_u^2);

% Vacuum / max-speed limiter (T cannot go negative)
if T_ratio_u < 1e-8
    T_ratio_u = 1e-8;
end

p_ratio_u = T_ratio_u^(gamma_air/(gamma_air-1));
Cp_u(i) = (2/(gamma_air*M^2)) * (p_ratio_u - 1);
Re_x=rho*M*a_inf*x_i(i)*c_root/mu;
slope(i)=s_u(i)-aoa;
Cp_u_min_0=1-(1+aoa/(2*r_le)^.5)^2;
Cp_u_min_0=1-(4*aoa^2/(r_le+aoa^2));
Cp_u_min=Cp_u_min_0/(1-M^2)^.5;

T_0 = T_inf*(1+(y-1)/2*M^2);
V_max_B = sqrt(2*Cp*T_0);
Cp_u_min_limit = 1-(V_max_B/(M*a_inf))^2;

aff=4*r_le;

if Cp_u_min>Cp_u(i) && Cp_u_min<0 && x_i(i)<.04
Cp_u(i)=Cp_u_min;
end

% Cd_u_B=1;.03667;
% theta_C=atan(s_u(i))-aoa;
% Cp_u_3(i)=2*(sin(theta_C))*abs(sin(theta_C));
% Cp_u_3(i)=2*(sin(theta_C))/(1-M^2)^.5;
% p0 = p_inf*(1+(y-1)/2*M^2)^(y/(y-1));
% Cp_u_3(i)=(p0-p_inf)*sin(theta_C)/(.5*y*p_inf*M^2); 
% p_u_C(i)=(Cp_u_3(i)+p_inf/(.5*rho*(M*a_inf)^2))*(.5*rho*(M*a_inf)^2); 

dD_u(i)=(Cp_u(i))*s_u(i)*dx;
dL_u(i)=-(Cp_u(i))*dx;
D_u=dD_u(i)+D_u;
L_u=dL_u(i)+L_u;
r(i)=(L_u)/(D_u);

v_l(i)=v_inf-gamma/2;
% v_xl(i)=v_inf*-(aoa-s_camber_i(i));
% v_xl_t(i)=(v_inf+v_xl(i));
% V_xl_tt(i)=v_xl_t(i)*cos(aoa);
% v_yl(i)=v_xl_t(i)*tan(s_camber_i(i));
% v_l(i)=(v_xl_t(i)^2+v_yl(i)^2)^.5;

theta_D=atan(-s_l(i))+aoa;
p0 = p_inf*(1+(y-1)/2*M^2)^(y/(y-1));
Cp_l_3(i)=(p0-p_inf)*sin(theta_D)/(.5*y*p_inf*M^2);
p_u_D(i)=(Cp_l_3(i)+p_inf/(.5*rho*(M*a_inf)^2))*(.5*rho*(M*a_inf)^2);

V_ratio_l = v_l(i) / v_inf;
T_ratio_l = 1 + 0.5*(gamma_air-1)*M^2*(1 - V_ratio_l^2);
if T_ratio_l < 1e-8
    T_ratio_l = 1e-8;
end
p_ratio_l = T_ratio_l^(gamma_air/(gamma_air-1));
Cp_l(i) = (2/(gamma_air*M^2)) * (p_ratio_l - 1);
dD_l(i)=-Cp_l(i)*s_l(i)*dx;
dL_l(i)=Cp_l(i)*dx;
D_l=dD_l(i)+D_l;
L_l=dL_l(i)+L_l;
r_l(i)=(L_l)/(D_l);

D_u_true(i)=(L_u)*sin(aoa)+(D_u)*cos(aoa);
L_u_true(i)=(L_u)*cos(aoa)-(D_u)*sin(aoa);
r_u_true(i)=L_u_true(i)/D_u_true(i);
D_l_true(i)=(L_l)*sin(aoa)+(D_l)*cos(aoa);
L_l_true(i)=(L_l)*cos(aoa)-(D_l)*sin(aoa);
C_l(i)=(L_u_true(i)+L_l_true(i));

if M<1
C_l_alpha=2*pi/(1-M_n^2)^.5;
eta_s = C_l_alpha/(2*pi/beta);
F = 1.07*(1 + D_f/b)^2;
C_L_alpha = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep_xc)^2/beta^2))); %Raymer
else
C_l_alpha=4/(M_n^2-1)^.5*cos(sweep); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.17 page 331
C_L_alpha=C_l_alpha*(1-1/(2*AR*(M_n^2-1)^.5)); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.18 page 331

end


K0=1/C_L_alpha;

if M>=1
K100=1/(pi*AR);
else
K100=1/(pi*AR*e);
end

lowerIdxcl = find(C_L >= CL_ww, 1, 'last');
if lowerIdxcl == length(CL_ww)
upperIdxcl = lowerIdxcl;
else
upperIdxcl = lowerIdxcl+1;
end

CL_ww_1=CL_ww(lowerIdxcl);
CL_ww_2=CL_ww(upperIdxcl);
S_ww_1=S_ww(lowerIdxcl);
S_ww_2=S_ww(upperIdxcl);
C_ww=-(S_ww_2-S_ww_1)/(CL_ww_2-CL_ww_1)*CL_ww_1+S_ww_1;
S_ww_t=(S_ww_2-S_ww_1)/(CL_ww_2-CL_ww_1)*C_L+C_ww;

if lowerIdxcl==length(CL_ww)
    S_ww_t=S_ww_2;
elseif upperIdxcl==1
    S_ww_t=S_ww_1;
end

K=S_ww_t*K100+(1-S_ww_t)*K0; %Raymer

c_d_i=K*C_L^2;


Cd_i=C_L^2/(pi*AR*e);
Cd_p=C_d_0;
r_i(i)=(L_u_true(i)+L_l_true(i))/(D_u_true(i)+D_l_true(i)+c_d_i+C_d_0);
Cl_t=L_u_true(i)+L_l_true(i);

% p_u(i)=(Cp_u(i)+p_inf/(.5*y*p_inf*(M)^2))*(.5*y*p_inf*(M)^2);
% p_min(i)=(Cp_u(i)+Cp_u_3(i)+p_inf/(.5*rho*(M*a_inf)^2))*(.5*rho*(M*a_inf)^2);  
% p_u_ii(i)=p_u(i)+(p_u_C(i)-p_inf);
% p_u_iii(i)=p_u(i);
% % p_min_min(i)=min(p_u_ii(2:end));
% p_min_min(i)=min(p_u(2:end));
% Cp_min(i)=min(Cp_u+Cp_u_3); %Adding the stagnation pressure (Cp_u_3) is an assumption
Cp_min=C_p_0*(cos(sweep))^2;

% --- Local Mach number ---
M_local = sqrt((2/(y-1)) * ((1 + ((y-1)/2)*M^2) * ((y*M^2/2)*Cp_u(i) + 1)^(-(y-1)/y) - 1 ) );
% --- Local temperature from isentropic relation ---
T_local = T_inf *(1 + ((y-1)/2)*M^2) / (1 + ((y-1)/2)*M_local^2);
% --- Local speed of sound ---
a_local = sqrt(y*R*T_local);
% --- Local velocity ---
V_local(i) = M_local*a_local;
% --- freestream speed of sound ---
a = sqrt(y*R*T_inf);
% --- Freestream velocity ---
V_inf = M*a;

% --- Local Mach number ---
M_cp0 = sqrt((2/(y-1)) * ((1 + ((y-1)/2)*M^2) * ((y*M^2/2)*Cp_min + 1)^(-(y-1)/y) - 1 ) );
% --- Local temperature from isentropic relation ---
T_cp0 = T_inf *(1 + ((y-1)/2)*M^2) / (1 + ((y-1)/2)*M_cp0^2);
% --- Local speed of sound ---
a_cp0 = sqrt(y*R*T_cp0);
% --- Local velocity ---
V_cp0 = M_cp0*a_cp0;

% if p_u_iii(i)==C_p_0
% iii=i;
% sum6(iii-1)=0;
% end

if x_i(i)<=x_cp0
iii=i;
dsum6(i)=(V_local(i)/V_cp0)^3*c_root*dx;
sum6(i)=dsum6(i)+sum6(i-1);
else
x_p(i)=sum6(iii)+(x_i(i)-x_cp0)*c_root;
Re_p=V_local(iii)*x_p(i)/mu;
Cpp(i)=1-(V_local(i)/V_cp0)^2;
dCpp(i)=Cpp(i)-Cpp(i-1);
if dCpp(i)<0
   dCpp(i)=0;
end
s_A(i)=Cpp(i)*(x_p(i)*dCpp(i)/(c_root*dx))^.5;
s_B(i)=.39*(Re_p/10^6)^.1;
end

else
    
    
%% ----- 5b. SUPERSONIC BRANCH: shock-expansion marching -----
M_n=M*cos(sweep);
L_s_2d=4/(M_n^2-1)^.5*cos(sweep); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.17 page 331
L_s_3d=L_s_2d*(1-1/(2*AR*(M_n^2-1)^.5)); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.18 page 331

aoa=C_L/L_s_3d;   

uu(i)=a1_u*x_i(i)^(1)+a2_u*x_i(i)^(2)+a3_u*x_i(i)^(3)+a4_u*x_i(i)^(4)+a5_u*x_i(i)^(5);
ll(i)=a1_l*x_i(i).^(1)+a2_l*x_i(i).^(2)+a3_l*x_i(i).^(3)+a4_l*x_i(i).^(4)+a5_l*x_i(i).^(5);
s_u(i)=a1_u + 2*a2_u*x_i(i).^(1) + 3*a3_u*x_i(i).^(2) + 4*a4_u*x_i(i)^(3) + 5*a5_u*x_i(i).^(4);
s_l(i)=a1_l+2*a2_l*x_i(i)^(1)+3*a3_l*x_i(i)^(2)+4*a4_l*x_i(i)^(3)+5*a5_l*x_i(i)^(4);

Bi=45;
M1=M;
beta_low=asin(1/M);
beta_high=pi/2;                     % shock-angle search bounds

ct=dx_i;
if i==2

if theta_le_u-aoa>0

beta_sh_u=(beta_low+beta_high)/2;
for xxxx=2:xxxx_max

% Oblique-shock theta-beta-M relation (upper LE)
wing_angle_u=atan(2*cot(beta_sh_u)*(M_n^2*(sin(beta_sh_u))^2-1)/(M_n^2*(y+cos(2*beta_sh_u))+2));

if wing_angle_u<(theta_le_u-aoa)
    beta_sh_u=beta_sh_u+(beta_high-beta_low)/2^xxxx;
else
    beta_sh_u=beta_sh_u-(beta_high-beta_low)/2^xxxx;
end

end

Mn1u = M_n*sin(beta_sh_u);
Mn2u = sqrt((1+((y-1)/2)*Mn1u^2) / (y*Mn1u^2-(y-1)/2));
M2u = Mn2u/sin(beta_sh_u-(theta_le_u-aoa));
M2u = Mn2u/cos(pi/2-beta_sh_u-aoa+theta_le_u);
vM2=((atan((((M2u^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2u^2)-1)^0.5)));

else
        
theta_A=theta_le_u-aoa; 
vM1=((atan((((M_n^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M_n^2)-1)^0.5)));
vM2(i)=-theta_A+vM1;
M2u=2*M/2; % M3 initial condition set to a very high value
M2f=2*M; % fixed M3 initial condition reduced in the for loop

for iiiiii = 2:N
    
    vM2i(i)=((atan((((M2u^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2u^2)-1)^0.5)));
    
    if vM2i(i)>vM2(i)
        M2u=M2u-(M2f/(2^iiiiii));
    else
        M2u=M2u+(M2f/(2^iiiiii));
    end
   
end

M2u=M2u;

vM2=((atan((((M2u^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2u^2)-1)^0.5)));

end
    
T01= T_inf*(1+(y-1)/2*M^2);
p01dp1u=(((y-1)/2)*(M^2)+1)^(y/(y-1));
p02dp2u=(((y-1)/2)*(M2u^2)+1)^(y/(y-1));

if theta_le_u-aoa>0
Mn1u = M_n*sin(beta_sh_u);
p02dp01 = (((y+1)*Mn1u^2)/((y-1)*Mn1u^2+2))^(y/(y-1))*((y+1)/(2*y*Mn1u^2-(y-1)))^(1/(y-1));
else
p02dp01=1;
end

p02 = p02dp01*p01dp1u;
p2u(i)=p02dp01/p02dp2u*p01dp1u;

Cpu1=(p2u(i)-1)/(.5*y*(M1^2));
Cnu1=-Cpu1*ct;
Cau1=Cpu1*s_u(2)*ct;

else

p3u(2)=p2u(2);
M3(2)=M2u;

% top right surface
theta_1=atan(s_u(i-1));
theta_2=atan(s_u(i));
theta_u(i)=theta_2-theta_1;

if theta_u(i)>0   
beta_sh_u_3=(beta_low+beta_high)/2;
for xxxx=2:xxxx_max

% --- Oblique-shock theta-beta-M relation ---
wing_angle_3_u=atan(2*cot(beta_sh_u_3)*(M3(i-1)^2*(sin(beta_sh_u_3))^2-1)/(M3(i-1)^2*(y+cos(2*beta_sh_u_3))+2));

if wing_angle_3_u<theta_u(i)
    beta_sh_u_3=beta_sh_u_3+(beta_high-beta_low)/2^xxxx;
else
    beta_sh_u_3=beta_sh_u_3-(beta_high-beta_low)/2^xxxx;
end

end

Mn2u3 = M3(i-1)*sin(beta_sh_u_3);
Mn3u = sqrt((1+((y-1)/2)*Mn2u3^2) / (y*Mn2u3^2-(y-1)/2));
M3(i) = Mn2u3/cos(pi/2-beta_sh_u_3+theta_u(i));
vM3(i)=((atan((((M3(i)^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M3(i)^2)-1)^0.5)));
p02dp2u3(i)=(((y-1)/2)*(M3(i-1)^2)+1)^(y/(y-1));
p03dp3u(i)=(((y-1)/2)*(M3(i)^2)+1)^(y/(y-1));
p02dp01 = (((y+1)*Mn2u3^2)/((y-1)*Mn2u3^2+2))^(y/(y-1))*((y+1)/(2*y*Mn2u3^2-(y-1)))^(1/(y-1));
p03(i) = p02dp01*p02dp2u3(i);
% p3u(i)=p03(i)/p03dp3u;
p02dp01u3(i) = (((y+1)*Mn2u3^2)/((y-1)*Mn2u3^2+2))^(y/(y-1))*((y+1)/(2*y*Mn2u3^2-(y-1)))^(1/(y-1));
p3u(i) = p02dp01u3(i)/p03dp3u(i)*p02dp2u3(i)*p3u(i-1);
p03(i)=p03dp3u(i)*p3u(i);
       
else
      
p03(2)=p02;    
vM3(2)=vM2;
vM3(i)=-theta_u(i)+vM3(i-1);
M3i=2*M1/2; % M3 initial condition set to a very high value
M3f=2*M1; % fixed M3 initial condition reduced in the for loop

for iiiiii = 2:N
    
    vM3i(i)=((atan((((M3i^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-...
        (atan(((M3i^2)-1)^0.5)));
    
    if vM3i(i)>vM3(i)
        M3i=M3i-(M3f/(2^iiiiii));
    else
        M3i=M3i+(M3f/(2^iiiiii));
    end
   
end

M3(i)=M3i;

p01dp1u=(((y-1)/2)*(M1^2)+1)^(y/(y-1));
p03dp3u=(((y-1)/2)*(M3(i)^2)+1)^(y/(y-1));
p03(i)=p03(i-1);
p3u(i)=p03(i)/p03dp3u;

end

Cpu2(i)=(p3u(i)-1)/(.5*y*(M1^2));
Cnu2(i)=-Cpu2(i)*ct;
Cau2(i)=Cpu2(i)*s_u(i)*ct;

end

ct=dx_i;

if i==2

if -theta_le_l+aoa>0    
beta_sh_l=(beta_low+beta_high)/2;
for xxxxx=2:xxxxx_max

% --- Oblique-shock theta-beta-M relation ---
wing_angle_l=atan(2*cot(beta_sh_l)*(M_n^2*(sin(beta_sh_l))^2-1)/(M_n^2*(y+cos(2*beta_sh_l))+2));

if wing_angle_l<(-theta_le_l+aoa)
    beta_sh_l=beta_sh_l+(beta_high-beta_low)/2^xxxxx;
else
    beta_sh_l=beta_sh_l-(beta_high-beta_low)/2^xxxxx;
end

end

Mn1l = M_n*sin(beta_sh_l);
Mn2l = sqrt((1+((y-1)/2)*Mn1l^2) / (y*Mn1l^2-(y-1)/2));
M2l = Mn2l/sin(beta_sh_l-(-theta_le_l+aoa));
M2l = Mn2l/cos(pi/2-beta_sh_l+aoa-theta_le_l);
vM2l=((atan((((M2l^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2l^2)-1)^0.5)));

else

theta_Al=-theta_le_l+aoa; 
vM1l=((atan((((M_n^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M_n^2)-1)^0.5)));
vM2l(i)=-theta_Al+vM1l;      
M2l=2*M/2; % M3 initial condition set to a very high value
M2f=2*M; % fixed M3 initial condition reduced in the for loop

for iiiiii = 2:N   
    vM2il(i)=((atan((((M2l^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2l^2)-1)^0.5)));  
    if vM2il(i)>vM2l(i)
        M2l=M2l-(M2f/(2^iiiiii));
    else
        M2l=M2l+(M2f/(2^iiiiii));
    end   
end

M2l=M2l;

vM2l=((atan((((M2l^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M2l^2)-1)^0.5)));
end

T01= T_inf*(1+(y-1)/2*M^2);
p01dp1l=(((y-1)/2)*(M^2)+1)^(y/(y-1));
p02dp2l=(((y-1)/2)*(M2l^2)+1)^(y/(y-1));

if -theta_le_l+aoa>0
Mn1l = M_n*sin(beta_sh_l);
p02dp01l = (((y+1)*Mn1l^2)/((y-1)*Mn1l^2+2))^(y/(y-1))*((y+1)/(2*y*Mn1l^2-(y-1)))^(1/(y-1));
else
p02dp01l=1;
end

p02l = p02dp01l*p01dp1l;
p2l(i)=p02dp01l/p02dp2l*p01dp1l;

Cpl1=(p2l(i)-1)/(.5*y*(M1^2));
Cnl1=Cpl1*ct;
Cal1=-Cpl1*s_l(2)*ct;

else

p3l(2)=p2l(2);
M3l(2)=M2l;       
theta_1_l(i)=atan(s_l(i-1));
theta_2_l(i)=atan(s_l(i));
theta_l(i)=theta_2_l(i)-theta_1_l(i);   
       
if theta_l(i)<0   
beta_sh_l_3=(beta_low+beta_high)/2;
for xxxxxxx=2:xxxxxxx_max

% --- Oblique-shock theta-beta-M relation ---
wing_angle_3_l(i)=atan(2*cot(beta_sh_l_3)*(M3l(i-1)^2*(sin(beta_sh_l_3))^2-1)/(M3l(i-1)^2*(y+cos(2*beta_sh_l_3))+2));

if wing_angle_3_l(i)<-theta_l(i)
    beta_sh_l_3=beta_sh_l_3+(beta_high-beta_low)/2^xxxxxxx;
else
    beta_sh_l_3=beta_sh_l_3-(beta_high-beta_low)/2^xxxxxxx;
end

end

beta_sh_l_3_i(i)=beta_sh_l_3;

Mn2l3(i) = M3l(i-1)*sin(beta_sh_l_3);
Mn3l = sqrt((1+((y-1)/2)*Mn2l3(i)^2) / (y*Mn2l3(i)^2-(y-1)/2));
M3l(i) = Mn2l3(i)/cos(pi/2-beta_sh_l_3-theta_l(i));
vM3l(i)=((atan((((M3l(i)^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-(atan(((M3l(i)^2)-1)^0.5)));
% Mn3u = sqrt((1+((y-1)/2)*Mn2l3^2) / (y*Mn2l3^2-(y-1)/2));
p02dp2l3(i)=(((y-1)/2)*(M3l(i-1)^2)+1)^(y/(y-1));
p03dp3l(i)=(((y-1)/2)*(M3l(i)^2)+1)^(y/(y-1));
p02dp01l3(i) = (((y+1)*Mn2l3(i)^2)/((y-1)*Mn2l3(i)^2+2))^(y/(y-1))*((y+1)/(2*y*Mn2l3(i)^2-(y-1)))^(1/(y-1));
p03l(i) = p02dp01l3(i)*p02dp2l3(i);
% p03l(i) = p02dp01l3(i)/p03dp3l(i);
% p3l(i)=p03l(i)/p03dp3l(i);
p3l(i)=p03l(i)/p03dp3l(i);
p3l(i) = p02dp01l3(i)/p03dp3l(i)*p02dp2l3(i)*p3l(i-1);
p03l(i)=p03dp3l(i)*p3l(i);
   
else    
    
vM3l(2)=vM2l;
vM3l(i)=theta_l(i)+vM3l(i-1);
M3i=10*M1/2; % M3 initial condition set to a very high value
M3f=10*M1; % fixed M3 initial condition reduced in the for loop


for iiiiii = 2:N
    
    vM3il(i)=((atan((((M3i^2)-1)*((y-1)/(y+1)))^0.5)*(((y+1)/(y-1))^0.5))-...
        (atan(((M3i^2)-1)^0.5)));
    
    if vM3il(i)>vM3l(i)
        M3i=M3i-(M3f/(2^iiiiii));
    else
        M3i=M3i+(M3f/(2^iiiiii));
    end
    
end

M3l(i)=M3i;
p03dp3l(i)=(((y-1)/2)*(M3l(i)^2)+1)^(y/(y-1));
p3l(i)=p02l/p03dp3l(i);

end

Cpl2(i)=(p3l(i)-1)/(.5*y*(M1^2));
Cnl2(i)=Cpl2(i)*ct;
Cal2(i)=-Cpl2(i)*s_l(i)*ct;

Cn_B(1)=Cnu1+Cnl1;
Ca_B(1)=Cau1+Cal1;

Cn_B(i)=Cnu2(i)+Cnl2(i);
Ca_B(i)=Cau2(i)+Cal2(i);

Cl_B(i)=(Cn_B(i)*cos(aoa))-(Ca_B(i)*sin(aoa));
Cd_B(i)=(Cn_B(i)*sin(aoa))+(Ca_B(i)*cos(aoa));

end

% Integrate normal/axial force coefficients over the airfoil and rotate to lift/drag
if i>=i_max-2

Cn=Cnu1+sum(Cnu2)+Cnl1+sum(Cnl2);   % normal-force coefficient
Ca=Cau1+sum(Cau2)+Cal1+sum(Cal2);   % axial-force coefficient

Cl(i)=(Cn*cos(aoa))-(Ca*sin(aoa));
Cl(i)=C_L;
Cd=(Cn*sin(aoa))+(Ca*cos(aoa));


if M<1
beta=(1-M^2)^.5;
else
beta=(M^2-1)^.5;
end

if M<1
C_L_alpha = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep_xc)^2/beta^2))); %Raymer
else
C_l_alpha=4/(M_n^2-1)^.5*cos(sweep); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.17 page 331
C_L_alpha=C_l_alpha*(1-1/(2*AR*(M_n^2-1)^.5)); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.18 page 331
end


K0=1/C_L_alpha;

if M>=1
K100=1/(pi*AR);
else
K100=(pi*AR*e);
end

lowerIdxcl = find(C_L >= CL_ww, 1, 'last');
if lowerIdxcl == length(CL_ww)
upperIdxcl = lowerIdxcl;
else
upperIdxcl = lowerIdxcl+1;
end

CL_ww_1=CL_ww(lowerIdxcl);
CL_ww_2=CL_ww(upperIdxcl);
S_ww_1=S_ww(lowerIdxcl);
S_ww_2=S_ww(upperIdxcl);
C_ww=-(S_ww_2-S_ww_1)/(CL_ww_2-CL_ww_1)*CL_ww_1+S_ww_1;
S_ww_t=(S_ww_2-S_ww_1)/(CL_ww_2-CL_ww_1)*C_L+C_ww;

if lowerIdxcl==length(CL_ww)
    S_ww_t=S_ww_2;
elseif upperIdxcl==1
    S_ww_t=S_ww_1;
end

K=S_ww_t*K100+(1-S_ww_t)*K0; %Raymer

c_d_i=K*C_L^2;

% Cd_p_fus=.008;

% M_n=M*cos(sweep);
% 
% if M_n >= 1
%     C_d_wave_wing_i = (C_L^2/4)*sqrt(M_n^2 - 1); %Derived and rearranged based on equations from wikipedia and Ackeret theory
%     C_d_wave_wing_i_max=(C_L_max_W^2/4)*sqrt(M_n^2 - 1);
% else
%     C_d_wave_wing_i = 0;    % do not use Ackeret
%     C_d_wave_wing_i_max=0;
% end

Cd_t=C_d_0+c_d_i+Cd;

r_i(i)=C_L/Cd_t;

end

end

end

%% ======================== 6. TRACK BEST L/D GEOMETRY ========================
r_j(n)=r_i(i);                      % L/D for this crest combination
r_j_max=max(r_j(2:end));

if r_j(n)==r_j_max
    r_le_j=r_le;
    x_crest_u_j=x_crest_u;
    y_crest_u_j=y_crest_u;
    x_crest_l_j=x_crest_l;
    y_crest_l_j=y_crest_l;
    aoa_j=aoa;
    a1_u_j=a1_u;
    a1_l_j=a1_l;
    a2_u_j=a2_u;
    a2_l_j=a2_l;
    a3_u_j=a3_u;
    a3_l_j=a3_l;
    a4_u_j=a4_u;
    a4_l_j=a4_l;
    a5_u_j=a5_u;
    a5_l_j=a5_l;
    if M<1
    s_A_j=s_A;
    s_B_j=s_B;
    Cl_j=dL_u;
    else
    Cl_j=Cl_B;
    end
end
    

end

r_k(p)=r_j_max;
r_k_max=max(r_k(2:end));

if r_k(p)==r_k_max
    r_le_k=r_le_j;
    x_crest_u_k=x_crest_u_j;
    y_crest_u_k=y_crest_u_j;
    x_crest_l_k=x_crest_l_j;
    y_crest_l_k=y_crest_l_j;
    aoa_k=aoa_j;
    a1_u_k=a1_u_j;
    a1_l_k=a1_l_j;
    a2_u_k=a2_u_j;
    a2_l_k=a2_l_j;
    a3_u_k=a3_u_j;
    a3_l_k=a3_l_j;
    a4_u_k=a4_u_j;
    a4_l_k=a4_l_j;
    a5_u_k=a5_u_j;
    a5_l_k=a5_l_j;
    Cl_k=Cl_j;
    if M<1
    s_A_k=s_A_j;
    s_B_k=s_B_j;
    end
end

end

r_n(nn)=r_k_max;
r_n_max=max(r_n(2:end));

if r_n(nn)==r_n_max
    r_le_n=r_le_k;
    x_crest_u_nn=x_crest_u_k;
    y_crest_u_nn=y_crest_u_k;
    x_crest_l_nn=x_crest_l_k;
    y_crest_l_nn=y_crest_l_k;
    aoa_n=aoa_k;
    a1_u_nn=a1_u_k;
    a1_l_nn=a1_l_k;
    a2_u_nn=a2_u_k;
    a2_l_nn=a2_l_k;
    a3_u_nn=a3_u_k;
    a3_l_nn=a3_l_k;
    a4_u_nn=a4_u_k;
    a4_l_nn=a4_l_k;
    a5_u_nn=a5_u_k;
    a5_l_nn=a5_l_k;
    Cl_nn=Cl_k;
    if M<1
    s_A_nn=s_A_k;
    s_B_nn=s_B_k;
    end
end

end

r_p(pp)=r_n_max;
r_p_max=max(r_p(2:end));

if r_p(pp)==r_p_max
    r_le_p=r_le_n;
    x_crest_u_p=x_crest_u_nn;
    y_crest_u_p=y_crest_u_nn;
    x_crest_l_p=x_crest_l_nn;
    y_crest_l_p=y_crest_l_nn;
    aoa_p=aoa_n;
    a1_u_pp=a1_u_nn;
    a1_l_pp=a1_l_nn;
    a2_u_pp=a2_u_nn;
    a2_l_pp=a2_l_nn;
    a3_u_pp=a3_u_nn;
    a3_l_pp=a3_l_nn;
    a4_u_pp=a4_u_nn;
    a4_l_pp=a4_l_nn;
    a5_u_pp=a5_u_nn;
    a5_l_pp=a5_l_nn;
    Cl_pp=Cl_nn;
    if M<1
    s_A_pp=s_A_nn;
    s_B_pp=s_B_nn;
    end
end

end

%% ======================== 7. STRATFORD RESULT + FINAL GEOMETRY ========================
if M<1
    
diff_s=s_A_pp-s_B_pp;               % LHS - RHS of Stratford criterion
max_diff_s=max(real(diff_s(iii:end)));  % only check aft of upper crest

if max_diff_s>0
    fprintf('flow separation\n')    % s_A > s_B  =>  recovery too steep
else
    fprintf('no flow separation\n')
end

end

aoa_final=aoa_p;

r_final=r_p_max;

fprintf('aoa=%f\nx_crest_u=%f\ny_crest_u=%f\n',aoa_final,x_crest_u_p,y_crest_u_p)

fprintf('x_crest_l=%f\ny_crest_l=%f\nL/D=%f\n',x_crest_l_p,y_crest_l_p,r_final)

if M<1
x=linspace(0,1,100);
u_final=a1_u_pp*x.^(1/2)+a2_u_pp*x.^(1)+a3_u_pp*x.^(2)+a4_u_pp*x.^(3)+a5_u_pp*x.^(4);
l_final=a1_l_pp*x.^(1/2)+a2_l_pp*x.^(1)+a3_l_pp*x.^(2)+a4_l_pp*x.^(3)+a5_l_pp*x.^(4);
else
x=linspace(0,1,100);
u_final=a1_u_pp*x.^(1)+a2_u_pp*x.^(2)+a3_u_pp*x.^(3)+a4_u_pp*x.^(4)+a5_u_pp*x.^(5);
l_final=a1_l_pp*x.^(1)+a2_l_pp*x.^(2)+a3_l_pp*x.^(3)+a4_l_pp*x.^(4)+a5_l_pp*x.^(5);
end

plot(x,u_final,'-r')
hold on
plot(x,l_final,'-r')
xlabel('x/c')
ylabel('y/c')
title('Best L/D Airfoil')

figure(2)
plot(x_i(3:end-1),Cl_pp(3:end-1)*i_max,'.r')
xlabel('% chord')
ylabel('C_L')
title('Airfoil Lift Distribution')




