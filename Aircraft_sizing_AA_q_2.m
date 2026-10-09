clc
clear
close all

%% ==============================================================
%  AIRCRAFT SIZING AND PERFORMANCE MODEL
%  ==============================================================
%
%  Author: Brian Wilkinson
%
%  --------------------------------------------------------------
%  DESCRIPTION
%  --------------------------------------------------------------
%  Conceptual aircraft sizing and performance analysis program
%  using Raymer-style weight estimation, aerodynamic drag models,
%  propulsion analysis, constraint analysis, and iterative sizing.
%
%  The program can analyze both custom aircraft and a selection of
%  historical/reference aircraft.
%
%  --------------------------------------------------------------
%  AIRCRAFT CASES
%  --------------------------------------------------------------
%    aircraft_case = 0   Custom aircraft
%    aircraft_case = 1   Concorde
%    aircraft_case = 2   Tu-144D
%    aircraft_case = 3   B737-200
%    aircraft_case = 4   B777-200ER
%    aircraft_case = 5   A320-200
%    aircraft_case = 6   CJ-3
%    aircraft_case = 7   2000EX
%    aircraft_case = 8  XB-70 Valkyrie
%    aircraft_case = 9  SR-71 Blackbird

%To run this code, set aircraft_case = 0 if you want this script to size an aircraft
%based on custom inputs. Set aircraft_case>0 to select an aircraft used for
%verification of the methods in this script. 

%The outputs for this script are for the values at book takeoff gross
%weight and book altitude by setting q_max equal to two where TOGW does 
%not iterate. Ignore all output figures and only consider the percent
%errors output. In the other script, q_max is set equal to 50 where the 
%TOGW iterates 50 times which is enough for the TOGW to converge.

%The error output at the bottom of the command window represents the
%percent errors of certain variables. S is wing area, b is wingspan, c_r is
%the root chord length, TOGW is takeoff gross weight, We is empty weight,
%Wf is fuel weight, alt is design altitude, R is range, L/D is lift to drag
%ratio, TSFC is thrust specific fuel consumption. 


aircraft_case=9; %0=custom inputs, 1=Concorde, 2=TU-144D,
%3=B737-200 4=B777-200ER 5=A320-200 6=CJ-3 7=2000EX 8=XB-70 9=SR-71

if aircraft_case==0
% ============================================================
% ==================== MISSION REQUIREMENTS =================
% ============================================================

M = 2;                         % Cruise Mach number [-]
Range_cruise = 5600000;        % Cruise range [m] 

TOGW_i(1) = 185000*9.81;       % Initial takeoff gross weight [N]
W_e(1)    = 78700*9.81;        % Initial empty operating weight [N]
W_payload = 13400*9.81;        % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload;  % Initial fuel weight [N]


Np        = 100;               % Passenger capacity [-]

type2     = 0;                 % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 




% ============================================================
% ==================== PROPULSION ===========================
% ============================================================

type        = 0;               % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 4;               % Number of engines used in propulsion calculation [-]
N_en        = 4;               % Total number of engines [-]

Afterburner = 1;               % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;               % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;               % Waverider configuration: 0 = disabled, 1 = enabled

Cd_engine   = 0.278;           % Engine/nacelle drag coefficient [-]

% ============================================================
% ================ IDEAL ENGINE DATA =====================
% ============================================================

A_inlet     = .951;            % Inlet capture area per engine [m^2]

TIT_d_low   = 1200;            % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;            % High design turbine inlet temperature [K]
T_0_6_high  = 1800;            % High turbine/combustor total temperature [K]
T_0_3_sl    = 550;             % Sea-level compressor total temperature which the sea level thrust depends on [K]

CPR_min     = 15.5;            % Minimum compressor pressure ratio [-]
CPR_max     = 15.5;            % Maximum compressor pressure ratio [-]

BR_min      = 0;               % Minimum bypass ratio [-]
BR_max      = 0;               % Maximum bypass ratio [-]
FPR         = 0;               % Fan pressure ratio [-]

n_ab_low    = 0;               % Low afterburner setting [-]
n_ab_high   = 0.6;             % High afterburner setting [-]

eta_inlet   = 0.92;            % Inlet efficiency at M=2 [-]
eta_c       = 0.85;            % Compressor efficiency [-]
eta_f       = 0.88;            % Fan efficiency [-]
eta_t       = 0.88;            % Turbine efficiency [-]
eta_b       = 0.98;            % Burner/combustor efficiency [-]
eta_ab      = 0.85;            % Afterburner efficiency [-]
eta_m       = 0.98;            % Mechanical efficiency [-]
eta_n       = 0.94;            % Nozzle efficiency [-]



% ============================================================
% ================ EXISTING ENGINE DATA =====================
% ============================================================

Th_wet         = 169200;       % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 140000;       % Cruise wet/reheat thrust per engine [N]
Th_dry         = 142300;       % Maximum dry thrust per engine [N]
Th_dry_cruise  = 140000;       % Cruise dry thrust per engine [N]

TIT_e_low      = 1200;         % Engine low turbine inlet temperature [K]
TIT_e          = 1400;         % Engine turbine inlet temperature [K]
CPR_e          = 15.5;         % Engine compressor pressure ratio [-]
BR_e           = 0;            % Engine bypass ratio [-]
FPR_e          = 0;            % Engine fan pressure ratio [-]

eta_t_e        = 0.88;         % Engine turbine efficiency [-]
eta_f_e        = 0.88;         % Engine fan efficiency [-]
eta_ab_e       = 0.85;         % Engine afterburner efficiency [-]
eta_c_e        = 0.85;         % Engine compressor efficiency [-]
eta_m_e        = 0.98;         % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]

A_inlet_e      = .951;   % Existing engine inlet area [m^2]
T_0_6_k_e      = 1704+273;     % Existing engine turbine temperature [K]

W_eng_e        = 3175*9.81;    % Engine weight per engine [N] (~7000 lbf class)


% ============================================================
% ==================== AERODYNAMICS ==========================
% ============================================================

Subsonic_airfoil = 5;          % Airfoil model/index [-]
delta        = 0;              % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;              % Sweep calculation method/index [-]

x_c          = 0.5;            % Airfoil maximum thickness/camber reference location [-]

cl_max       = 1.35;           % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;            % Aircraft maximum lift coefficient [-] (low-AR delta)

tdc          = 0.03;           % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 80;             % Sea-level stall speed [m/s]

c_f          = 0.0020;         % Skin-friction coefficient [-]
u_0          = 1.7894e-5;      % Reference dynamic viscosity [Pa*s]
Su           = 110.4;          % Sutherland constant [K]

nose_angle   = 5;              % Nose half angle

sweep        = 55*pi/180;      % Leading edge sweep [rad]

A_base       = 8;              % Base area [m^2]
lambda       = 0.15;           % Taper/geometry parameter [-]
droop        = 0;              % Drooped nose: 0 = no droop
AR           = 1.83;           % Wing aspect ratio [-]


% ============================================================
% ==================== TAIL INPUTS =================
% ============================================================

% -------------------- Horizontal tail ----------------------

S_H       = 0;                 % Horizontal-tail planform area [m^2]
b_H       = 0;                 % Horizontal-tail span [m]
sweep_H   = 0;                 % Horizontal-tail sweep [rad]
AR_H      = 0;                 % Horizontal-tail aspect ratio [-]
e_H       = 0;                 % Horizontal-tail Oswald efficiency [-]
CL_H      = 0;                 % Horizontal-tail lift coefficient [-]
alpha_H   = 0;                 % Horizontal-tail angle of attack [rad]

c_H       = 5.0;               % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;              % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.08;              % Horizontal-tail thickness-to-chord ratio [-]


% ---------------------- Vertical tail ----------------------

S_V       = 33.91;             % Vertical-tail/fin planform area [m^2]
b_V       = 11.32;             % Vertical-fin height/span [m]
sweep_V   = 51*pi/180;         % Vertical-fin leading-edge sweep [rad]

AR_V      = b_V^2/S_V;         % Vertical-fin aspect ratio [-]

e_V       = 0.80;              % Vertical-fin Oswald efficiency [-]
CL_V      = 0;                 % Vertical-fin lift coefficient [-]
beta_V    = 0;                 % Sideslip angle [rad]

t_c_V     = 0.046;             % Vertical-fin thickness ratio [-]

c_V       = 5.5;               % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;              % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.08;              % Vertical-tail maximum thickness-to-chord ratio [-]


% ============================================================
% ================= ATMOSPHERE / EARTH ======================
% ============================================================

Cp         = 1009;             % Specific heat of air [J/(kg*K)]
rho_sl     = 1.226;            % Sea-level air density [kg/m^3]
T_sl       = 288.15;           % Sea-level standard temperature [K]
R          = 287.05;           % Specific gas constant for air [J/(kg*K)]
y          = 1.4;              % Ratio of specific heats [-]
g          = 9.81;             % Gravitational acceleration [m/s^2]
p_sl       = 101300;           % Sea-level standard pressure [Pa]
radius_e   = 6.371e6;           % Earth radius [m]

mu_0       = 1.716e-5;         % Reference dynamic viscosity [Pa*s]
T_0        = 273.15;           % Reference temperature [K]
Su         = 110.4;            % Sutherland constant [K]


% ============================================================
% ============= STRUCTURAL / RAYMER GEOMETRY ================
% ============================================================

N_z = 3.75;                    % Ultimate load factor / structural factor [-]
N_l = 3.0;                     % Landing load factor / structural factor [-]
ratio_fin = 19;                % Fin geometry ratio [-]

D_f    = 3.10;                 % Fuselage maximum diameter [m]
L      = 61.66;                % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.88;                 % Fuselage width [m]

p_cabin = p_sl;                % Cabin pressure [Pa]

t_aluminum = 0.002;            % Aluminum skin thickness [m]
rho_aluminum = 2800;           % Aluminum density [kg/m^3]


% ----------------------- Wing geometry ----------------------

S      = 358.25;               % Wing reference area [m^2]
S_csw  = 28.0;                 % Elevon/control-surface area [m^2]
S_ht   = 0.1;                  % Horizontal-tail/canard area [m^2]
S_vt   = 32.0;                 % Vertical-tail area [m^2]

B_h    = 6.0;                  % Horizontal-tail span/reference dimension [m]
F_w    = 2.88;                 % Fuselage width [m]
H_t_Hv = 0;                    % Horizontal-tail to vertical-tail parameter [-]
L_t    = 22.0;                 % Tail moment arm [m]

Sr_Svt = 0.35;                 % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.4;                  % Vertical-tail geometry parameter [-]
sweep_vt = 50*pi/180;          % Vertical-tail sweep [rad]

S_cs   = S_csw + 0.35*S_vt;    % Total control-surface area [m^2]


% --------------------- Landing gear -------------------------

L_m  = 3.50;                   % Main landing gear length [m]
L_n  = 2.80;                   % Nose landing gear length [m]

N_mw = 8;                      % Main landing gear wheels [-]
N_nw = 2;                      % Nose landing gear wheels [-]
N_mss = 2;                     % Main landing gear struts/assemblies [-]

L_s  = 3.50*3.28;              % Landing gear strut length [ft]


% ---------------- Raymer correction factors -----------------

K_dw = 0.768;                  % Design wing weight correction factor [-]
K_vs = 1.0;                    % Vertical stabilizer correction factor [-]
K_dwf = 0.774;                 % Design wing/fuselage correction factor [-]

K_rht = 1.0;                   % Horizontal-tail correction factor [-]
K_uht = 1.0;                   % Horizontal-tail correction factor [-]
K_door = 1.06;                 % Door correction factor [-]
K_Lg = 1.12;                   % Landing gear correction factor [-]

K_cb = 1.0;                    % Cabin correction factor [-]
K_tpg = 1.0;                   % Tail/pressure-group correction factor [-]
K_mp = 1.0;                    % Main propulsion correction factor [-]
K_np = 1.0;                    % Nose propulsion correction factor [-]

K_vg = 1.0;                    % Variable-geometry correction factor [-]
K_d = 1.75;                    % Drag/structural correction factor [-]
K_ws = 0.0;                    % Wing sweep correction factor [-]
Ls_Ld = 0.40;                  % Landing-gear length ratio [-]

K_sys = 1.65;                  % Systems weight constant [-]

K_gr = 0.007;                  % Landing gear weight coefficient [-]
a_g  = 0.95;                   % Landing gear weight exponent [-]
b_g  = 0.60;                   % Landing gear strut length exponent [-]


% -------------------- Fuselage / cabin ---------------------

D_e     = 1.21;                % Equivalent fuselage/cabin diameter [m]
L_d     = 8.0;                 % Door/fuselage length parameter [m]
L_tp    = 2.0;                 % Tail-pipe length [m]
L_sh    = 1.5;                 % Shaft/structural length parameter [m]
L_ec    = 25.0;                % Equipment/cabin length [m]


% ---------------------- Fuel system -------------------------

V_t   = 119.5;                 % Total fuel tank volume [m^3]
Vi_Vt = 0.90;                  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;                  % Tank volume fraction [-]

N_t   = 17;                    % Number of fuel tanks [-]
SFC   = 1.19;                  % Cruise specific fuel consumption [lb/(lbf*hr)]


% ------------------- Systems / equipment -------------------

N_c = 3;                       % Number of crew [-]
N_ci = 2.0;                    % Crew/instrumentation parameter [-]
N_s = 2;                       % Number of seats/seat-related parameter [-]
N_u = 8;                       % Number of utility/system units [-]

Nf = 5;                        % Number of furnishings/functional units [-]
Nm = 2;                        % Number of miscellaneous units [-]

W_uav = 800*9.81;              % UAV/equipment weight [N]


% -------------------- Electrical systems -------------------

K_vsh = 1.0;                   % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;                   % Mechanical/control systems factor [-]
R_kva = 80;                    % Electrical system rating [kVA]


% ------------------------ Doors ----------------------------

N_doors = 4;                   % Number of passenger/service doors [-]
N_cargo_doors = 1;             % Number of cargo doors [-]

S_door = 1.5;                  % Passenger/service door area [m^2]
S_cargo_door = 2.0;            % Cargo door area [m^2]

S_floor_main = 80;             % Main-deck floor area [m^2]
S_floor_upper = 0;             % Upper-deck floor area [m^2]

w_main = 2.1/0.0208854;        % Main-deck floor areal weight [lb/ft^2]
w_upper = 0;                   % Upper-deck floor areal weight [lb/ft^2]


% ============================================================
% ================= NACELLE / INTAKE =========================
% ============================================================

K_ng     = 1.0;                % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;                % Pylon factor [-]
K_tr     = 1.18;               % Thrust reverser factor [-]

N_Lt     = 8.0;                % Intake + engine characteristic length [m]
N_w      = 1.40;               % Equivalent nacelle/inlet width [m]

S_n      = pi*N_w*N_Lt;        % Equivalent nacelle wetted/reference area [m^2]

N_e      = 4;                  % Number of engines [-]
W_en     = W_eng_e;            % Engine weight per engine [N]

N_Lt_ft  = N_Lt/0.3048;        % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;         % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;       % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;    % Engine weight per engine [lb]


% ============================================================
% ================= PERFORMANCE / TRAJECTORY ================
% ============================================================

rate_of_climb    = 5000/3.28/60;     % Rate of climb [m/s]
rate_of_descent  = 3000/3.28/60;     % Rate of descent [m/s]

Takeoff_field_length = 3400;         % Takeoff field length [m]
Landing_field_length = 2200;         % Landing field length [m]

g_turn = 1.2;                        % Turning/acceleration load factor [-]
mu_2 = 0.03;                         % Ground friction coefficient [-]

alt_0 = 1000;                        % Initial altitude [m]
acc = 2.0;                           % Acceleration [m/s^2]

V_lof = 1.1*V_stall_sl;              % Lift-off speed [m/s]


% ============================================================
% ================= AEROTHERMAL PARAMETERS ==================
% ============================================================

TPS = 0;                             % Thermal protection system: 0=no, 1=yes

R_wle = 0.344/12;                    % Wing leading-edge radius [ft]

g_s = g*3.2797;                      % Gravity conversion parameter [ft/s^2]

J = 778;                             % Mechanical equivalent of heat [ft*lbf/BTU]

emmissivity = 0.8;                   % Radiative emissivity [-]

Cp_wall = 1000;                      % Wall/TPS specific heat [J/(kg*K)]
rho_TPS = 40;                        % TPS areal density [kg/m^2]

rho_titanium = 4510;                 % Titanium density [kg/m^3]

T_ins_max = 581;                     % Maximum insulation temperature [K]
T_sys_max = 273+70;                  % Maximum systems temperature [K]

k_titanium = 21;                     % Titanium thermal conductivity [W/(m*K)]
k_insulation = .3;                   % Insulation thermal conductivity [W/(m*K)]

T_initial = 230;                     % Initial internal air temperature [K]

t_ins = .051;                        % Insulation thickness [m]
t_tit = .001;                        % Titanium thickness [m]

Cp_tit = 523.5;                      % Titanium specific heat [J/(kg*K)]
Cp_ins = 500;                        % Insulation specific heat [J/(kg*K)]

rho_ins = 112;270;                   % Insulation density [kg/m^3]

S_cooling = 39;20;39;                % Active cooling surface area [m^2]
rho_cooling = 300;                   % Active cooling surface areal density [kg/m^2]

x = 4;                               % Heating-analysis longitudinal location [m]

Pr = 0.72;                           % Prandtl number [-]


% ============================================================
% ==================== FUEL PROPERTIES =======================
% ============================================================

E_sp = 44E6;                         % Fuel specific energy [J/kg]

ratio_fo = .294;                     % Stoichiometric fuel-to-oxygen mass ratio [-]

Cp_f = 2010;                         % Fuel specific heat [J/(kg*K)]

rho_f       = 800;                   % Fuel density [kg/m^3]


% ============================================================
% ================= ITERATION / SEARCH PARAMETERS ===========
% ============================================================

min_alt = 16000;                     % Minimum search altitude [m]
max_alt = 16001;                     % Maximum search altitude [m]
WdS_max = 6000;                      % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 200;                    % Number of altitude points shown in design point plot [-]
i_max = 20;                          % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                         % number of iterations for finding the existing engine turbine inlet temperature during cruise [-]
j_max = 20;                          % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                          % number of iterations for finding the ideal engine turbine inlet temperature during cruise [-]

q_max = 2;                           % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration

dd_max = 15;                         % number of iterations from altitude=0 to altitude=design for the descent phase 
cc_max = 15;                         % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;                        % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;                         % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft

iii_max = 30;                        % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft


ff_max = 30;                         % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;                         % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;                    % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition

f_6_k_e_i_low = 0;                  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = .5;                % Upper bound for existing engine afterburner fuel fraction flow rate


% ============================================================
% =============== REFERENCE AIRCRAFT INPUTS =================
% ============================================================

S0      = 358.25;                    % Reference wing area [m^2]
b0      = 25.56;                     % Reference wingspan [m]
c_r0    = 27.66;                     % Reference root chord [m]

Lambda0 = 0;                         % Reference wing sweep [rad]
                                       % 0 if model does not require/reference sweep

Alt0    = 16000;                     % Reference cruise altitude [m]
R0      = 6200e3;                    % Reference range [m]

L_D     = 7.14;                      % Reference cruise lift-to-drag ratio [-]
                                       % Cruise Mach approximately 2.0

TSFC0   = 33.8e-6;                   % Reference cruise TSFC [kg/(N*s)]
                                       % Cruise/dry


elseif aircraft_case==1

% ==================== MISSION REQUIREMENTS ====================
M = 2;  % Cruise Mach number [-]
Range_cruise = 5600000;  % Cruise range [m] 
TOGW_i(1) = 185000*9.81;     % Initial takeoff gross weight [N] 
W_e(1)    = 78700*9.81;    % Initial empty operating weight [N]
W_payload = 13400*9.81;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 100;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 4;  % Number of engines used in propulsion calculation [-]
N_en        = 4;  % Total number of engines [-]
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = .951;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1800;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 15.5;  % Minimum compressor pressure ratio [-]
CPR_max     = 15.5;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.6;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency at M=2 [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]
Cd_engine   = 0.278;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA (Olympus 593) ====================
Th_wet         = 169200;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 140000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 142300;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 140000;  % Cruise dry thrust per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 15.5;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = .951;1.125;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1704+273;  % Existing engine turbine temperature [K]
W_eng_e        = 3175*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 5;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.5;  % Airfoil maximum thickness/camber reference location [-]
cl_max       = 1.35;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;  % Aircraft maximum lift coefficient [-] (low-AR delta)
tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 80;  % Sea-level stall speed [m/s]
c_f          = 0.0020;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 5;  % Nose half angle
sweep        = 55*pi/180;  % Leading edge sweep [rad] (using published value on reputable website for the ogive wing)
A_base       = 8;  % Base area [m^2]
lambda       = 0.15;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 1.83;  % Wing aspect ratio [-]

%% ================= CONCORDE TAIL INPUTS =================

% Horizontal tail
S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail sweep [rad]
AR_H      = 0;  % Horizontal-tail aspect ratio [-]
e_H       = 0;  % Horizontal-tail Oswald efficiency [-]
CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]
c_H  = 5.0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H = 0.08;  % Horizontal-tail thickness-to-chord ratio [-]


% Vertical tail / fin
S_V       = 33.91;  % Vertical-tail/fin planform area [m^2]
b_V       = 11.32;  % Vertical-fin height/span [m]
sweep_V   = 51*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

t_c_V     = 0.046;  % Vertical-tail maximum thickness-to-chord ratio [-]

c_V  = 5.5;  % Vertical-tail mean aerodynamic chord [m]
x_t_V = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V = 0.08;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009; rho_sl = 1.226; T_sl = 288.15; R = 287.05;  % Specific heat of air [J/(kg*K)]
y = 1.4; g = 9.81; p_sl = 101300; radius_e = 6.371e6;  % Ratio of specific heats [-]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY (Concorde) ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 19;  % Fin geometry ratio [-]
D_f    = 3.10;  % Fuselage maximum diameter [m]
L      = 61.66;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.88;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 358.25;  % Wing reference area [m^2]
S_csw  = 28.0;  % Elevon/control-surface area [m^2]
S_ht   = 0.1;  % Horizontal-tail/canard area [m^2]
S_vt   = 32.0;  % Vertical-tail area [m^2]
B_h    = 6.0;  % Horizontal-tail span/reference dimension [m]
F_w    = 2.88;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 22.0;  % Tail moment arm [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.4;  % Vertical-tail geometry parameter [-]
sweep_vt = 50*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.35*S_vt;  % Total control-surface area [m^2]

L_m  = 3.50;  % Main landing gear length [m]
L_n  = 2.80;  % Nose landing gear length [m]
N_mw = 8;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 3.50*3.28;  % Landing gear strut length [ft]

K_dw = 0.768; K_vs = 1.0; K_dwf = 0.774;  % Design wing weight correction factor [-]
K_rht = 1.0; K_uht = 1.0; K_door = 1.06; K_Lg = 1.12;  % Horizontal-tail correction factor [-]
K_cb = 1.0; K_tpg = 1.0; K_mp = 1.0; K_np = 1.0;  % Cabin correction factor [-]
K_vg = 1.0; K_d = 1.75; K_ws = 0.0; Ls_Ld = 0.40;  % Variable-geometry correction factor [-]
K_sys=1.65;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


D_e     = 1.21;  % Equivalent fuselage/cabin diameter [m]
L_d     = 8.0;  % Door/fuselage length parameter [m]
L_tp    = 2.0;  % Tail-pipe length [m]
L_sh    = 1.5;  % Shaft/structural length parameter [m]
L_ec    = 25.0;  % Equipment/cabin length [m]

V_t   = 119.5;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]
N_t   = 17;  % Number of fuel tanks [-]
SFC   = 1.19;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 3; N_ci = 2.0; N_s = 2; N_u = 8;  % Number of crew [-]
Nf = 5; Nm = 2; W_uav = 800*9.81;  % Number of furnishings/functional units [-]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]
R_kva = 80;  % Electrical system rating [kVA]

N_doors = 4; N_cargo_doors = 1;  % Number of passenger/service doors [-]
S_door = 1.5; S_cargo_door = 2.0;  % Passenger/service door area [m^2]
S_floor_main = 80; S_floor_upper = 0;  % Main-deck floor area [m^2]
w_main = 2.1/0.0208854; w_upper = 0;  % Main-deck floor areal weight [lb/ft^2]

%% ---- nacelle / intake (4 integrated 2-D ramps) ----
K_ng     = 1.0;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.18;  % Thrust reverser factor [-]
N_Lt     = 8.0;  % Intake + engine characteristic length [m]
N_w      = 1.40;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]
N_e      = 4;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]
N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 5000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 3000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3400;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS=0;  % Thermal protection system: 0=no, 1=yes
R_wle = 0.344/12;  % Wing leading-edge radius [ft]
g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity=0.8;  % Radiative emissivity [-]
Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
rho_TPS=40;  % TPS areal density [kg/m^2]
rho_titanium=4510;  % Titanium density [kg/m^3]
T_ins_max=581;  % Maximum insulation temperature [K]
T_sys_max=273+70;  % Maximum systems temperature [K]
k_titanium=21;  % Titanium thermal conductivity [W/(m*K)]
k_insulation=.3;  % Insulation thermal conductivity [W/(m*K)]
T_initial=230;  % Initial internal air temperature [K]
t_ins=.051;  % Insulation thickness [m]
t_tit=.001;.0015;  % Titanium thickness [m]
Cp_tit=523.5;  % Titanium specific heat [J/(kg*K)]
Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
rho_ins=112;270;  % Insulation density [kg/m^3]
S_cooling=39;20;39;  % Active cooling surface area [m^2]
rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
x = 4;  % Heating-analysis longitudinal location [m]
Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt=16000;  % Minimum search altitude [m]
max_alt=16001;  % Maximum search altitude [m]
WdS_max=6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points=200;  % Number of altitude points shown in design point plot [-]
i_max=20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;  % number of iterations for finding the existing engine turbine inlet temperature during cruise [-]
j_max=20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;  % number of iterations for finding the ideal engine turbine inlet temperature during cruise [-]
q_max=2;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max=15;  % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max=15;  % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max=30;  % number of points used for the carpet plot that has smoothed out curves
xx_max=20;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max=30;  % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max=30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max=30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max=30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low=0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high=.5;  % Upper bound for existing engine afterburner fuel fraction flow rate


% --- Concorde ---

S0      = 358.25;  % Reference wing area [m^2]
b0      = 25.56;  % Reference wingspan [m]
c_r0    = 27.66;  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]

Alt0    = 16000;  % Reference cruise altitude [m]
R0      = 6200e3;  % Reference range [m]
L_D     = 7.14;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 33.8e-6;  % Reference cruise TSFC [kg/(N*s)]


elseif aircraft_case==2


% ==================== MISSION REQUIREMENTS (Tu-144D) ====================
M = 2.15;  % Cruise Mach number [-]
Range_cruise = 5500000;  % Cruise range [m] 
TOGW_i(1) = 207000*9.81;  % Initial takeoff gross weight [N] 
W_e(1)    = 99200*9.81;  % Initial empty operating weight [N] 
W_payload = 2000*9.81;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 140;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (4 x RD-36-51) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 4;  % Number of engines used in propulsion calculation [-]
N_en        = 4;  % Total number of engines [-]
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 1.20;  % Inlet capture area per engine [m^2]
TIT_d_low   = 800;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1700;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 14.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 14.0;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency at M=2 [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.87;  % Turbine efficiency [-]
eta_b       = 0.97;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.93;  % Nozzle efficiency [-]
Cd_engine   = 0.25;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 196000;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 196000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 196000;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 180000;  % Cruise dry thrust per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 14.0;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.87;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 1.2;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1600;  % Existing engine turbine temperature [K]
W_eng_e        = 4000*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 5;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.5;  % Airfoil maximum thickness/camber reference location [-]
cl_max       = 0.90;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;  % Aircraft maximum lift coefficient [-] (low-AR delta)
tdc          = 0.035;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 71;  % Sea-level stall speed [m/s]
c_f          = 0.0021;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 6;  % Nose half angle
sweep        = 57*pi/180;  % Leading edge sweep [rad]
A_base       = 10;  % Base area [m^2]
lambda       = 0.12;  % Taper/geometry parameter [-]
droop        = 1;  % Drooped nose: 0 = no droop
AR           = 1.64;  % Wing aspect ratio [-]

% ============================================================
% TU-144D TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 16.5;  % Horizontal-tail span [m]
sweep_H   = 30*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 4.2;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.08;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 48.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 10.5;  % Vertical-fin height/span [m]
sweep_V   = 50*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 4.8;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.08;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009; rho_sl = 1.226; T_sl = 288.15; R = 287.05;  % Specific heat of air [J/(kg*K)]
y = 1.4; g = 9.81; p_sl = 101300; radius_e = 6.371e6;  % Ratio of specific heats [-]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 19;  % Fin geometry ratio [-]
D_f    = 3.50;  % Fuselage maximum diameter [m]
L      = 65.70;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 3.30;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 507.0;  % Wing reference area [m^2]
S_csw  = 35.0;  % Elevon/control-surface area [m^2]
S_ht   = 20.0;  % Horizontal-tail/canard area [m^2]
S_vt   = 36.0;  % Vertical-tail area [m^2]
B_h    = 7.5;  % Horizontal-tail span/reference dimension [m]
F_w    = 3.30;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 24.0;  % Tail moment arm [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.3;  % Vertical-tail geometry parameter [-]
sweep_vt = 50*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.35*S_vt;  % Total control-surface area [m^2]

L_m  = 3.60;  % Main landing gear length [m]
L_n  = 2.90;  % Nose landing gear length [m]
N_mw = 8;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 3.60*3.28;  % Landing gear strut length [ft]

K_dw = 0.768; K_vs = 1.0; K_dwf = 0.774;  % Design wing weight correction factor [-]
K_rht = 1.0; K_uht = 1.0; K_door = 1.06; K_Lg = 1.12;  % Horizontal-tail correction factor [-]
K_cb = 1.0; K_tpg = 1.0; K_mp = 1.0; K_np = 1.0;  % Cabin correction factor [-]
K_vg = 1.0; K_d = 1.75; K_ws = 0.0; Ls_Ld = 0.40;  % Variable-geometry correction factor [-]
K_sys=1.65;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]

        
D_e     = 1.30;  % Equivalent fuselage/cabin diameter [m]
L_d     = 9.0;  % Door/fuselage length parameter [m]
L_tp    = 2.2;  % Tail-pipe length [m]
L_sh    = 1.6;  % Shaft/structural length parameter [m]
L_ec    = 28.0;  % Equipment/cabin length [m]

V_t   = 116.0;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]
N_t   = 18;  % Number of fuel tanks [-]
SFC   = 1.26;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 3; N_ci = 2.0; N_s = 2; N_u = 8;  % Number of crew [-]
Nf = 5; Nm = 2; W_uav = 900*9.81;  % Number of furnishings/functional units [-]

N_doors = 4; N_cargo_doors = 1;  % Number of passenger/service doors [-]
S_door = 1.6; S_cargo_door = 2.2;  % Passenger/service door area [m^2]
S_floor_main = 95; S_floor_upper = 0;  % Main-deck floor area [m^2]
w_main = 2.1/0.0208854; w_upper = 0;  % Main-deck floor areal weight [lb/ft^2]

%% ---- nacelle / intake (4 integrated 2-D ramps) ----
K_ng     = 1.00;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.00;  % Pylon factor [-]
K_tr     = 1.18;  % Thrust reverser factor [-]

N_Lt     = 8.0;  % Intake + engine characteristic length [m]
N_w      = 1.35;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]

N_e      = 4;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]

N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 9840/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2500/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3600;  % Takeoff field length [m]
Landing_field_length = 2500;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS=0;  % Thermal protection system: 0=no, 1=yes
R_wle = 0.344/12;  % Wing leading-edge radius [ft]
g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity=0.8;  % Radiative emissivity [-]
Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
rho_TPS=40;  % TPS areal density [kg/m^2]
rho_titanium=4510;  % Titanium density [kg/m^3]
T_ins_max=581;  % Maximum insulation temperature [K]
T_sys_max=273+70;  % Maximum systems temperature [K]
k_titanium=21;  % Titanium thermal conductivity [W/(m*K)]
k_insulation=.3;  % Insulation thermal conductivity [W/(m*K)]
T_initial=230;  % Initial internal air temperature [K]
t_ins=.051;  % Insulation thickness [m]
t_tit=.001;.0015;  % Titanium thickness [m]
Cp_tit=523.5;  % Titanium specific heat [J/(kg*K)]
Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
rho_ins=112;270;  % Insulation density [kg/m^3]
S_cooling=39;20;39;  % Active cooling surface area [m^2]
rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
x = 4;  % Heating-analysis longitudinal location [m]
Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 18900;  % Minimum search altitude [m]
max_alt = 18901;  % Maximum search altitude [m]
WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20; ii_max = 25; j_max = 20; k_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
q_max = 2; dd_max = 15; cc_max = 15; zzz_max = 30;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
xx_max = 20; iii_max = 30;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max = 30; kk_max = 30; kk_Dmax_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
f_6_k_e_i_low = 0; f_6_k_e_i_high = 0.5;  % Lower bound for existing engine afterburner fuel fraction flow rate

S0      = 507.0;  % Reference wing area [m^2]
b0      = 28.80;  % Reference wingspan [m]
c_r0    = 31.90;  % Reference root chord [m]
Lambda0 = 57*pi/180;  % Reference wing sweep [rad]

Alt0    = 18900;  % Reference cruise altitude [m]
R0      = 6200000;  % Reference range [m]
L_D     = 8;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 4.0e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 57*pi/180;
% % 



elseif aircraft_case==3

% ==================== MISSION REQUIREMENTS (737-200) ====================
M = 0.74;  % Cruise Mach number [-]
Range_cruise = 4600000;  % Cruise range [m] 
TOGW_i(1) = 58740*9.81;  % Initial takeoff gross weight [N] 
W_payload = 10000*9.81;  % payload weight [N] 
W_e(1)    = 27200*9.81;  % Initial empty operating weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 130;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (2 x JT8D-15) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.85;  % Inlet capture area per engine [m^2]
TIT_d_low   = 800;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1350;  % High design turbine inlet temperature [K]
T_0_6_high  = 1350;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 12.5;  % Minimum compressor pressure ratio [-]
CPR_max     = 12.5;  % Maximum compressor pressure ratio [-]
BR_min      = 1.0;  % Minimum bypass ratio [-]
BR_max      = 1.0;  % Maximum bypass ratio [-]
FPR         = 1.65;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.97;  % Inlet efficiency at M=2 [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
Cd_engine   = .38;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 69000;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 25000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 69000;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 25000;  % Cruise dry thrust per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1350;  % Engine turbine inlet temperature [K]
CPR_e          = 12.5;  % Engine compressor pressure ratio [-]
BR_e           = 1.0;  % Engine bypass ratio [-]
FPR_e          = 1.65;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.8;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.82;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 0.83;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1350;  % Existing engine turbine temperature [K]
W_eng_e        = 1455*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.35;  % Airfoil maximum thickness/camber reference location [-]
% cl_max       = 2.6;
% CL_max       = 2.4;          % landing flaps
tdc          = 0.125;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 90;  % Sea-level stall speed [m/s]
c_f          = 0.0030;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 12;  % Nose half angle
sweep        = 25*pi/180;  % Leading edge sweep [rad]
A_base       = 4;  % Base area [m^2]
lambda       = 0.27;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 7.88;  % Wing aspect ratio [-]

% ============================================================
% B737-200 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 32.8;  % Horizontal-tail planform area [m^2]
b_H       = 12.9;  % Horizontal-tail span [m]
sweep_H   = 32*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 2.55;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 19.5;  % Vertical-tail/fin planform area [m^2]
b_V       = 6.0;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 3.25;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009; rho_sl = 1.226; T_sl = 288.15; R = 287.05;  % Specific heat of air [J/(kg*K)]
y = 1.4; g = 9.81; p_sl = 101300; radius_e = 6.371e6;  % Ratio of specific heats [-]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 8.1;  % Fin geometry ratio [-]
D_f    = 3.76;  % Fuselage maximum diameter [m]
L      = 30.53;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 3.76;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 102.0;  % Wing reference area [m^2]
S_csw  = 18.0;  % Elevon/control-surface area [m^2]
S_ht   = 31.3;  % Horizontal-tail/canard area [m^2]
S_vt   = 20.0;  % Vertical-tail area [m^2]
B_h    = 10.8;  % Horizontal-tail span/reference dimension [m]
F_w    = 3.76;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 14.5;  % Tail moment arm [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.8;  % Vertical-tail geometry parameter [-]
sweep_vt = 35*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.30*S_vt;  % Total control-surface area [m^2]

L_m  = 2.40;  % Main landing gear length [m]
L_n  = 1.80;  % Nose landing gear length [m]
N_mw = 4;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 2.40*3.28;  % Landing gear strut length [ft]

K_dw = 1.0; K_vs = 1.0; K_dwf = 1.0;  % Design wing weight correction factor [-]
K_rht = 1.0; K_uht = 1.0; K_door = 1.06; K_Lg = 1.0;  % Horizontal-tail correction factor [-]
K_cb = 1.0; K_tpg = 1.0; K_mp = 1.0; K_np = 1.0;  % Cabin correction factor [-]
K_vg = 1.0; K_d = 1.0; K_ws = 0.0; Ls_Ld = 0.20;  % Variable-geometry correction factor [-]
K_sys=1.65;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


D_e     = 1.24;  % Equivalent fuselage/cabin diameter [m]
L_d     = 2.0;  % Door/fuselage length parameter [m]
L_tp    = 2.0;  % Tail-pipe length [m]
L_sh    = 1.0;  % Shaft/structural length parameter [m]
L_ec    = 16.0;  % Equipment/cabin length [m]

V_t   = 17.9;  % Total fuel tank volume [m^3]
Vi_Vt = 0.85;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.15;  % Tank volume fraction [-]
N_t   = 4;  % Number of fuel tanks [-]
SFC   = 0.78;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 2; N_ci = 2.0; N_s = 2; N_u = 8;  % Number of crew [-]
Nf = 4; Nm = 2; W_uav = 600*9.81;  % Number of furnishings/functional units [-]

N_doors = 4; N_cargo_doors = 2;  % Number of passenger/service doors [-]
S_door = 1.4; S_cargo_door = 1.8;  % Passenger/service door area [m^2]
S_floor_main = 70; S_floor_upper = 0;  % Main-deck floor area [m^2]
w_main = 2.1/0.0208854; w_upper = 0;  % Main-deck floor areal weight [lb/ft^2]

%% ---- nacelle inputs (metric) ----
K_ng     = 1.017;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.18;  % Thrust reverser factor [-]
N_Lt     = 4.0;  % Intake + engine characteristic length [m]
N_w      = 1.30;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]
N_e      = N_en;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]
N_Lt_ft  = 13.12;  % Intake + engine length [ft]
N_w_ft   = 4.20;  % Equivalent nacelle width [ft]
S_n_ft   = 194;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e*.225;  % Engine weight per engine [lb]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 3000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 2000;  % Takeoff field length [m]
Landing_field_length = 1400;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude [m]
acc = 1.5;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=42.8E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 11300;  % Minimum search altitude [m]
max_alt = 11301;  % Maximum search altitude [m]
WdS_max = 7000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20; ii_max = 25; j_max = 20; k_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
q_max = 2; dd_max = 15; cc_max = 15; zzz_max = 30;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
xx_max = 20; iii_max = 30;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max = 30; kk_max = 30; kk_Dmax_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
f_6_k_e_i_low = 0; f_6_k_e_i_high = 0.5;  % Lower bound for existing engine afterburner fuel fraction flow rate


S0      = 102.00;  % Reference wing area [m^2]
b0      = 28.35;  % Reference wingspan [m]
c_r0    = 5.9;7.32;  % Reference root chord [m]
Lambda0 = 25*pi/180;  % Reference wing sweep [rad]

Alt0    = 11300;  % Reference cruise altitude [m]
R0      = 4800e3;  % Reference range [m]
L_D     = 14;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 22.7e-6;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 25*pi/180;





elseif aircraft_case==4






% ==================== MISSION REQUIREMENTS (777-200ER) ====================
M = 0.84;  % Cruise Mach number [-]
Range_cruise = 13080000;  % Cruise range [m] 
TOGW_i(1) = 297550*9.81;  % Initial takeoff gross weight [N] 
W_payload = 15000*9.81;  % payload weight [N] 
W_e(1)    = 145150*9.81;  % Initial empty operating weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload;  % Initial fuel weight [N]
Np        = 301;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (2 x GE90-94B class) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 7.66;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1400;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1750;  % High design turbine inlet temperature [K]
T_0_6_high  = 1750;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 700;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 40;  % Minimum compressor pressure ratio [-]
CPR_max     = 40;  % Maximum compressor pressure ratio [-]
BR_min      = 8.4;  % Minimum bypass ratio [-]
BR_max      = 8.4;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.98;  % Inlet efficiency at M=2 [-]
eta_c       = 0.88;  % Compressor efficiency [-]
eta_f       = 0.91;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.995;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.98;  % Nozzle efficiency [-]
Cd_engine   = 0.055;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 417000;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 70000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 417000;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 70000;  % Cruise dry thrust per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1750;  % Engine turbine inlet temperature [K]
CPR_e          = 40;  % Engine compressor pressure ratio [-]
BR_e           = 8.4;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]
eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.91;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.88;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 7.66;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1750;  % Existing engine turbine temperature [K]
W_eng_e        = 7550*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.38;  % Airfoil maximum thickness/camber reference location [-]
% cl_max       = 2.6;          % 2-D section CLmax (flaps)
% CL_max       = 2.5;          % 3-D wing CLmax landing — sizes S
tdc          = 0.12;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 101;  % Sea-level stall speed [m/s]
c_f          = 0.0028;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 12;  % Nose half angle
sweep        = 31.6*pi/180;  % Leading edge sweep [rad]
A_base       = 10;  % Base area [m^2]
lambda       = 0.21;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 8.68;  % Wing aspect ratio [-]

% ============================================================
% B777-200ER TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 75.0;  % Horizontal-tail planform area [m^2]
b_H       = 28.0;  % Horizontal-tail span [m]
sweep_H   = 31*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 5.5;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 52.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 12.8;  % Vertical-fin height/span [m]
sweep_V   = 37*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 5.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]


% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;  % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;  % Sea-level air density [kg/m^3]
T_sl = 288.15;  % Sea-level standard temperature [K]
R = 287.05;  % Specific gas constant for air [J/(kg*K)]
y = 1.4;  % Ratio of specific heats [-]
g = 9.81;  % Gravitational acceleration [m/s^2]
p_sl = 101300;  % Sea-level standard pressure [Pa]
radius_e = 6.371e6;  % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 10.3;  % Fin geometry ratio [-]
D_f    = 6.20;  % Fuselage maximum diameter [m]
L      = 63.73;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 6.20;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 427.8;  % Wing reference area [m^2]
S_csw  = 55.0;  % Elevon/control-surface area [m^2]
S_ht   = 101.0;  % Horizontal-tail/canard area [m^2]
S_vt   = 53.0;  % Vertical-tail area [m^2]
B_h    = 21.5;  % Horizontal-tail span/reference dimension [m]
F_w    = 6.20;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 32.0;  % Tail moment arm [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.8;  % Vertical-tail geometry parameter [-]
sweep_vt = 40*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.30*S_vt;  % Total control-surface area [m^2]

L_m  = 4.20;  % Main landing gear length [m]
L_n  = 3.20;  % Nose landing gear length [m]
N_mw = 12;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 4.20*3.28;  % Landing gear strut length [ft]

K_dw = 1.0;  % Design wing weight correction factor [-]
K_vs = 1.0;  % Vertical stabilizer correction factor [-]
K_dwf = 1.0;  % Design wing/fuselage correction factor [-]
K_rht = 1.0;  % Horizontal-tail correction factor [-]
K_uht = 1.0;  % Horizontal-tail correction factor [-]
K_door = 1.06;  % Door correction factor [-]
K_Lg = 1.0;  % Landing gear correction factor [-]
K_cb = 1.0;  % Cabin correction factor [-]
K_tpg = 1.0;  % Tail/pressure-group correction factor [-]
K_mp = 1.0;  % Main propulsion correction factor [-]
K_np = 1.0;  % Nose propulsion correction factor [-]
K_vg = 1.0;  % Variable-geometry correction factor [-]
K_d = 1.0;  % Drag/structural correction factor [-]
K_ws = 0.0;  % Wing sweep correction factor [-]
Ls_Ld = 0.15;  % Landing-gear length ratio [-]
K_sys=1.65;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


D_e     = 3.12;  % Equivalent fuselage/cabin diameter [m]
L_d     = 3.5;  % Door/fuselage length parameter [m]
L_tp    = 3.0;  % Tail-pipe length [m]
L_sh    = 2.0;  % Shaft/structural length parameter [m]
L_ec    = 28.0;  % Equipment/cabin length [m]

V_t   = 171.2;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]
N_t   = 4;  % Number of fuel tanks [-]
SFC   = 0.54;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 2;  % Number of crew [-]
N_ci = 2.0;  % Crew/instrumentation parameter [-]
N_s = 2;  % Number of seats/seat-related parameter [-]
N_u = 10;  % Number of utility/system units [-]
Nf = 5;  % Number of furnishings/functional units [-]
Nm = 2;  % Number of miscellaneous units [-]
W_uav = 1200*9.81;  % UAV/equipment weight [N]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]
R_kva = 120;  % Electrical system rating [kVA]

N_doors = 8;  % Number of passenger/service doors [-]
N_cargo_doors = 4;  % Number of cargo doors [-]
S_door = 1.8;  % Passenger/service door area [m^2]
S_cargo_door = 3.5;  % Cargo door area [m^2]
S_floor_main = 280;  % Main-deck floor area [m^2]
S_floor_upper = 0;  % Upper-deck floor area [m^2]
w_main = 2.1/0.0208854;  % Main-deck floor areal weight [lb/ft^2]
w_upper = 0;  % Upper-deck floor areal weight [lb/ft^2]

%% ---- nacelle 15.31 (pylon GE90) ----
K_ng     = 1.017;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.18;  % Thrust reverser factor [-]
N_Lt     = 7.3;  % Intake + engine characteristic length [m]
N_w      = 3.90;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]
N_e      = N_en;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]
N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 2500/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3200;  % Takeoff field length [m]
Landing_field_length = 1700;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude [m]
acc = 1.2;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 11550;  % Minimum search altitude [m]
max_alt = 11551;  % Maximum search altitude [m]
WdS_max = 8000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20; ii_max = 25; j_max = 20; k_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
q_max = 2;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15; cc_max = 15; zzz_max = 30;  % number of iterations from altitude=0 to altitude=design for the descent phase
xx_max = 20; iii_max = 30;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max = 30; kk_max = 30; kk_Dmax_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
f_6_k_e_i_low = 0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate

% book-check constants (table / percent error)
S0      = 427.8;  % Reference wing area [m^2]
b0      = 60.93;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.21));  % Reference root chord [m]
Lambda0 = 31.6*pi/180;  % Reference wing sweep [rad]
Alt0    = 11550;  % Reference cruise altitude [m]
R0      = 13080e3;  % Reference range [m]
L_D     = 18.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 0.54*2.832e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 31.64*pi/180;
% 
% 
% 
% 
% 
% 
elseif aircraft_case==5
% 
% 
% 
% 
% 
% 
% 
% 
% 
% ==================== MISSION REQUIREMENTS (A320-200) ====================
M = 0.82;  % Cruise Mach number [-]
Range_cruise = 5800000;  % Cruise range [m] 
TOGW_i(1) = 77000*9.81;  % Initial takeoff gross weight [N] 
W_payload = 8000*9.81;  % payload weight [N] 
W_e(1)    = 42200*9.81;  % Initial empty operating weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 180;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (2 x CFM56-5B class) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 2.35;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1550;  % High design turbine inlet temperature [K]
T_0_6_high  = 1550;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 600;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 32;  % Minimum compressor pressure ratio [-]
CPR_max     = 32;  % Maximum compressor pressure ratio [-]
BR_min      = 5.5;  % Minimum bypass ratio [-]
BR_max      = 5.5;  % Maximum bypass ratio [-]
FPR         = 1.7;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.98;  % Inlet efficiency at M=2 [-]
eta_c       = 0.87;  % Compressor efficiency [-]
eta_f       = 0.90;  % Fan efficiency [-]
eta_t       = 0.89;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.98;  % Nozzle efficiency [-]
Cd_engine   = 0.1018;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 120100;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 28000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 120100;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 28000;  % Cruise dry thrust per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1550;  % Engine turbine inlet temperature [K]
CPR_e          = 32;  % Engine compressor pressure ratio [-]
BR_e           = 5.5;  % Engine bypass ratio [-]
FPR_e          = 1.7;  % Engine fan pressure ratio [-]
eta_t_e        = 0.89;  % Engine turbine efficiency [-]
eta_f_e        = 0.90;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.87;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 2.35;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1550;  % Existing engine turbine temperature [K]
W_eng_e        = 2380*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.37;  % Airfoil maximum thickness/camber reference location [-]
% cl_max       = 2.6;          % 2-D flapped
% CL_max       = 2.5;          % 3-D landing — sizes S
tdc          = 0.125;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 92;  % Sea-level stall speed [m/s]
c_f          = 0.0029;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 12;  % Nose half angle
sweep        = 27*pi/180;  % Leading edge sweep [rad]
A_base       = 5;  % Base area [m^2]
lambda       = 0.24;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 9.48;  % Wing aspect ratio [-]

% ============================================================
% A320-200 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 31.0;  % Horizontal-tail planform area [m^2]
b_H       = 12.0;  % Horizontal-tail span [m]
sweep_H   = 28*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 2.75;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 21.5;  % Vertical-tail/fin planform area [m^2]
b_V       = 7.2;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 3.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009; rho_sl = 1.226; T_sl = 288.15; R = 287.05;  % Specific heat of air [J/(kg*K)]
y = 1.4; g = 9.81; p_sl = 101300; radius_e = 6.371e6;  % Ratio of specific heats [-]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 9.1;  % Fin geometry ratio [-]
D_f    = 4.14;  % Fuselage maximum diameter [m]
L      = 37.57;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 4.14;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 122.6;  % Wing reference area [m^2]
S_csw  = 22.0;  % Elevon/control-surface area [m^2]
S_ht   = 31.0;  % Horizontal-tail/canard area [m^2]
S_vt   = 21.5;  % Vertical-tail area [m^2]
B_h    = 12.4;  % Horizontal-tail span/reference dimension [m]
F_w    = 4.14;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 17.5;  % Tail moment arm [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.8;  % Vertical-tail geometry parameter [-]
sweep_vt = 35*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.30*S_vt;  % Total control-surface area [m^2]

L_m  = 2.80;  % Main landing gear length [m]
L_n  = 2.10;  % Nose landing gear length [m]
N_mw = 4;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 2.80*3.28;  % Landing gear strut length [ft]

K_dw = 1.0; K_vs = 1.0; K_dwf = 1.0;  % Design wing weight correction factor [-]
K_rht = 1.0; K_uht = 1.0; K_door = 1.06; K_Lg = 1.0;  % Horizontal-tail correction factor [-]
K_cb = 1.0; K_tpg = 1.0; K_mp = 1.0; K_np = 1.0;  % Cabin correction factor [-]
K_vg = 1.0; K_d = 1.0; K_ws = 0.0; Ls_Ld = 0.18;  % Variable-geometry correction factor [-]
K_sys = 1.50;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


D_e     = 1.83;  % Equivalent fuselage/cabin diameter [m]
L_d     = 2.5;  % Door/fuselage length parameter [m]
L_tp    = 2.2;  % Tail-pipe length [m]
L_sh    = 1.2;  % Shaft/structural length parameter [m]
L_ec    = 18.0;  % Equipment/cabin length [m]

V_t   = 24.2;  % Total fuel tank volume [m^3]
Vi_Vt = 0.88;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.12;  % Tank volume fraction [-]
N_t   = 4;  % Number of fuel tanks [-]
SFC   = 0.60;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 2; N_ci = 2.0; N_s = 2; N_u = 8;  % Number of crew [-]
Nf = 5; Nm = 2; W_uav = 700*9.81;  % Number of furnishings/functional units [-]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]
R_kva = 90;  % Electrical system rating [kVA]

N_doors = 4; N_cargo_doors = 2;  % Number of passenger/service doors [-]
S_door = 1.6; S_cargo_door = 2.2;  % Passenger/service door area [m^2]
S_floor_main = 110; S_floor_upper = 0;  % Main-deck floor area [m^2]
w_main = 2.1/0.0208854; w_upper = 0;  % Main-deck floor areal weight [lb/ft^2]

%% ---- nacelle 15.31 (pylon CFM56) ----
K_ng     = 1.017;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.18;  % Thrust reverser factor [-]
N_Lt     = 5.2;  % Intake + engine characteristic length [m]
N_w      = 2.10;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]
N_e      = N_en;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]
N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 2500/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 1800/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 2100;  % Takeoff field length [m]
Landing_field_length = 1500;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude [m]
acc = 1.3;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 11550;  % Minimum search altitude [m]
max_alt = 11551;  % Maximum search altitude [m]
WdS_max = 7000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20; ii_max = 25; j_max = 20; k_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
q_max = 2; dd_max = 15; cc_max = 15; zzz_max = 30;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
xx_max = 20; iii_max = 30;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max = 30; kk_max = 30; kk_Dmax_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
f_6_k_e_i_low = 0; f_6_k_e_i_high = 0.5;  % Lower bound for existing engine afterburner fuel fraction flow rate

% book checks
S0      = 122.6;  % Reference wing area [m^2]
b0      = 34.10;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.24));  % Reference root chord [m]
Lambda0 = 25*pi/180;  % Reference wing sweep [rad]
Alt0    = 11550;  % Reference cruise altitude [m]
R0      = 6100e3;  % Reference range [m]
L_D     = 16.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 0.60*2.832e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0=27*pi/180;


elseif aircraft_case==6
    
% ==================== MISSION REQUIREMENTS (Citation CJ3) ====================
M = 0.70;  % Cruise Mach number [-]
Range_cruise = 3500000;  % Cruise range [m] 
TOGW_i(1) = 6291*9.81;  % Initial takeoff gross weight [N] 
W_e(1)    = 3874*9.81;  % Initial empty operating weight [N] 
W_payload = 400*9.81;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 6;  % Passenger capacity [-]
type2     = 1;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (2 x Williams FJ44-3A) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.25;  % Inlet capture area per engine [m^2]
TIT_d_low   = 700;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1400;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 18;  % Minimum compressor pressure ratio [-]
CPR_max     = 18;  % Maximum compressor pressure ratio [-]
BR_min      = 3;  % Minimum bypass ratio [-]
BR_max      = 3;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.97;  % Inlet efficiency at M=2 [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
Cd_engine   = 0.35;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 12544;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 2500;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 12544;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 2500;  % Cruise dry thrust per engine [N]
TIT_e_low      = 700;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 18;  % Engine compressor pressure ratio [-]
BR_e           = 3;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 0.25;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1400;  % Existing engine turbine temperature [K]
W_eng_e        = 240*9.81;  % Engine weight per engine [N] (~7000 lbf class)

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.35;  % Airfoil maximum thickness/camber reference location [-]
% cl_max       = 1.8;
% CL_max       = 1.7;          % landing flaps — sizes S
tdc          = 0.14;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 54;  % Sea-level stall speed [m/s]
c_f          = 0.0032;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 10;  % Nose half angle
sweep        = 0*pi/180;  % Leading edge sweep [rad]
A_base       = 1.2;  % Base area [m^2]
lambda       = 0.30;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 9.68;  % Wing aspect ratio [-]

% ============================================================
% CESSNA CITATION CJ3 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 8.5;  % Horizontal-tail planform area [m^2]
b_H       = 5.5;  % Horizontal-tail span [m]
sweep_H   = 25*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 1.55;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 5.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 3.0;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 1.75;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009; rho_sl = 1.226; T_sl = 288.15; R = 287.05;  % Specific heat of air [J/(kg*K)]
y = 1.4; g = 9.81; p_sl = 101300; radius_e = 6.371e6;  % Ratio of specific heats [-]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.80;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
ratio_fin = 10.4;  % Fin geometry ratio [-]
D_f    = 1.50;  % Fuselage maximum diameter [m]
L      = 15.59;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 1.50;  % Fuselage width [m]
p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.0015;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 27.32;  % Wing reference area [m^2]
S_csw  = 4.0;  % Elevon/control-surface area [m^2]
S_ht   = 6.57;  % Horizontal-tail/canard area [m^2]
S_vt   = 5.23;  % Vertical-tail area [m^2]
B_h    = 6.35;  % Horizontal-tail span/reference dimension [m]
F_w    = 1.50;  % Fuselage width [m]
H_t_Hv = 1;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 7.5;  % Tail moment arm [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.6;  % Vertical-tail geometry parameter [-]
sweep_vt = 49*pi/180;  % Vertical-tail sweep [rad]
S_cs   = S_csw + 0.30*S_vt;  % Total control-surface area [m^2]

L_m  = 1.40;  % Main landing gear length [m]
L_n  = 1.10;  % Nose landing gear length [m]
N_mw = 2;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]
L_s  = 1.40*3.28;  % Landing gear strut length [ft]

K_dw = 1.0; K_vs = 1.0; K_dwf = 1.0;  % Design wing weight correction factor [-]
K_rht = 1.0; K_uht = 1.0; K_door = 1.0; K_Lg = 1.12;  % Horizontal-tail correction factor [-]
K_cb = 1.0; K_tpg = 1.0; K_mp = 1.0; K_np = 1.0;  % Cabin correction factor [-]
K_vg = 1.0; K_d = 1.0; K_ws = 0.0; Ls_Ld = 0.20;  % Variable-geometry correction factor [-]
K_sys = 1.00;  % Systems weight constant [-]
K_gr = 0.007; a_g = 0.95; b_g = 0.60;  % Landing gear weight coefficient [-]


D_e     = 0.56;  % Equivalent fuselage/cabin diameter [m]
L_d     = 1.2;  % Door/fuselage length parameter [m]
L_tp    = 1.0;  % Tail-pipe length [m]
L_sh    = 0.8;  % Shaft/structural length parameter [m]
L_ec    = 10.0;  % Equipment/cabin length [m]

V_t   = 2.66;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]
N_t   = 2;  % Number of fuel tanks [-]
SFC   = 0.55;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 2; N_ci = 2.0; N_s = 2; N_u = 6;  % Number of crew [-]
Nf = 4; Nm = 2; W_uav = 200*9.81;  % Number of furnishings/functional units [-]

K_vsh = 1.0; K_mc = 1.0; R_kva = 20;  % Vertical/horizontal stabilizer systems factor [-]

N_doors = 1; N_cargo_doors = 0;  % Number of passenger/service doors [-]
S_door = 0.8; S_cargo_door = 0.1;  % Passenger/service door area [m^2]
S_floor_main = 8; S_floor_upper = 0;  % Main-deck floor area [m^2]
w_main = 2.1/0.0208854; w_upper = 0;  % Main-deck floor areal weight [lb/ft^2]

%% ---- nacelle (aft-fuse pylons) ----
K_ng     = 1.0;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.0;  % Thrust reverser factor [-]
N_Lt     = 2.2;  % Intake + engine characteristic length [m]
N_w      = 0.70;  % Equivalent nacelle/inlet width [m]
S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]
N_e      = N_en;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]
N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]

% ==================== AEROTHERMAL (unused at M=0.70; kept for code) ====================
R_wle = 0.02;  % Wing leading-edge radius [ft]
g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity=0.8;  % Radiative emissivity [-]
Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
rho_TPS=40;  % TPS areal density [kg/m^2]
rho_titanium=4510;  % Titanium density [kg/m^3]
T_ins_max=581;  % Maximum insulation temperature [K]
T_sys_max=273+70;  % Maximum systems temperature [K]
k_titanium=21;  % Titanium thermal conductivity [W/(m*K)]
k_insulation=.3;  % Insulation thermal conductivity [W/(m*K)]
T_initial=230;  % Initial internal air temperature [K]
t_ins=.051;  % Insulation thickness [m]
t_tit=.001;  % Titanium thickness [m]
Cp_tit=523.5;  % Titanium specific heat [J/(kg*K)]
Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
rho_ins=112;  % Insulation density [kg/m^3]
S_cooling=0;  % Active cooling surface area [m^2]
rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
x = 4;  % Heating-analysis longitudinal location [m]
Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=42.8E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 4478/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 969;  % Takeoff field length [m]
Landing_field_length = 844;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude [m]
acc = 1.5;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 13716;  % Minimum search altitude [m]
max_alt = 13717;  % Maximum search altitude [m]
WdS_max = 4000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 20;  % Number of altitude points shown in design point plot [-]
i_max=20; ii_max=25; j_max=20; k_max=20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
q_max=2; dd_max=15; cc_max=15; zzz_max=30;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
xx_max=20; iii_max=30;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max=30; kk_max=30; kk_Dmax_max=30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
f_6_k_e_i_low=0; f_6_k_e_i_high=0.5;  % Lower bound for existing engine afterburner fuel fraction flow rate

% book checks
S0      = 27.32;  % Reference wing area [m^2]
b0      = 16.26;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.30));  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]
Alt0    = 13716;  % Reference cruise altitude [m]
R0      = 3780e3;  % Reference range [m]
L_D     = 13.0;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 1.898e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 0;
    
elseif aircraft_case==7
    
    
    
    
    
% ==================== MISSION REQUIREMENTS (Dassault Falcon 2000EX) ====================
M = 0.80;  % Cruise Mach number [-]
Range_cruise = 6800000;  % Cruise range [m] 
TOGW_i(1) = 19141*9.81;  % Initial takeoff gross weight [N] 
W_e(1)    = 10487*9.81;  % Initial empty operating weight [N] 
W_payload = 2000*4.44822;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 10;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION (2 x Pratt & Whitney Canada PW308C) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled

A_inlet     = 0.56;  % Inlet capture area per engine [m^2]
TIT_d_low   = 900;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1400;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]

CPR_min     = 25.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 25.0;  % Maximum compressor pressure ratio [-]
BR_min      = 3.5;  % Minimum bypass ratio [-]
BR_max      = 3.5;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]

n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]

eta_inlet   = 0.97;  % Inlet efficiency at M=2 [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
Cd_engine   = 0.37;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 31140;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 7030;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 31140;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 7030;  % Cruise dry thrust per engine [N]

TIT_e_low      = 900;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 25.0;  % Engine compressor pressure ratio [-]
BR_e           = 3.5;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]

eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 0.56;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1400;  % Existing engine turbine temperature [K]

W_eng_e        = 1336*4.44822;  % Engine weight per engine [N] (~7000 lbf class)
    
% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % Airfoil model/index [-]
delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.35;  % Airfoil maximum thickness/camber reference location [-]

% cl_max       = 1.6;
% CL_max       = 1.7;

tdc          = 0.12;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 70;  % Sea-level stall speed [m/s]
c_f          = 0.0032;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]

Su           = 110.4;  % Sutherland constant [K]
nose_angle   = 10;  % Nose half angle

sweep        = 0*pi/180;  % Leading edge sweep [rad]

A_base       = 1.2;  % Base area [m^2]
lambda       = 0.30;  % Taper/geometry parameter [-]
droop        = 0;  % Drooped nose: 0 = no droop
AR           = 7.63;  % Wing aspect ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;  % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;  % Sea-level air density [kg/m^3]
T_sl = 288.15;  % Sea-level standard temperature [K]
R = 287.05;  % Specific gas constant for air [J/(kg*K)]
y = 1.4;  % Ratio of specific heats [-]
g = 9.81;  % Gravitational acceleration [m/s^2]
p_sl = 101300;  % Sea-level standard pressure [Pa]
radius_e = 6.371e6;  % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ============================================================
% GULFSTREAM 2000EX / G200 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 13.5;  % Horizontal-tail planform area [m^2]
b_H       = 8.0;  % Horizontal-tail span [m]
sweep_H   = 28*pi/180;  % Horizontal-tail sweep [rad]
AR_H      = b_H^2/S_H;  % Horizontal-tail aspect ratio [-]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]

CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 1.70;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]


% ============================================================
% Vertical tail / fin
% ============================================================

S_V       = 8.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 4.5;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep [rad]
AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 2.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.80;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]

ratio_fin = 10.4;  % Fin geometry ratio [-]

D_f    = 2.34;  % Fuselage maximum diameter [m]
L      = 20.23;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.34;  % Fuselage width [m]

p_cabin = p_sl;  % Cabin pressure [Pa]
t_aluminum = 0.0015;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

S      = 49.00;  % Wing reference area [m^2]
S_csw  = 5.0;  % Elevon/control-surface area [m^2]
S_ht   = 8.0;  % Horizontal-tail/canard area [m^2]
S_vt   = 5.5;  % Vertical-tail area [m^2]

B_h    = 5.0;  % Horizontal-tail span/reference dimension [m]
F_w    = 2.34;  % Fuselage width [m]
H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]
L_t    = 8.0;  % Tail moment arm [m]

Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.6;  % Vertical-tail geometry parameter [-]
sweep_vt = 40*pi/180;  % Vertical-tail sweep [rad]

S_cs   = S_csw + 0.30*S_vt;  % Total control-surface area [m^2]

% ==================== LANDING GEAR ====================
L_m  = 1.40;  % Main landing gear length [m]
L_n  = 1.10;  % Nose landing gear length [m]

N_mw = 4;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]

L_s  = 1.40*3.28;  % Landing gear strut length [ft]


K_dw = 1.0;  % Design wing weight correction factor [-]
K_vs = 1.0;  % Vertical stabilizer correction factor [-]
K_dwf = 1.0;  % Design wing/fuselage correction factor [-]

K_rht = 1.0;  % Horizontal-tail correction factor [-]
K_uht = 1.0;  % Horizontal-tail correction factor [-]

K_door = 1.0;  % Door correction factor [-]
K_Lg = 1.12;  % Landing gear correction factor [-]

K_cb = 1.0;  % Cabin correction factor [-]
K_tpg = 1.0;  % Tail/pressure-group correction factor [-]
K_mp = 1.0;  % Main propulsion correction factor [-]
K_np = 1.0;  % Nose propulsion correction factor [-]

K_vg = 1.0;  % Variable-geometry correction factor [-]
K_d = 1.0;  % Drag/structural correction factor [-]
K_ws = 0.0;  % Wing sweep correction factor [-]
Ls_Ld = 0.20;  % Landing-gear length ratio [-]

K_sys = 1.00;  % Systems weight constant [-]
K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g = 0.95;  % Landing gear weight exponent [-]
b_g = 0.60;  % Landing gear strut length exponent [-]

% ==================== PROPULSION / NACELLE ====================

D_e     = 1.15;  % Equivalent fuselage/cabin diameter [m]
L_d     = 1.5;  % Door/fuselage length parameter [m]
L_tp    = 1.2;  % Tail-pipe length [m]
L_sh    = 1.0;  % Shaft/structural length parameter [m]
L_ec    = 10.0;  % Equipment/cabin length [m]

V_t   = 9.414;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]
N_t   = 2;  % Number of fuel tanks [-]

SFC   = 0.58;  % Cruise specific fuel consumption [lb/(lbf*hr)]

N_c = 2;  % Number of crew [-]
N_ci = 2.0;  % Crew/instrumentation parameter [-]
N_s = 2;  % Number of seats/seat-related parameter [-]
N_u = 6;  % Number of utility/system units [-]

Nf = 4;  % Number of furnishings/functional units [-]
Nm = 2;  % Number of miscellaneous units [-]

W_uav = 300*9.81;  % UAV/equipment weight [N]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]
R_kva = 15;  % Electrical system rating [kVA]

N_doors       = 2;  % Number of passenger/service doors [-]
N_cargo_doors = 1;  % Number of cargo doors [-]

S_door        = 1.5;  % Passenger/service door area [m^2]
S_cargo_door  = 2.0;  % Cargo door area [m^2]

S_floor_main  = 35;  % Main-deck floor area [m^2]
S_floor_upper = 0;  % Upper-deck floor area [m^2]

w_main  = 2.1/0.0208854;  % Main-deck floor areal weight [lb/ft^2]
w_upper = 0;  % Upper-deck floor areal weight [lb/ft^2]

% ==================== NACELLE (AFT-FUSE) ====================
K_ng     = 1.0;  % Nacelle installation factor: not pylon-mounted [-]
K_p      = 1.0;  % Pylon factor [-]
K_tr     = 1.0;  % Thrust reverser factor [-]

N_Lt     = 2.2;  % Intake + engine characteristic length [m]
N_w      = 1.00;  % Equivalent nacelle/inlet width [m]

S_n      = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]

N_e      = N_en;  % Number of engines [-]
W_en     = W_eng_e;  % Engine weight per engine [N]

N_Lt_ft  = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft   = N_w/0.3048;  % Equivalent nacelle width [ft]
S_n_ft   = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]
W_en_lb  = W_eng_e/4.44822;  % Engine weight per engine [lb]


% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 4375/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]

Takeoff_field_length = 1700;  % Takeoff field length [m]
Landing_field_length = 650;  % Landing field length [m]

g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]

alt_0 = 0;  % Initial altitude [m]
acc = 1.5;  % Acceleration [m/s^2]

V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=43.15E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 14200;  % Minimum search altitude [m]
max_alt = 14201;  % Maximum search altitude [m]

WdS_max = 4000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 20;  % Number of altitude points shown in design point plot [-]

i_max=20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;  % number of iterations for finding the existing engine turbine inlet temperature during cruise [-]
j_max=20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;  % number of iterations for finding the ideal engine turbine inlet temperature during cruise [-]


q_max=2;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max=15;  % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max=15;  % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max=30;  % number of points used for the carpet plot that has smoothed out curves

xx_max=20;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max=30;  % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft

ff_max=30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max=30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max=30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition

f_6_k_e_i_low=0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high=0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate

% ==================== BOOK CHECKS ====================
S0      = 49.00;  % Reference wing area [m^2]
b0      = 19.33;  % Reference wingspan [m]

c_r0    = 2*S0/(b0*(1+0.30));  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]

Alt0    = 14326;  % Reference cruise altitude [m]
R0      = 7180000;  % Reference range [m]

L_D     = 13.0;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 1.84e-5;  % Reference cruise TSFC [kg/(N*s)]

sweep0  = 0;

elseif aircraft_case==8
    
% ============================================================
% XB-70 VALKYRIE INPUTS
% ============================================================

% ==================== MISSION REQUIREMENTS ====================
M = 3.0;  % Cruise Mach number [-]
Range_cruise = 6600000;  % Cruise range [m] 

TOGW_i(1) = 242000*9.81;  % Initial takeoff gross weight [N] 
W_c0      = 214090*9.81; % Book weight after climb
W_cf      = 325000/.225; % Book weight before descent
W_e(1)    = 115000*9.81;  % Initial empty operating weight [N]
W_payload = 0*9.81;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 0;  % Passenger capacity [-]
type2     = 2;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 




% ==================== PROPULSION ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 6;  % Number of engines used in propulsion calculation [-]
N_en        = 6;  % Total number of engines [-]
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 1;  % Waverider configuration: 0 = disabled, 1 = enabled

A_inlet     = 1.204;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1900;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]

CPR_min     = 8.75;  % Minimum compressor pressure ratio [-]
CPR_max     = 8.75;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]

n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.5;  % High afterburner setting [-]

eta_inlet   = 0.92;  % Inlet efficiency at M=2 [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]

Cd_engine   = .9;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]


% ==================== EXISTING ENGINE DATA ====================
% General Electric YJ93 turbojet

Th_wet         = 128000;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 115000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 85000;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 80000;  % Cruise dry thrust per engine [N]

TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 8.75;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]

eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 1.204;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1900;  % Existing engine turbine temperature [K]

W_eng_e        = 2700*9.81;  % Engine weight per engine [N] (~7000 lbf class)


% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 5;  % Airfoil model/index [-]

delta        = 1;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.50;  % Airfoil maximum thickness/camber reference location [-]

cl_max       = 1.20;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 0.80;  % Aircraft maximum lift coefficient [-] (low-AR delta)

tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]

V_stall_sl   = 90;  % Sea-level stall speed [m/s]

c_f          = 0.0020;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]

nose_angle   = 3;  % Nose half angle

sweep        = 65*pi/180;  % Leading edge sweep [rad]

A_base       = 8.0;  % Base area [m^2]
lambda       = 0.15;  % Taper/geometry parameter [-]
droop        = 1;  % Drooped nose: 0 = no droop

AR           = 1.75;  % Wing aspect ratio [-]


% ==================== XB-70 TAIL INPUTS ====================

% ------------------------------------------------------------
% Horizontal tail
% ------------------------------------------------------------
% XB-70 has no conventional horizontal tail.
% Pitch control is provided by the canards/elevons.

S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail sweep [rad]
AR_H      = 0;  % Horizontal-tail aspect ratio [-]
e_H       = 0;  % Horizontal-tail Oswald efficiency [-]
CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0;  % Horizontal-tail thickness-to-chord ratio [-]


% ------------------------------------------------------------
% Vertical tails / fins
% ------------------------------------------------------------

% Two vertical fins are mounted on the outboard wing/nacelle region.
S_V       = 38.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 8.0;  % Vertical-fin height/span [m]
sweep_V   = 55*pi/180;  % Vertical-fin leading-edge sweep [rad]

AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 4.75;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.06;  % Vertical-tail maximum thickness-to-chord ratio [-]

CD0_V     = 0.006;


% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;  % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;  % Sea-level air density [kg/m^3]
T_sl = 288.15;  % Sea-level standard temperature [K]
R = 287.05;  % Specific gas constant for air [J/(kg*K)]

y = 1.4;  % Ratio of specific heats [-]
g = 9.81;  % Gravitational acceleration [m/s^2]

p_sl = 101300;  % Sea-level standard pressure [Pa]
radius_e = 6.371e6;  % Earth radius [m]

mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su   = 110.4;  % Sutherland constant [K]


% ==================== STRUCTURAL / RAYMER GEOMETRY ====================

N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]

ratio_fin = 19;  % Fin geometry ratio [-]

D_f    = 3.05;  % Fuselage maximum diameter [m]
L      = 56.64;  % Fuselage/aircraft length (0 = solve for) [m]

W_fuse = 3.05;  % Fuselage width [m]

p_cabin = p_sl;  % Cabin pressure [Pa]


t_aluminum  = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

skin=3; %1=aluminum, 2=titanium, 3=stainless steel


% ==================== MAIN WING ====================

S      = 585.0;  % Wing reference area [m^2]
S_csw  = 50.0;  % Elevon/control-surface area [m^2]
S_ht   = 0;  % Horizontal-tail/canard area [m^2]
S_vt   = 38.0;  % Vertical-tail area [m^2]

B_h    = 0;  % Horizontal-tail span/reference dimension [m]
F_w    = 3.05;  % Fuselage width [m]

H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]

L_t    = 20.0;  % Tail moment arm [m]

Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.4;  % Vertical-tail geometry parameter [-]

sweep_vt = 55*pi/180;  % Vertical-tail sweep [rad]

S_cs   = S_csw + 0.35*S_vt;  % Total control-surface area [m^2]


% ==================== FUSELAGE / LANDING GEAR ====================

L_m  = 3.50;  % Main landing gear length [m]
L_n  = 2.80;  % Nose landing gear length [m]

N_mw = 8;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]

L_s  = 3.50*3.28;  % Landing gear strut length [ft]


% ==================== RAYMER CONSTANTS ====================

K_dw = 0.768;  % Design wing weight correction factor [-]
K_vs = 1.0;  % Vertical stabilizer correction factor [-]
K_dwf = 0.774;  % Design wing/fuselage correction factor [-]

K_rht = 1.0;  % Horizontal-tail correction factor [-]
K_uht = 1.0;  % Horizontal-tail correction factor [-]

K_door = 1.06;  % Door correction factor [-]
K_Lg = 1.12;  % Landing gear correction factor [-]

K_cb = 1.0;  % Cabin correction factor [-]
K_tpg = 1.0;  % Tail/pressure-group correction factor [-]

K_mp = 1.0;  % Main propulsion correction factor [-]
K_np = 1.0;  % Nose propulsion correction factor [-]

K_vg = 1.0;  % Variable-geometry correction factor [-]

K_d = 1.75;  % Drag/structural correction factor [-]

K_ws = 0.0;  % Wing sweep correction factor [-]
Ls_Ld = 0.40;  % Landing-gear length ratio [-]

K_sys = 1.65;  % Systems weight constant [-]

K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


% ==================== ENGINE / NACELLE ====================


D_e     = 1.15;  % Equivalent fuselage/cabin diameter [m]

L_d     = 8.0;  % Door/fuselage length parameter [m]
L_tp    = 2.0;  % Tail-pipe length [m]
L_sh    = 1.5;  % Shaft/structural length parameter [m]
L_ec    = 25.0;  % Equipment/cabin length [m]


% ==================== FUEL / VOLUME ====================

V_t   = 167.0;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]

N_t   = 17;  % Number of fuel tanks [-]

SFC   = 1.20;  % Cruise specific fuel consumption [lb/(lbf*hr)]


% ==================== SYSTEMS ====================

N_c  = 3;  % Number of crew [-]
N_ci = 2.0;  % Crew/instrumentation parameter [-]
N_s  = 2;  % Number of seats/seat-related parameter [-]
N_u  = 8;  % Number of utility/system units [-]

Nf = 5;  % Number of furnishings/functional units [-]
Nm = 2;  % Number of miscellaneous units [-]

W_uav = 800*9.81;  % UAV/equipment weight [N]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]

R_kva = 80;  % Electrical system rating [kVA]


% ==================== DOORS / FLOOR ====================

N_doors = 0;  % Number of passenger/service doors [-]
N_cargo_doors = 0;  % Number of cargo doors [-]

S_door = 0;  % Passenger/service door area [m^2]
S_cargo_door = 0;  % Cargo door area [m^2]

S_floor_main = 0;  % Main-deck floor area [m^2]
S_floor_upper = 0;  % Upper-deck floor area [m^2]

w_main = 0;  % Main-deck floor areal weight [lb/ft^2]
w_upper = 0;  % Upper-deck floor areal weight [lb/ft^2]


% ==================== NACELLE / INTAKE ====================

K_ng = 1.0;  % Nacelle installation factor: not pylon-mounted [-]
K_p  = 1.0;  % Pylon factor [-]

K_tr = 1.18;  % Thrust reverser factor [-]

N_Lt = 8.0;  % Intake + engine characteristic length [m]
N_w  = 1.238;  % Equivalent nacelle/inlet width [m]

S_n = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]

N_e = 6;  % Number of engines [-]

W_en = W_eng_e;  % Engine weight per engine [N]

N_Lt_ft = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft  = N_w/0.3048;  % Equivalent nacelle width [ft]

S_n_ft = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]

W_en_lb = W_eng_e/4.44822;  % Engine weight per engine [lb]


% ==================== PERFORMANCE / TRAJECTORY ====================

rate_of_climb    = 27000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 10000/3.28/60;  % Rate of descent [m/s]

Takeoff_field_length = 3200;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]

g_turn = 1.2;  % Turning/acceleration load factor [-]

mu_2 = 0.03;  % Ground friction coefficient [-]

alt_0 = 1000;  % Initial altitude [m]

acc = 2.0;  % Acceleration [m/s^2]

V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]


% ==================== AEROTHERMAL PARAMETERS ====================

TPS = 1;  % Thermal protection system: 0=no, 1=yes
R_wle = 0.344/12;  % Wing leading-edge radius [ft]
g_s = g*3.2797;  % Gravity conversion parameter [ft/s^2]
J = 778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity = 0.8;  % Radiative emissivity [-]
Cp_wall = 1000;  % Wall/TPS specific heat [J/(kg*K)]
rho_TPS = 40;  % TPS areal density [kg/m^2]
rho_titanium = 4510;  % Titanium density [kg/m^3]
T_ins_max = 581;  % Maximum insulation temperature [K]
T_sys_max = 273+70;  % Maximum systems temperature [K]
k_titanium = 21;  % Titanium thermal conductivity [W/(m*K)]
k_insulation = 0.3;  % Insulation thermal conductivity [W/(m*K)]
T_initial = 230;  % Initial internal air temperature [K]
t_ins = 0.0254;  % Insulation thickness [m]
t_tit = 0.0015;  % Titanium thickness [m]
Cp_tit = 523.5;  % Titanium specific heat [J/(kg*K)]
Cp_ins = 500;  % Insulation specific heat [J/(kg*K)]
rho_ins = 379.392;  % Insulation density [kg/m^3]
S_cooling = 39;  % Active cooling surface area [m^2]
rho_cooling = 300;  % Active cooling surface areal density [kg/m^2]
x = 4;  % Heating-analysis longitudinal location [m]
Pr = 0.72;  % Prandtl number [-]


% ==================== FUEL PROPERTIES ====================

E_sp = 44E6;  % Fuel specific energy [J/kg]

ratio_fo = 0.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]

Cp_f = 2010;  % Fuel specific heat [J/(kg*K)]


% ==================== ITERATION / SEARCH PARAMETERS ====================

min_alt = 23000;  % Minimum search altitude [m]
max_alt = 23001;  % Maximum search altitude [m]

WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]

alt_points = 200;  % Number of altitude points shown in design point plot [-]

i_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;  % number of iterations for finding the existing engine turbine inlet temperature during cruise [-]
j_max = 20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;  % number of iterations for finding the ideal engine turbine inlet temperature during cruise [-]

q_max = 2;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;  % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;  % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;  % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft

iii_max = 30;  % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft

ff_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition

f_6_k_e_i_low = 0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate


% ============================================================
% XB-70 VERIFICATION AIRCRAFT DATA
% ============================================================

S0      = 585.0;  % Reference wing area [m^2]
b0      = 32.0;  % Reference wingspan [m]
c_r0    = 36.5;  % Reference root chord [m]
Lambda0 = 65*pi/180;  % Reference wing sweep [rad]

Alt0    = 23000;  % Reference cruise altitude [m]
R0      = 6900e3;  % Reference range [m]

L_D     = 6.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 51e-6;  % Reference cruise TSFC [kg/(N*s)]

elseif aircraft_case==9
    
    % ============================================================
% SR-71 BLACKBIRD VERIFICATION AIRCRAFT
% ============================================================

% ==================== MISSION REQUIREMENTS ====================

M = 3.20;  % Cruise Mach number [-]
Range_cruise = 3400000;  % Cruise range [m] 

TOGW_i(1) = 78000*9.81;  % Initial takeoff gross weight [N] 
W_e(1)    = 30600*9.81;  % Initial empty operating weight [N] 
W_payload = 1600*9.81;  % payload weight [N] 
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]

Np        = 0;  % Passenger capacity [-]
type2     = 2;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family 


% ==================== PROPULSION ====================

type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
n           = 2;  % Number of engines used in propulsion calculation [-]
N_en        = 2;  % Total number of engines [-]
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 2;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled

A_inlet     = 0.90;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1900;  % High turbine/combustor total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]

CPR_min     = 8.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 8.0;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]

n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.5;  % High afterburner setting [-]

eta_inlet   = 0.92;  % Inlet efficiency at M=2 [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]

Cd_engine   = 0.55;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]


% ==================== EXISTING ENGINE DATA ====================
% Pratt & Whitney J58

Th_wet         = 151000;  % Maximum wet/reheat thrust per engine [N]
Th_wet_cruise  = 140000;  % Cruise wet/reheat thrust per engine [N]
Th_dry         = 107000;  % Maximum dry thrust per engine [N]
Th_dry_cruise  = 100000;  % Cruise dry thrust per engine [N]

TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine turbine inlet temperature [K]
CPR_e          = 8.0;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]

eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
A_inlet_e      = 0.90;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1900;  % Existing engine turbine temperature [K]

W_eng_e        = 2720*9.81;  % Engine weight per engine [N] (~7000 lbf class)


% ==================== AERODYNAMICS ====================

Subsonic_airfoil = 5;  % Airfoil model/index [-]

delta        = 0;  % Delta-wing flag: 0 = conventional/defined geometry
sweep_solver = 0;  % Sweep calculation method/index [-]
x_c          = 0.50;  % Airfoil maximum thickness/camber reference location [-]

cl_max       = 1.20;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 0.80;  % Aircraft maximum lift coefficient [-] (low-AR delta)

tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]

V_stall_sl   = 95;  % Sea-level stall speed [m/s]

c_f          = 0.0020;  % Skin-friction coefficient [-]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
Su           = 110.4;  % Sutherland constant [K]

nose_angle   = 3;  % Nose half angle

sweep        = 60*pi/180;  % Leading edge sweep [rad]

A_base       = 8.0;  % Base area [m^2]
lambda       = 0.01;  % Taper/geometry parameter [-]
droop        = 1;  % Drooped nose: 0 = no droop

AR           = 1.70;  % Wing aspect ratio [-]


% ==================== SR-71 TAIL INPUTS ====================

% ------------------------------------------------------------
% Horizontal tail
% ------------------------------------------------------------

S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail sweep [rad]
AR_H      = 0;  % Horizontal-tail aspect ratio [-]
e_H       = 0;  % Horizontal-tail Oswald efficiency [-]
CL_H      = 0;  % Horizontal-tail lift coefficient [-]
alpha_H   = 0;  % Horizontal-tail angle of attack [rad]

c_H       = 0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0;  % Horizontal-tail thickness-to-chord ratio [-]


% ------------------------------------------------------------
% Vertical tails
% ------------------------------------------------------------

% Two large canted vertical stabilizers

S_V       = 23.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 5.0;  % Vertical-fin height/span [m]
sweep_V   = 15*pi/180;  % Vertical-fin leading-edge sweep [rad]

AR_V      = b_V^2/S_V;  % Vertical-fin aspect ratio [-]

e_V       = 0.80;  % Vertical-fin Oswald efficiency [-]
CL_V      = 0;  % Vertical-fin lift coefficient [-]
beta_V    = 0;  % Sideslip angle [rad]

c_V       = 4.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.06;  % Vertical-tail maximum thickness-to-chord ratio [-]

CD0_V     = 0.006;


% ==================== ATMOSPHERE / EARTH ====================

Cp = 1009;  % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;  % Sea-level air density [kg/m^3]
T_sl = 288.15;  % Sea-level standard temperature [K]
R = 287.05;  % Specific gas constant for air [J/(kg*K)]

y = 1.4;  % Ratio of specific heats [-]
g = 9.81;  % Gravitational acceleration [m/s^2]

p_sl = 101300;  % Sea-level standard pressure [Pa]
radius_e = 6.371e6;  % Earth radius [m]

mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su   = 110.4;  % Sutherland constant [K]


% ==================== STRUCTURAL / RAYMER GEOMETRY ====================

N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]

ratio_fin = 19;  % Fin geometry ratio [-]

D_f    = 1.70;  % Fuselage maximum diameter [m]
L      = 32.74;  % Fuselage/aircraft length (0 = solve for) [m]

W_fuse = 1.70;  % Fuselage width [m]

p_cabin = p_sl;  % Cabin pressure [Pa]


t_aluminum  = 0.002;  % Aluminum skin thickness [m]
rho_aluminum = 2800;  % Aluminum density [kg/m^3]

skin=2; %1=aluminum, 2=titanium, 3=stainless steel


% ==================== MAIN WING ====================

S      = 167.2;  % Wing reference area [m^2]
S_csw  = 20.0;  % Elevon/control-surface area [m^2]
S_ht   = 0;  % Horizontal-tail/canard area [m^2]
S_vt   = 23.0;  % Vertical-tail area [m^2]

B_h    = 0;  % Horizontal-tail span/reference dimension [m]
F_w    = 1.70;  % Fuselage width [m]

H_t_Hv = 0;  % Horizontal-tail to vertical-tail parameter [-]

L_t    = 10.0;  % Tail moment arm [m]

Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
A_vt   = 1.4;  % Vertical-tail geometry parameter [-]

sweep_vt = 15*pi/180;  % Vertical-tail sweep [rad]

S_cs   = S_csw + 0.35*S_vt;  % Total control-surface area [m^2]


% ==================== FUSELAGE / LANDING GEAR ====================

L_m  = 3.50;  % Main landing gear length [m]
L_n  = 2.80;  % Nose landing gear length [m]

N_mw = 8;  % Main landing gear wheels [-]
N_nw = 2;  % Nose landing gear wheels [-]
N_mss = 2;  % Main landing gear struts/assemblies [-]

L_s  = 3.50*3.28;  % Landing gear strut length [ft]


% ==================== RAYMER CONSTANTS ====================

K_dw = 0.768;  % Design wing weight correction factor [-]
K_vs = 1.0;  % Vertical stabilizer correction factor [-]
K_dwf = 0.774;  % Design wing/fuselage correction factor [-]

K_rht = 1.0;  % Horizontal-tail correction factor [-]
K_uht = 1.0;  % Horizontal-tail correction factor [-]

K_door = 1.06;  % Door correction factor [-]
K_Lg = 1.12;  % Landing gear correction factor [-]

K_cb = 1.0;  % Cabin correction factor [-]
K_tpg = 1.0;  % Tail/pressure-group correction factor [-]

K_mp = 1.0;  % Main propulsion correction factor [-]
K_np = 1.0;  % Nose propulsion correction factor [-]

K_vg = 1.0;  % Variable-geometry correction factor [-]

K_d = 1.75;  % Drag/structural correction factor [-]

K_ws = 0.0;  % Wing sweep correction factor [-]
Ls_Ld = 0.40;  % Landing-gear length ratio [-]

K_sys = 1.65;  % Systems weight constant [-]

K_gr = 0.007;  % Landing gear weight coefficient [-]
a_g  = 0.95;  % Landing gear weight exponent [-]
b_g  = 0.60;  % Landing gear strut length exponent [-]


% ==================== ENGINE / NACELLE ====================


D_e     = 1.27;  % Equivalent fuselage/cabin diameter [m]

L_d     = 4.57;  % Door/fuselage length parameter [m]
L_tp    = 1.5;  % Tail-pipe length [m]
L_sh    = 1.0;  % Shaft/structural length parameter [m]
L_ec    = 15.0;  % Equipment/cabin length [m]


% ==================== FUEL / VOLUME ====================

V_t   = 46.3;  % Total fuel tank volume [m^3]
Vi_Vt = 0.90;  % Usable/installed tank volume ratio [-]
Vp_Vt = 0.10;  % Tank volume fraction [-]

N_t   = 9;  % Number of fuel tanks [-]

SFC   = 1.20;  % Cruise specific fuel consumption [lb/(lbf*hr)]


% ==================== SYSTEMS ====================

N_c  = 3;  % Number of crew [-]
N_ci = 2.0;  % Crew/instrumentation parameter [-]
N_s  = 2;  % Number of seats/seat-related parameter [-]
N_u  = 8;  % Number of utility/system units [-]

Nf = 5;  % Number of furnishings/functional units [-]
Nm = 2;  % Number of miscellaneous units [-]

W_uav = 800*9.81;  % UAV/equipment weight [N]

K_vsh = 1.0;  % Vertical/horizontal stabilizer systems factor [-]
K_mc  = 1.0;  % Mechanical/control systems factor [-]

R_kva = 80;  % Electrical system rating [kVA]


% ==================== DOORS / FLOOR ====================

N_doors = 0;  % Number of passenger/service doors [-]
N_cargo_doors = 0;  % Number of cargo doors [-]

S_door = 0;  % Passenger/service door area [m^2]
S_cargo_door = 0;  % Cargo door area [m^2]

S_floor_main = 0;  % Main-deck floor area [m^2]
S_floor_upper = 0;  % Upper-deck floor area [m^2]

w_main = 0;  % Main-deck floor areal weight [lb/ft^2]
w_upper = 0;  % Upper-deck floor areal weight [lb/ft^2]


% ==================== NACELLE / INTAKE ====================

K_ng = 1.0;  % Nacelle installation factor: not pylon-mounted [-]
K_p  = 1.0;  % Pylon factor [-]

K_tr = 1.18;  % Thrust reverser factor [-]

N_Lt = 4.57;  % Intake + engine characteristic length [m]
N_w  = 1.27;  % Equivalent nacelle/inlet width [m]

S_n = pi*N_w*N_Lt;  % Equivalent nacelle wetted/reference area [m^2]

N_e = 2;  % Number of engines [-]

W_en = W_eng_e;  % Engine weight per engine [N]

N_Lt_ft = N_Lt/0.3048;  % Intake + engine length [ft]
N_w_ft  = N_w/0.3048;  % Equivalent nacelle width [ft]

S_n_ft = S_n/0.3048^2;  % Equivalent nacelle area [ft^2]

W_en_lb = W_eng_e/4.44822;  % Engine weight per engine [lb]


% ==================== PERFORMANCE / TRAJECTORY ====================

rate_of_climb    = 60.0;  % Rate of climb [m/s]
rate_of_descent  = 7.0;  % Rate of descent [m/s]

Takeoff_field_length = 3000;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]

g_turn = 1.2;  % Turning/acceleration load factor [-]

mu_2 = 0.03;  % Ground friction coefficient [-]

alt_0 = 1000;  % Initial altitude [m]

acc = 2.0;  % Acceleration [m/s^2]

V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]


% ==================== AEROTHERMAL PARAMETERS ====================

TPS = 1;  % Thermal protection system: 0=no, 1=yes

R_wle = 0.344/12;  % Wing leading-edge radius [ft]

g_s = g*3.2797;  % Gravity conversion parameter [ft/s^2]

J = 778;  % Mechanical equivalent of heat [ft*lbf/BTU]

emmissivity = 0.8;  % Radiative emissivity [-]

Cp_wall = 1000;  % Wall/TPS specific heat [J/(kg*K)]

rho_TPS = 40;  % TPS areal density [kg/m^2]
rho_titanium = 4510;  % Titanium density [kg/m^3]

T_ins_max = 581;  % Maximum insulation temperature [K]
T_sys_max = 273+70;  % Maximum systems temperature [K]

k_titanium = 21;  % Titanium thermal conductivity [W/(m*K)]
k_insulation = 0.3;  % Insulation thermal conductivity [W/(m*K)]

T_initial = 230;  % Initial internal air temperature [K]

t_ins = 0;  % Insulation thickness [m]
t_tit = 0.0015;  % Titanium thickness [m]

Cp_tit = 523.5;  % Titanium specific heat [J/(kg*K)]
Cp_ins = 500;  % Insulation specific heat [J/(kg*K)]

rho_ins = 112;  % Insulation density [kg/m^3]

S_cooling = 20;  % Active cooling surface area [m^2] %Assumed
rho_cooling = 300;  % Active cooling surface areal density [kg/m^2] %Assumed

x = 4;  % Heating-analysis longitudinal location [m]

Pr = 0.72;  % Prandtl number [-]


% ==================== FUEL PROPERTIES ====================

E_sp = 43E6;  % Fuel specific energy [J/kg]

ratio_fo = 0.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]

Cp_f = 2010;  % Fuel specific heat [J/(kg*K)]


% ==================== ITERATION / SEARCH PARAMETERS ====================

min_alt = 24000;  % Minimum search altitude [m]
max_alt = 24001;  % Maximum search altitude [m]

WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]

alt_points = 200;  % Number of altitude points shown in design point plot [-]

i_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;  % number of iterations for finding the existing engine turbine inlet temperature during cruise [-]
j_max = 20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;  % number of iterations for finding the ideal engine turbine inlet temperature during cruise [-]
q_max = 2;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;  % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;  % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;  % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;  % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft

iii_max = 30;  % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft

ff_max = 30;  % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition

f_6_k_e_i_low = 0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate


% ============================================================
% SR-71 VERIFICATION AIRCRAFT DATA
% ============================================================

S0      = 167.2;  % Reference wing area [m^2]
b0      = 16.94;  % Reference wingspan [m]
c_r0    = 19.0;  % Reference root chord [m]
Lambda0 = 60*pi/180;  % Reference wing sweep [rad]

Alt0    = 24000;  % Reference cruise altitude [m]
R0      = 5400e3;  % Reference range [m]

L_D     = 5.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 50e-6;  % Reference cruise TSFC [kg/(N*s)]
    
end

CL_ww=[0 0.1 0.3 0.4 0.5 0.6 0.8];
S_ww=[.91 .91 .925 .9275 .935 .935 .915];


if M<1
if Subsonic_airfoil==1
%sc20012-il - NASA SC(2)-0012 AIRFOIL
Alpha=[	-19	-18.75	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25];%	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5];
Cl=	[-1.1266	-1.1716	-1.2097	-1.2439	-1.273	-1.2971	-1.316	-1.3317	-1.3438	-1.3533	-1.3594	-1.3622	-1.3732	-1.3827	-1.3875	-1.3882	-1.3862	-1.382	-1.376	-1.3684	-1.3598	-1.35	-1.3396	-1.3286	-1.3172	-1.3043	-1.2883	-1.2707	-1.252	-1.232	-1.214	-1.1955	-1.1743	-1.152	-1.1292	-1.1059	-1.0823	-1.0601	-1.0359	-1.0111	-0.9874	-0.9623	-0.9375	-0.9127	-0.8872	-0.8633	-0.8329	-0.7983	-0.7662	-0.7364	-0.7108	-0.6872	-0.6621	-0.636	-0.6096	-0.5831	-0.5557	-0.5285	-0.5011	-0.4734	-0.446	-0.4185	-0.3908	-0.3634	-0.3364	-0.3093	-0.2826	-0.2565	-0.2311	-0.204	-0.1749	-0.1456	-0.1168	-0.0874	-0.0582	-0.0291	0	0.0291	0.0582	0.0873	0.1168	0.1456	0.1749	0.204	0.2311	0.2565	0.2826	0.3093	0.3364	0.3634	0.3908	0.4185	0.446	0.4734	0.5011	0.5286	0.5557	0.5831	0.6096	0.636	0.6621	0.6872	0.7109	0.7364	0.7663	0.7984	0.833	0.8633	0.8872	0.9127	0.9375	0.9624	0.9874	1.0111	1.036	1.0602	1.0823	1.106	1.1293	1.1522	1.1745	1.1956	1.2142	1.2322	1.2522	1.271	1.2886	1.3047	1.3177	1.3292	1.3402	1.3507	1.3604	1.3692	1.3768	1.3829	1.3872	1.3893];	%1.3886	1.3841	1.3745	1.3647	1.3614	1.3548	1.3458	1.3332	1.3181	1.2991	1.275	1.2464	1.2122];
Cd=	[0.13239	0.11888	0.10714	0.09645	0.08682	0.07837	0.07112	0.06485	0.0596	0.05507	0.05128	0.04805	0.04408	0.04049	0.03754	0.0351	0.03302	0.03121	0.02963	0.02826	0.02701	0.0259	0.0249	0.02397	0.02314	0.02237	0.02163	0.02093	0.02027	0.01967	0.01886	0.01804	0.01745	0.01694	0.01646	0.01601	0.01558	0.01496	0.01457	0.01424	0.01376	0.01344	0.0131	0.01278	0.01257	0.01222	0.01197	0.01178	0.01139	0.01118	0.01105	0.01079	0.01062	0.0105	0.01033	0.0101	0.00996	0.00976	0.00958	0.00937	0.00917	0.00894	0.0087	0.00842	0.0081	0.00776	0.00728	0.00672	0.00612	0.00585	0.00581	0.0058	0.0058	0.00579	0.00577	0.00579	0.00579	0.00579	0.00577	0.00579	0.0058	0.0058	0.00581	0.00585	0.00612	0.00672	0.00729	0.00776	0.00809	0.00842	0.0087	0.00894	0.00917	0.00937	0.00958	0.00975	0.00996	0.0101	0.01033	0.0105	0.01062	0.01079	0.01105	0.01118	0.01139	0.01177	0.01197	0.01222	0.01257	0.01278	0.0131	0.01344	0.01376	0.01423	0.01457	0.01496	0.01558	0.01601	0.01646	0.01694	0.01745	0.01804	0.01886	0.01967	0.02027	0.02092	0.02163	0.02237	0.02313	0.02397	0.02489	0.0259	0.02701	0.02825	0.02963	0.0312	0.033	0.03508];	%0.03753	0.04046	0.04407	0.04789	0.05118	0.05504	0.05951	0.06484	0.07102	0.07831	0.08676	0.09633	0.10705];
Cdp=	[0.12994	0.11625	0.10434	0.09349	0.08369	0.07507	0.06767	0.06124	0.05584	0.05118	0.04726	0.04391	0.03979	0.03606	0.03298	0.03044	0.02826	0.02636	0.02471	0.02325	0.02193	0.02075	0.01967	0.01867	0.01776	0.01693	0.01612	0.01535	0.01462	0.01396	0.01308	0.01223	0.0116	0.01107	0.01055	0.01006	0.00959	0.00897	0.00858	0.00821	0.00775	0.00744	0.00709	0.0068	0.00657	0.00625	0.00601	0.0058	0.00543	0.00521	0.00503	0.00472	0.0045	0.00431	0.00407	0.00383	0.00365	0.00344	0.00325	0.00307	0.0029	0.00273	0.00258	0.00242	0.00227	0.00212	0.00196	0.00177	0.0016	0.0016	0.0016	0.0016	0.0016	0.00159	0.00159	0.00159	0.00161	0.00159	0.0016	0.00159	0.0016	0.0016	0.0016	0.00159	0.0016	0.00177	0.00196	0.00212	0.00227	0.00242	0.00258	0.00273	0.0029	0.00307	0.00325	0.00344	0.00366	0.00383	0.00407	0.00431	0.0045	0.00472	0.00503	0.00521	0.00543	0.00579	0.00601	0.00625	0.00657	0.0068	0.00709	0.00744	0.00775	0.00821	0.00858	0.00897	0.00959	0.01006	0.01055	0.01106	0.0116	0.01223	0.01308	0.01396	0.01463	0.01535	0.01612	0.01693	0.01776	0.01867	0.01966	0.02074	0.02193	0.02324	0.0247	0.02635	0.02824	0.03042];	%0.03297	0.03603	0.03978	0.04375	0.04716	0.05115	0.05575	0.06123	0.06757	0.07501	0.08363	0.09336	0.10425];
Cm=	[0.0144	0.0066	-0.0002	-0.0063	-0.0118	-0.0167	-0.0208	-0.0241	-0.0265	-0.0282	-0.0292	-0.0299	-0.0301	-0.0299	-0.0294	-0.0286	-0.0277	-0.0266	-0.0254	-0.024	-0.0226	-0.021	-0.0192	-0.0173	-0.0153	-0.0134	-0.0119	-0.0107	-0.0096	-0.0086	-0.0074	-0.0063	-0.0054	-0.0048	-0.0041	-0.0036	-0.003	-0.0023	-0.0018	-0.0015	-0.0009	-0.0005	-0.0001	0.0003	0.0006	0.0012	0.0005	-0.0011	-0.0023	-0.0028	-0.0022	-0.0013	-0.0007	-0.0003	0	0.0003	0.0003	0.0004	0.0005	0.0005	0.0005	0.0006	0.0005	0.0005	0.0005	0.0005	0.0005	0.0006	0.0008	0.001	0.0008	0.0006	0.0006	0.0004	0.0002	0.0001	0	-0.0001	-0.0002	-0.0004	-0.0006	-0.0006	-0.0008	-0.0009	-0.0008	-0.0006	-0.0005	-0.0005	-0.0005	-0.0005	-0.0005	-0.0006	-0.0005	-0.0005	-0.0005	-0.0004	-0.0003	-0.0003	0	0.0003	0.0007	0.0013	0.0022	0.0028	0.0023	0.0011	-0.0005	-0.0012	-0.0006	-0.0003	0.0001	0.0005	0.0009	0.0014	0.0018	0.0023	0.003	0.0035	0.0041	0.0047	0.0054	0.0062	0.0074	0.0086	0.0095	0.0106	0.0119	0.0133	0.0152	0.0172	0.0191	0.0209	0.0224	0.0239	0.0253	0.0265	0.0276	0.0285];%	0.0292	0.0297	0.0299	0.0297	0.029	0.028	0.0263	0.0239	0.0206	0.0165	0.0116	0.006	-0.0001];
tdc=.12;

elseif Subsonic_airfoil==2
%sc20406-il - NASA SC(2)-0406 AIRFOIL
Alpha=[	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5];
Cl=[	-0.6124	-0.5852	-0.5576	-0.5299	-0.5019	-0.4737	-0.4448	-0.4154	-0.3863	-0.3575	-0.3286	-0.2997	-0.2713	-0.243	-0.2147	-0.1862	-0.1573	-0.1272	-0.0984	-0.0707	-0.0435	-0.0163	0.0108	0.038	0.081	0.1212	0.1517	0.1764	0.2007	0.2263	0.2521	0.2783	0.3313	0.3586	0.3862	0.4136	0.441	0.4683	0.4951	0.5213	0.5469	0.5707	0.5933	0.6217	0.6492	0.6759	0.7024	0.728	0.7531	0.7781	0.8022	0.8244	0.8232	0.8385	0.8519	0.8622	0.8661];	
Cd=[	0.01742	0.01581	0.01436	0.01314	0.01225	0.01114	0.00996	0.0091	0.0086	0.00827	0.00792	0.00759	0.00738	0.00718	0.00696	0.00667	0.00618	0.00531	0.0048	0.00461	0.00449	0.0044	0.00435	0.00432	0.00419	0.00412	0.00413	0.0042	0.00436	0.00458	0.00496	0.00558	0.00716	0.0076	0.00789	0.00807	0.00824	0.00843	0.00874	0.00912	0.00939	0.00966	0.00992	0.01043	0.01126	0.01242	0.01371	0.01561	0.01782	0.01975	0.02192	0.02478	0.0425	0.04663	0.05062	0.05523	0.06154];	
Cdp=[	0.01305	0.01119	0.00953	0.00816	0.00716	0.00592	0.00457	0.00359	0.00304	0.00268	0.00229	0.00203	0.00186	0.0017	0.00156	0.0014	0.00124	0.00105	0.00098	0.00096	0.00096	0.00099	0.00104	0.0011	0.00103	0.00103	0.00108	0.00112	0.00114	0.00115	0.00119	0.00129	0.00164	0.00181	0.00196	0.00214	0.00231	0.00249	0.00275	0.00324	0.00359	0.00396	0.00431	0.00486	0.0058	0.00708	0.00853	0.01065	0.01315	0.01534	0.01782	0.02104	0.0401	0.04445	0.04861	0.0534	0.05989];	
Cm=[	-0.032	-0.0322	-0.0325	-0.0327	-0.0329	-0.0332	-0.0337	-0.0342	-0.0346	-0.0349	-0.0351	-0.0355	-0.0357	-0.0359	-0.0361	-0.0365	-0.0371	-0.0383	-0.039	-0.039	-0.039	-0.0388	-0.0387	-0.0386	-0.0419	-0.0446	-0.0449	-0.0438	-0.0426	-0.0419	-0.0413	-0.0412	-0.0415	-0.0415	-0.0416	-0.0414	-0.0413	-0.0411	-0.0409	-0.0403	-0.0397	-0.0387	-0.0374	-0.0375	-0.0375	-0.0372	-0.037	-0.0365	-0.0361	-0.0356	-0.0351	-0.0343	-0.0303	-0.0302	-0.0303	-0.0307	-0.0318];	
tdc=.06;

elseif Subsonic_airfoil==3
%sc20412-il - NASA SC(2)-0412 AIRFOIL
Alpha=[	-19.25	-19	-18.75	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25];%	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18];
Cl=[	-1.1059	-1.1447	-1.1776	-1.2045	-1.2275	-1.2454	-1.2602	-1.2715	-1.2798	-1.2851	-1.2908	-1.3034	-1.3115	-1.3155	-1.3157	-1.3132	-1.3084	-1.3016	-1.2932	-1.2836	-1.2729	-1.2613	-1.2495	-1.2372	-1.2241	-1.2107	-1.1993	-1.1932	-1.1776	-1.159	-1.1391	-1.1184	-1.0968	-1.0747	-1.054	-1.032	-1.0089	-0.9854	-0.9617	-0.9381	-0.9141	-0.8899	-0.8655	-0.8409	-0.8116	-0.7777	-0.7436	-0.7092	-0.6747	-0.64	-0.6047	-0.5696	-0.5363	-0.5014	-0.4663	-0.4303	-0.3944	-0.3581	-0.3215	-0.285	-0.2513	-0.2162	-0.1802	-0.1425	-0.1058	-0.0714	-0.0415	-0.0119	0.0171	0.0457	0.0739	0.1024	0.1312	0.1601	0.1889	0.2177	0.2462	0.2749	0.3039	0.3321	0.3598	0.386	0.4116	0.4387	0.466	0.4936	0.5217	0.5499	0.5782	0.6065	0.6347	0.6629	0.6913	0.7193	0.7477	0.7759	0.8037	0.8319	0.8596	0.8869	0.9147	0.9422	0.9689	0.9965	1.0236	1.05	1.0772	1.1039	1.1292	1.1557	1.1817	1.2073	1.2311	1.2567	1.2819	1.3066	1.3306	1.353	1.3747	1.3983	1.421	1.4432	1.4648	1.4857	1.5055	1.5235	1.5376	1.549	1.5655	1.5796	1.591	1.5972	1.6046	1.6113	1.617	1.6216	1.625	1.6274	1.6283];%	1.628	1.6265	1.6238	1.6193	1.6119	1.6026	1.5887	1.5715	1.5493	1.5223	1.4878];
Cd=[	0.1273	0.11529	0.10484	0.09572	0.08756	0.08041	0.07405	0.06847	0.06356	0.05932	0.05523	0.05058	0.04674	0.04359	0.04093	0.03861	0.03658	0.03477	0.03315	0.03166	0.03028	0.029	0.02777	0.0266	0.02553	0.02456	0.02356	0.02209	0.02109	0.02031	0.01962	0.01897	0.01839	0.01788	0.01711	0.01648	0.01604	0.01564	0.01529	0.01476	0.01432	0.01401	0.01372	0.01338	0.01291	0.0126	0.01233	0.01189	0.0116	0.01138	0.01099	0.01073	0.01051	0.01014	0.00991	0.00958	0.00931	0.00904	0.00875	0.00843	0.00809	0.00767	0.00718	0.00656	0.00575	0.00533	0.00526	0.00525	0.00525	0.00528	0.00531	0.00538	0.00543	0.00553	0.00562	0.00569	0.00582	0.00598	0.00607	0.00623	0.00648	0.00701	0.00774	0.00827	0.00875	0.00919	0.00946	0.00969	0.00993	0.01016	0.01038	0.01062	0.01081	0.01107	0.01125	0.01147	0.01175	0.01189	0.0121	0.0124	0.01259	0.01284	0.0132	0.0134	0.01367	0.01406	0.01429	0.01456	0.01503	0.01531	0.01563	0.01601	0.01664	0.01699	0.0174	0.0178	0.01827	0.01893	0.01965	0.02011	0.02064	0.02122	0.0218	0.0224	0.02307	0.0239	0.02503	0.02634	0.02708	0.02788	0.02874	0.02969	0.03079	0.032	0.03333	0.0348	0.03645	0.03829	0.04035];%	0.04264	0.0452	0.04805	0.0513	0.05512	0.05944	0.0648	0.07117	0.07894	0.08823	0.0993];
Cdp=[	0.12472	0.11253	0.10192	0.09265	0.08433	0.07704	0.07053	0.06482	0.05978	0.05542	0.05121	0.04642	0.04245	0.03921	0.03647	0.03408	0.03197	0.0301	0.02841	0.02686	0.02542	0.02408	0.02277	0.02153	0.0204	0.01936	0.0183	0.01677	0.01574	0.01493	0.01421	0.01352	0.01289	0.01234	0.01153	0.01089	0.01044	0.01002	0.00965	0.00909	0.00866	0.00834	0.00803	0.00767	0.00722	0.00691	0.00663	0.0062	0.00593	0.0057	0.00534	0.0051	0.00488	0.00455	0.00432	0.00404	0.00381	0.0036	0.00338	0.00316	0.00296	0.00274	0.00251	0.00224	0.00193	0.00182	0.00182	0.00181	0.00182	0.00181	0.00181	0.0018	0.00183	0.00188	0.0019	0.00194	0.00199	0.00204	0.00207	0.00212	0.0022	0.00231	0.00251	0.00267	0.00283	0.00299	0.0031	0.00324	0.00338	0.00354	0.00371	0.00389	0.00406	0.00428	0.00446	0.00465	0.0049	0.00507	0.00527	0.00557	0.00578	0.00602	0.00637	0.0066	0.00685	0.00723	0.00748	0.00777	0.00824	0.00857	0.00892	0.00931	0.00996	0.01035	0.01079	0.01124	0.01175	0.01245	0.01323	0.01375	0.01435	0.01498	0.01562	0.01629	0.01703	0.01791	0.01911	0.02054	0.02136	0.02228	0.02324	0.0243	0.0255	0.02681	0.02826	0.02985	0.03162	0.03356	0.03574];%	0.03815	0.04084	0.04381	0.04718	0.05113	0.05559	0.0611	0.06763	0.07559	0.08507	0.09635];
Cm=[	-0.0058	-0.0127	-0.0186	-0.0237	-0.0284	-0.0324	-0.0359	-0.0389	-0.0413	-0.0433	-0.0449	-0.0464	-0.0473	-0.0478	-0.0481	-0.0482	-0.0482	-0.0481	-0.0479	-0.0476	-0.0473	-0.047	-0.0465	-0.046	-0.0454	-0.0447	-0.0436	-0.0423	-0.042	-0.0418	-0.0416	-0.0415	-0.0413	-0.0411	-0.0408	-0.0406	-0.0404	-0.0401	-0.0398	-0.0396	-0.0395	-0.0392	-0.039	-0.0388	-0.0397	-0.0414	-0.043	-0.0449	-0.0466	-0.0483	-0.0503	-0.0521	-0.0534	-0.0553	-0.057	-0.059	-0.061	-0.063	-0.0651	-0.0672	-0.0687	-0.0707	-0.0729	-0.0756	-0.0783	-0.0799	-0.0802	-0.0803	-0.0803	-0.0802	-0.0799	-0.0798	-0.0797	-0.0796	-0.0795	-0.0795	-0.0793	-0.0792	-0.0793	-0.0791	-0.0789	-0.0786	-0.0783	-0.0783	-0.0783	-0.0784	-0.0785	-0.0785	-0.0786	-0.0786	-0.0786	-0.0786	-0.0787	-0.0786	-0.0787	-0.0787	-0.0786	-0.0786	-0.0786	-0.0784	-0.0783	-0.0782	-0.0779	-0.0778	-0.0777	-0.0774	-0.0772	-0.077	-0.0765	-0.0762	-0.0759	-0.0755	-0.0748	-0.0743	-0.0739	-0.0733	-0.0727	-0.0718	-0.0708	-0.0701	-0.0693	-0.0684	-0.0675	-0.0664	-0.0652	-0.0638	-0.0618	-0.0595	-0.0579	-0.0559	-0.0535	-0.0503	-0.0476	-0.0451	-0.0426	-0.0403	-0.0382	-0.0363	-0.0346];%	-0.0332	-0.0322	-0.0316	-0.0313	-0.0317	-0.0326	-0.0344	-0.0372	-0.0414	-0.0468	-0.0533];
tdc=.12;

elseif Subsonic_airfoil==4
%sc20606-il - NASA SC(2)-0606 AIRFOIL
Alpha=[	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.75	6	9.25	9.5];
Cl=[	-0.5077	-0.5077	-0.6146	-0.6132	-0.6122	-0.6275	-0.6328	-0.6373	-0.6465	-0.651	-0.6532	-0.6491	-0.6388	-0.6249	-0.6047	-0.5755	-0.5447	-0.5144	-0.484	-0.4552	-0.4262	-0.3975	-0.3688	-0.3398	-0.311	-0.2823	-0.2502	-0.222	-0.1921	-0.1629	-0.1334	-0.1002	-0.0604	-0.0222	0.009	0.0386	0.0674	0.0961	0.1246	0.1527	0.1895	0.2388	0.2849	0.3209	0.3579	0.3812	0.4007	0.4239	0.4767	0.5042	0.5316	0.559	0.5864	0.6137	0.6406	0.6655	0.6923	0.7183	0.7445	0.7704	0.7962	0.8214	0.8462	0.8715	0.8946	0.922	0.9342	0.7425	0.7392];
Cd=[	0.10055	0.09669	0.10655	0.10197	0.09728	0.08892	0.08337	0.07817	0.07251	0.0624	0.05559	0.05018	0.04607	0.04172	0.0303	0.02292	0.01862	0.01577	0.01348	0.01234	0.01142	0.01075	0.01019	0.00969	0.00928	0.00893	0.00828	0.00809	0.00778	0.00757	0.00737	0.00693	0.00612	0.00548	0.00532	0.00525	0.00523	0.00522	0.00522	0.00525	0.00515	0.00481	0.00454	0.00442	0.00436	0.00478	0.00613	0.00732	0.00844	0.00872	0.00897	0.00933	0.00961	0.0099	0.01037	0.01142	0.01194	0.01255	0.01325	0.01406	0.01499	0.01638	0.01779	0.01852	0.02036	0.04687	0.05102	0.11824	0.12273];
Cdp=[	0.09893	0.09507	0.10484	0.10027	0.09559	0.08728	0.08174	0.07657	0.07095	0.06079	0.05385	0.04829	0.04403	0.03949	0.02744	0.01928	0.01446	0.01122	0.00865	0.00741	0.00639	0.00565	0.00502	0.00446	0.00399	0.0036	0.00286	0.00264	0.00235	0.00219	0.00208	0.00193	0.00176	0.00167	0.00168	0.00171	0.00175	0.00181	0.00189	0.00198	0.00192	0.00165	0.00145	0.00138	0.00131	0.00131	0.00154	0.00179	0.00219	0.00238	0.00259	0.00295	0.00328	0.00364	0.00416	0.00537	0.00596	0.00666	0.00747	0.00839	0.00944	0.01102	0.01264	0.01349	0.0156	0.04441	0.04878	0.11706	0.12155];
Cm=[	-0.0118	-0.0127	-0.002	-0.0041	-0.0063	-0.0105	-0.0137	-0.0169	-0.0206	-0.0334	-0.0381	-0.041	-0.0429	-0.0447	-0.0483	-0.0521	-0.0546	-0.0561	-0.0573	-0.058	-0.0586	-0.059	-0.0594	-0.0599	-0.0603	-0.0607	-0.0618	-0.062	-0.0626	-0.0631	-0.0637	-0.0653	-0.0687	-0.0716	-0.0725	-0.073	-0.0733	-0.0735	-0.0736	-0.0737	-0.0757	-0.0804	-0.0843	-0.0859	-0.0877	-0.0861	-0.0844	-0.084	-0.0837	-0.0837	-0.0835	-0.0833	-0.0831	-0.0829	-0.0826	-0.0818	-0.0815	-0.081	-0.0805	-0.0799	-0.0793	-0.0785	-0.0777	-0.0773	-0.0763	-0.0651	-0.0638	-0.0708	-0.0722];
tdc=.06;

elseif Subsonic_airfoil==5
%sc20612-il - NASA SC(2)-0612 AIRFOIL
Alpha=[	-19	-18.75	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5];%	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18];
Cl=[	-1.0935	-1.1235	-1.1475	-1.166	-1.1808	-1.1921	-1.2008	-1.207	-1.2112	-1.2136	-1.2146	-1.2142	-1.2123	-1.209	-1.2042	-1.1982	-1.1909	-1.1825	-1.1733	-1.1691	-1.1716	-1.1742	-1.1808	-1.1762	-1.1673	-1.1547	-1.1392	-1.1213	-1.1014	-1.0727	-1.0406	-1.0096	-0.9777	-0.9453	-0.9123	-0.879	-0.8455	-0.8117	-0.7802	-0.7481	-0.7149	-0.6814	-0.6471	-0.6129	-0.5782	-0.5434	-0.5087	-0.4729	-0.4375	-0.4019	-0.366	-0.3343	-0.3009	-0.2662	-0.2311	-0.196	-0.1604	-0.1235	-0.0859	-0.0474	-0.0085	0.0286	0.0606	0.0918	0.1219	0.1503	0.1784	0.2068	0.2352	0.2637	0.2921	0.3208	0.3493	0.3776	0.4061	0.4341	0.4615	0.4864	0.509	0.5334	0.5588	0.5852	0.6119	0.6392	0.6669	0.6947	0.7228	0.7505	0.7786	0.8063	0.8339	0.8618	0.8893	0.9165	0.9443	0.9714	0.9984	1.0257	1.0519	1.0789	1.1052	1.1312	1.1574	1.1832	1.2078	1.2335	1.2588	1.2836	1.3072	1.33	1.3542	1.3779	1.4012	1.424	1.4461	1.4664	1.4825	1.5028	1.5231	1.5425	1.5608	1.578	1.5939	1.6088	1.6217	1.63	1.6393	1.6471	1.6524	1.6538	1.648	1.6483	1.6528	1.6564	1.6581];%	1.6577	1.6559	1.6524	1.6464	1.6388	1.6287	1.6154	1.5994	1.5797	1.5561	1.5257	1.4878	1.4409	1.3801];
Cd=[	0.10266	0.09302	0.08463	0.07748	0.0711	0.06555	0.06065	0.05633	0.05251	0.04908	0.04603	0.04331	0.04084	0.03862	0.03659	0.03476	0.03311	0.03164	0.03031	0.0287	0.02691	0.02561	0.0246	0.0236	0.02269	0.02185	0.02105	0.0203	0.01961	0.01898	0.01843	0.01727	0.01661	0.01608	0.01559	0.01516	0.0145	0.01404	0.01367	0.01332	0.01285	0.01256	0.01217	0.01191	0.01159	0.01132	0.01115	0.01076	0.01053	0.01028	0.00999	0.0098	0.00954	0.00929	0.00903	0.00877	0.00845	0.00802	0.00752	0.00695	0.00636	0.00575	0.00557	0.0055	0.00547	0.00544	0.0054	0.0054	0.00541	0.00545	0.00547	0.00556	0.00564	0.00574	0.00584	0.00599	0.00624	0.0067	0.00756	0.0083	0.0089	0.00938	0.00981	0.01017	0.01047	0.01075	0.0109	0.01114	0.0113	0.01154	0.01177	0.01195	0.01219	0.01246	0.01265	0.01291	0.01316	0.01334	0.01368	0.01389	0.0142	0.01456	0.01485	0.01521	0.01571	0.01601	0.01635	0.01674	0.01727	0.01789	0.0183	0.01876	0.01925	0.01974	0.02028	0.02098	0.02211	0.02274	0.02332	0.02397	0.02467	0.02538	0.02611	0.02681	0.02756	0.02837	0.02933	0.03044	0.03177	0.03345	0.03581	0.03775	0.03934	0.04109	0.04309];%	0.04535	0.04789	0.05076	0.05406	0.05779	0.06213	0.06723	0.07326	0.0805	0.08898	0.09934	0.11156	0.12597	0.14414];
Cdp=[	0.09984	0.09005	0.08151	0.07422	0.0677	0.06203	0.05701	0.05257	0.04864	0.04511	0.04195	0.03914	0.03658	0.03425	0.03213	0.03021	0.02848	0.02693	0.02553	0.02384	0.02196	0.02062	0.01958	0.01854	0.01759	0.01671	0.01587	0.01506	0.01432	0.01364	0.01303	0.01183	0.01115	0.01059	0.01007	0.00961	0.00892	0.00847	0.00808	0.00771	0.00727	0.00698	0.00661	0.00636	0.00605	0.0058	0.00563	0.00528	0.00505	0.0048	0.00455	0.00436	0.00413	0.00392	0.00371	0.00351	0.0033	0.00308	0.00284	0.0026	0.00235	0.00215	0.00211	0.00207	0.00205	0.002	0.00197	0.00192	0.00194	0.00192	0.00194	0.002	0.00203	0.00208	0.00211	0.00215	0.0022	0.00228	0.00253	0.00277	0.00298	0.00318	0.00337	0.00355	0.00373	0.00391	0.00405	0.00425	0.00443	0.00463	0.00485	0.00504	0.00526	0.00552	0.00571	0.00594	0.0062	0.0064	0.00672	0.00695	0.00724	0.00761	0.00791	0.00826	0.00877	0.00912	0.0095	0.00991	0.01046	0.01113	0.01159	0.01209	0.01262	0.01315	0.01374	0.01449	0.0157	0.0164	0.01706	0.01778	0.01855	0.01935	0.02016	0.02094	0.02175	0.02263	0.02365	0.02483	0.02626	0.02806	0.03059	0.03266	0.03437	0.03623	0.03835];%	0.04076	0.04344	0.04644	0.04988	0.05375	0.05823	0.0635	0.06971	0.07713	0.0858	0.09636	0.1088	0.12342	0.14184];
Cm=[	-0.0343	-0.0398	-0.0446	-0.0488	-0.0525	-0.0556	-0.0582	-0.0603	-0.062	-0.0634	-0.0644	-0.0651	-0.0656	-0.0658	-0.0659	-0.0659	-0.0657	-0.0653	-0.0648	-0.0637	-0.0617	-0.0589	-0.0549	-0.0528	-0.0513	-0.0503	-0.0497	-0.0494	-0.0493	-0.0507	-0.0526	-0.055	-0.0569	-0.0587	-0.0606	-0.0624	-0.0645	-0.0664	-0.0678	-0.0691	-0.0709	-0.0725	-0.0743	-0.076	-0.0779	-0.0797	-0.0814	-0.0836	-0.0854	-0.0874	-0.0895	-0.0904	-0.0919	-0.0936	-0.0954	-0.0972	-0.0991	-0.1015	-0.1042	-0.1071	-0.1102	-0.1127	-0.1136	-0.1141	-0.1144	-0.1142	-0.1139	-0.1136	-0.1134	-0.1132	-0.113	-0.1129	-0.1127	-0.1125	-0.1124	-0.1121	-0.1118	-0.1111	-0.1101	-0.1096	-0.1092	-0.109	-0.1088	-0.1087	-0.1086	-0.1085	-0.1085	-0.1084	-0.1083	-0.1082	-0.108	-0.1079	-0.1078	-0.1076	-0.1074	-0.1072	-0.107	-0.1068	-0.1064	-0.1061	-0.1058	-0.1053	-0.1049	-0.1045	-0.1038	-0.1033	-0.1028	-0.1022	-0.1013	-0.1004	-0.0996	-0.0988	-0.098	-0.0971	-0.0961	-0.0947	-0.0928	-0.0915	-0.0902	-0.0887	-0.0871	-0.0854	-0.0834	-0.0813	-0.0789	-0.0758	-0.0729	-0.0701	-0.0671	-0.0638	-0.0601	-0.0573	-0.0551	-0.053	-0.0511];%	-0.0494	-0.0479	-0.0467	-0.0459	-0.0457	-0.046	-0.0472	-0.0494	-0.0528	-0.0575	-0.0636	-0.0711	-0.0801	-0.092];
tdc=.12;

elseif Subsonic_airfoil==6
%sc20706-il - NASA SC(2)-0706 AIRFOIL
Alpha=[	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	11	11.25];
Cl=[	-0.503	-0.509	-0.515	-0.5213	-0.5295	-0.5405	-0.557	-0.5861	-0.6005	-0.6085	-0.6076	-0.5968	-0.5903	-0.5512	-0.5205	-0.4531	-0.4158	-0.3835	-0.3527	-0.3208	-0.2891	-0.2582	-0.2212	-0.191	-0.1591	-0.1293	-0.0971	-0.0601	-0.0167	0.0249	0.0579	0.0877	0.117	0.146	0.1742	0.2023	0.2479	0.2996	0.348	0.3901	0.4307	0.4462	0.4683	0.4926	0.5188	0.546	0.5735	0.6004	0.628	0.6551	0.6799	0.7068	0.733	0.7591	0.7849	0.8111	0.8368	0.8621	0.887	0.9091	0.9312	0.9519	0.97	0.9387	0.9432];
Cd=[	0.08858	0.0837	0.07897	0.07438	0.06954	0.06454	0.05919	0.05251	0.04536	0.03857	0.03224	0.02703	0.0304	0.02173	0.01955	0.01457	0.01215	0.0113	0.01083	0.01032	0.0099	0.00958	0.00888	0.00868	0.0084	0.00826	0.00799	0.00749	0.00673	0.00614	0.00597	0.00594	0.00592	0.00593	0.00597	0.00601	0.00579	0.00535	0.00492	0.00465	0.00446	0.00618	0.00731	0.00822	0.00873	0.00905	0.00926	0.0097	0.00989	0.01019	0.01124	0.01155	0.01209	0.01272	0.01339	0.01409	0.01484	0.01597	0.01729	0.01996	0.02249	0.02528	0.02872	0.17193	0.17649];
Cdp=[	0.087	0.08213	0.07741	0.07285	0.06802	0.06305	0.05772	0.05109	0.04384	0.0369	0.03036	0.02493	0.02759	0.01808	0.01557	0.00988	0.0072	0.00627	0.00574	0.00518	0.00469	0.00434	0.00352	0.00332	0.00301	0.00287	0.0027	0.00255	0.0024	0.00232	0.00235	0.00239	0.00245	0.00254	0.00261	0.00271	0.00254	0.00217	0.0018	0.00157	0.00139	0.00158	0.00184	0.00208	0.0023	0.00252	0.00277	0.00324	0.00346	0.00379	0.00499	0.00535	0.00597	0.00667	0.00744	0.00824	0.00909	0.01035	0.01183	0.01483	0.01771	0.02089	0.02475	0.17071	0.17526];
Cm=[	-0.0175	-0.0184	-0.0193	-0.0201	-0.0211	-0.022	-0.0231	-0.0258	-0.0328	-0.0365	-0.0397	-0.0424	-0.0511	-0.0572	-0.0587	-0.063	-0.0658	-0.0671	-0.068	-0.0692	-0.0703	-0.0711	-0.0734	-0.0741	-0.0752	-0.0757	-0.0769	-0.0795	-0.0837	-0.0874	-0.0887	-0.0892	-0.0895	-0.0898	-0.0899	-0.09	-0.0939	-0.0991	-0.1035	-0.1064	-0.109	-0.1059	-0.1051	-0.1047	-0.1044	-0.1043	-0.1041	-0.1038	-0.1036	-0.1034	-0.1025	-0.1022	-0.1017	-0.1012	-0.1006	-0.1001	-0.0995	-0.0988	-0.098	-0.0966	-0.0952	-0.0936	-0.0914	-0.1224	-0.1245];
tdc=.06;

elseif Subsonic_airfoil==7
%sc20712-il - NASA SC(2)-0712 AIRFOIL
Alpha=[	-19.5	-19.25	-19	-18.75	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75];%	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18];
Cl=[	-1.065	-1.0958	-1.1211	-1.1417	-1.158	-1.1709	-1.18	-1.186	-1.19	-1.1914	-1.1917	-1.191	-1.1894	-1.1874	-1.1845	-1.1811	-1.1771	-1.1732	-1.1692	-1.166	-1.164	-1.165	-1.1731	-1.1796	-1.1806	-1.171	-1.1419	-1.1115	-1.0804	-1.0488	-1.0167	-0.9842	-0.9517	-0.9183	-0.8881	-0.8567	-0.8244	-0.7913	-0.7582	-0.7247	-0.6907	-0.6558	-0.621	-0.5863	-0.551	-0.5154	-0.4801	-0.4443	-0.4144	-0.3819	-0.3469	-0.3124	-0.2766	-0.2411	-0.205	-0.1691	-0.1332	-0.0964	-0.0591	-0.0207	0.0189	0.0607	0.0939	0.1251	0.1568	0.1884	0.2161	0.2456	0.2735	0.3028	0.3312	0.3594	0.388	0.4169	0.4457	0.4741	0.5017	0.5277	0.5494	0.571	0.5944	0.6197	0.6458	0.6726	0.6998	0.7274	0.7551	0.7828	0.8102	0.8378	0.8651	0.8923	0.9194	0.9466	0.9732	1.0005	1.0273	1.053	1.0799	1.1063	1.1322	1.1567	1.1829	1.2087	1.2341	1.2587	1.2815	1.3066	1.3313	1.3557	1.3796	1.401	1.4225	1.4457	1.468	1.4899	1.5112	1.5321	1.5516	1.5677	1.5806	1.598	1.6138	1.6277	1.6376	1.6483	1.6592	1.6701	1.6799	1.69	1.6984	1.7058	1.7105	1.7122];%	1.7093	1.7022	1.6896	1.6894	1.6859	1.6829	1.6762	1.6671	1.6563	1.6426	1.6251	1.6035	1.5782	1.5461	1.5051	1.4568	1.3922];
Cd=[	0.10551	0.09582	0.08727	0.0798	0.07318	0.06736	0.06233	0.05795	0.05408	0.05071	0.04764	0.04487	0.04236	0.04004	0.03791	0.03596	0.03421	0.03262	0.03118	0.02989	0.02875	0.02775	0.0269	0.02581	0.02423	0.02307	0.02208	0.02125	0.02048	0.01976	0.01913	0.01861	0.01742	0.0169	0.01642	0.01596	0.01559	0.01476	0.01432	0.01394	0.01361	0.01305	0.01269	0.01241	0.01205	0.01169	0.01144	0.01112	0.01083	0.01063	0.01027	0.01006	0.00974	0.00952	0.00925	0.00897	0.00869	0.00833	0.00792	0.00743	0.00686	0.00614	0.00585	0.00577	0.0057	0.00563	0.00557	0.00551	0.0055	0.00551	0.00554	0.00554	0.0056	0.00567	0.0057	0.00577	0.0059	0.00619	0.00696	0.00792	0.00868	0.00925	0.00972	0.01009	0.01041	0.01066	0.01091	0.01117	0.01141	0.0116	0.01186	0.01211	0.01236	0.01261	0.01294	0.01316	0.01344	0.01384	0.01405	0.01431	0.01464	0.01515	0.01542	0.01573	0.01607	0.01651	0.01712	0.01743	0.01778	0.01814	0.01855	0.01923	0.0199	0.02029	0.02076	0.02125	0.02176	0.02229	0.02291	0.02384	0.02497	0.02556	0.02622	0.02695	0.02772	0.02858	0.02948	0.03039	0.03138	0.03236	0.03348	0.03472	0.03621	0.03799];%	0.04024	0.04298	0.04645	0.04879	0.05161	0.05454	0.05807	0.06209	0.06673	0.0722	0.07876	0.08657	0.09567	0.10656	0.11955	0.13457	0.15415];
Cdp=[	0.1027	0.09285	0.08416	0.07656	0.06981	0.06386	0.05873	0.05425	0.05028	0.04683	0.04368	0.04082	0.03822	0.03582	0.03361	0.03158	0.02974	0.02808	0.02657	0.0252	0.024	0.02295	0.02205	0.02089	0.01924	0.01805	0.01702	0.01616	0.01534	0.01457	0.01387	0.0133	0.01209	0.01155	0.01104	0.01055	0.01014	0.00929	0.00885	0.00845	0.00809	0.00753	0.00719	0.0069	0.00653	0.00619	0.00594	0.00563	0.00537	0.00516	0.00484	0.00463	0.00438	0.00417	0.00395	0.00374	0.00354	0.00333	0.00312	0.0029	0.00266	0.00241	0.00237	0.00234	0.00228	0.00222	0.00216	0.00209	0.00207	0.00208	0.00205	0.00207	0.0021	0.00211	0.00212	0.00216	0.00218	0.00222	0.0024	0.0027	0.00296	0.00318	0.00338	0.00357	0.00374	0.00391	0.00408	0.00427	0.00446	0.00465	0.00487	0.00509	0.00534	0.00557	0.00588	0.00611	0.00637	0.00676	0.00701	0.00729	0.00762	0.00814	0.00844	0.00878	0.00912	0.00956	0.01022	0.01058	0.01097	0.01136	0.01178	0.01248	0.0132	0.01366	0.01419	0.01474	0.0153	0.01587	0.01652	0.01749	0.01873	0.0194	0.02015	0.02097	0.02183	0.02278	0.02375	0.02475	0.02583	0.02688	0.02808	0.0294	0.03097	0.03285];%	0.03523	0.03812	0.04177	0.04424	0.04719	0.05025	0.05392	0.0581	0.0629	0.06853	0.07527	0.08326	0.09255	0.10364	0.11685	0.13209	0.15192];
Cm=[	-0.0395	-0.045	-0.0499	-0.0542	-0.058	-0.0613	-0.0641	-0.0664	-0.0683	-0.0699	-0.0711	-0.072	-0.0726	-0.0729	-0.073	-0.0728	-0.0723	-0.0715	-0.0703	-0.0687	-0.0666	-0.0637	-0.0594	-0.0556	-0.0535	-0.0526	-0.0549	-0.0572	-0.0594	-0.0617	-0.0638	-0.0658	-0.0688	-0.0709	-0.0723	-0.0738	-0.0753	-0.0776	-0.0795	-0.0813	-0.0831	-0.0853	-0.0873	-0.0892	-0.0913	-0.0934	-0.0954	-0.0975	-0.0982	-0.0995	-0.1014	-0.1031	-0.1051	-0.107	-0.1091	-0.1111	-0.1131	-0.1154	-0.118	-0.1208	-0.1241	-0.128	-0.1294	-0.13	-0.1306	-0.1312	-0.1308	-0.1308	-0.1305	-0.1304	-0.1302	-0.13	-0.1298	-0.1297	-0.1297	-0.1295	-0.1291	-0.1284	-0.127	-0.1259	-0.1251	-0.1247	-0.1244	-0.1242	-0.124	-0.1239	-0.1238	-0.1236	-0.1235	-0.1233	-0.1231	-0.1228	-0.1226	-0.1223	-0.122	-0.1218	-0.1215	-0.121	-0.1206	-0.1203	-0.1198	-0.119	-0.1186	-0.1181	-0.1176	-0.1169	-0.1159	-0.1152	-0.1146	-0.1138	-0.113	-0.1118	-0.1106	-0.1097	-0.1087	-0.1075	-0.1063	-0.105	-0.1036	-0.1016	-0.0991	-0.0972	-0.0951	-0.0928	-0.0897	-0.0869	-0.0842	-0.0817	-0.0791	-0.0767	-0.0742	-0.0717	-0.0691	-0.0663];%	-0.0634	-0.0605	-0.0578	-0.0562	-0.0547	-0.0538	-0.0531	-0.053	-0.0536	-0.0551	-0.0578	-0.0616	-0.0667	-0.0733	-0.0813	-0.0911	-0.1045];
tdc=.12;

elseif Subsonic_airfoil==8
%NACA 0012
Alpha=[	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5];%	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5];
Cl=[	-1.2258	-1.2456	-1.2659	-1.2852	-1.3031	-1.3193	-1.3322	-1.3427	-1.3519	-1.3687	-1.3806	-1.3868	-1.3879	-1.3854	-1.3802	-1.3739	-1.3669	-1.359	-1.3502	-1.3399	-1.3281	-1.3149	-1.3132	-1.3004	-1.2834	-1.2649	-1.2455	-1.225	-1.2032	-1.1872	-1.1666	-1.1449	-1.1228	-1.1	-1.0807	-1.0591	-1.0368	-1.0143	-0.9948	-0.9735	-0.9517	-0.9313	-0.9104	-0.8888	-0.8687	-0.8475	-0.8268	-0.7975	-0.7634	-0.7289	-0.6939	-0.6601	-0.6268	-0.5571	-0.5213	-0.4903	-0.4588	-0.4276	-0.4004	-0.373	-0.3462	-0.3201	-0.2938	-0.2675	-0.241	-0.2144	-0.1878	-0.1611	-0.1343	-0.1075	-0.0806	-0.0538	-0.0268	0	0.0269	0.0538	0.0806	0.1075	0.1344	0.1611	0.1878	0.2144	0.241	0.2675	0.2938	0.3201	0.3462	0.373	0.4004	0.4276	0.4588	0.4903	0.5213	0.5572	0.6268	0.6601	0.694	0.729	0.7635	0.7976	0.8267	0.8474	0.8686	0.8887	0.9103	0.9312	0.9516	0.9734	0.9948	1.0143	1.0368	1.0591	1.0808	1.1001	1.1229	1.145	1.1668	1.1874	1.2033	1.2251	1.2457	1.2651	1.2837	1.3008	1.3137	1.3157	1.329	1.3406	1.3509	1.36	1.3679	1.375	1.3814	1.3864	1.3892];%	1.3881	1.3818	1.3702	1.3536	1.3451	1.3352	1.3218	1.3059	1.288	1.2685	1.2485	1.2284];
Cd=[	0.10236	0.09505	0.08782	0.08088	0.07429	0.06814	0.06256	0.05745	0.05263	0.04704	0.04234	0.03865	0.03577	0.03348	0.0316	0.02997	0.02849	0.02718	0.02602	0.02502	0.02416	0.02343	0.02207	0.02117	0.02049	0.01988	0.01931	0.01881	0.01839	0.01745	0.0169	0.01645	0.01602	0.01566	0.01499	0.01456	0.01423	0.01395	0.01341	0.01307	0.01279	0.0124	0.01209	0.01187	0.01149	0.01124	0.01095	0.01069	0.01034	0.01003	0.00973	0.00942	0.00911	0.00847	0.00816	0.00786	0.00756	0.00728	0.00705	0.00681	0.00659	0.0064	0.00623	0.00606	0.00593	0.00581	0.0057	0.00562	0.00556	0.00549	0.00546	0.00542	0.00541	0.0054	0.00541	0.00542	0.00546	0.00549	0.00555	0.00562	0.0057	0.00581	0.00593	0.00606	0.00623	0.0064	0.00659	0.00681	0.00705	0.00728	0.00756	0.00786	0.00816	0.00847	0.00911	0.00942	0.00972	0.01003	0.01034	0.01069	0.01095	0.01124	0.01149	0.01186	0.01208	0.0124	0.01279	0.01307	0.01341	0.01395	0.01423	0.01456	0.01499	0.01566	0.01602	0.01645	0.0169	0.01745	0.0184	0.01881	0.01931	0.01988	0.02049	0.02117	0.02207	0.02342	0.02415	0.02502	0.02602	0.02717	0.02848	0.02995	0.03158	0.03348	0.03575]; %	0.03863	0.04235	0.04702	0.05261	0.05734	0.06237	0.06802	0.07416	0.08075	0.08773	0.09493	0.10229];
Cdp=[	0.09929	0.09187	0.08451	0.07744	0.07074	0.06446	0.05874	0.05351	0.04856	0.04283	0.038	0.0342	0.03122	0.02885	0.0269	0.02518	0.02363	0.02223	0.02099	0.01991	0.01896	0.01816	0.0167	0.01576	0.01504	0.01438	0.01376	0.01321	0.01274	0.01175	0.01118	0.01069	0.01023	0.00982	0.00912	0.00868	0.00833	0.00801	0.00747	0.00712	0.00682	0.00642	0.00612	0.00587	0.00552	0.00527	0.005	0.00475	0.00446	0.00419	0.00395	0.00371	0.00348	0.00304	0.00284	0.00266	0.00248	0.00232	0.00217	0.00204	0.00191	0.0018	0.0017	0.0016	0.00151	0.00144	0.00137	0.00131	0.00126	0.00122	0.00118	0.00116	0.00115	0.00114	0.00115	0.00116	0.00118	0.00122	0.00126	0.00131	0.00137	0.00144	0.00151	0.0016	0.0017	0.0018	0.00191	0.00204	0.00218	0.00232	0.00248	0.00265	0.00284	0.00304	0.00348	0.00371	0.00395	0.00419	0.00446	0.00475	0.00499	0.00527	0.00552	0.00587	0.00612	0.00642	0.00682	0.00712	0.00747	0.00801	0.00832	0.00868	0.00912	0.00982	0.01023	0.01069	0.01118	0.01175	0.01274	0.01321	0.01376	0.01438	0.01504	0.01576	0.0167	0.01815	0.01895	0.01991	0.02099	0.02222	0.02361	0.02516	0.02687	0.02885	0.03121];%	0.03418	0.038	0.0428	0.04854	0.0534	0.05856	0.06434	0.0706	0.07731	0.08442	0.09174	0.09922];
Cm=[	-0.0012	-0.0049	-0.0086	-0.0121	-0.0154	-0.0185	-0.0213	-0.0238	-0.0261	-0.0285	-0.0303	-0.0313	-0.0317	-0.0317	-0.0313	-0.0305	-0.0295	-0.0283	-0.0268	-0.0252	-0.0236	-0.0219	-0.0187	-0.0169	-0.0156	-0.0144	-0.0134	-0.0124	-0.0116	-0.0101	-0.0091	-0.0082	-0.0074	-0.0066	-0.0053	-0.0043	-0.0034	-0.0024	-0.001	0.0002	0.0014	0.0027	0.004	0.0052	0.0067	0.008	0.0093	0.0088	0.0073	0.0058	0.0041	0.0026	0.0014	-0.0019	-0.0038	-0.0045	-0.0053	-0.0061	-0.0058	-0.0057	-0.0053	-0.0048	-0.0043	-0.0038	-0.0034	-0.003	-0.0025	-0.0021	-0.0018	-0.0014	-0.001	-0.0007	-0.0004	0	0.0003	0.0007	0.001	0.0014	0.0018	0.0021	0.0025	0.003	0.0034	0.0038	0.0043	0.0048	0.0053	0.0057	0.0058	0.0061	0.0053	0.0045	0.0038	0.0019	-0.0014	-0.0027	-0.0041	-0.0058	-0.0074	-0.0089	-0.0093	-0.008	-0.0067	-0.0052	-0.004	-0.0027	-0.0013	-0.0002	0.001	0.0024	0.0034	0.0043	0.0053	0.0066	0.0074	0.0082	0.0091	0.01	0.0116	0.0124	0.0133	0.0144	0.0155	0.0168	0.0186	0.0218	0.0235	0.0251	0.0267	0.0281	0.0294	0.0304	0.0311	0.0315	0.0315];%	0.0311	0.03	0.0282	0.0258	0.0235	0.0211	0.0182	0.0152	0.0118	0.0083	0.0046	0.0008];
tdc=.12;

elseif Subsonic_airfoil==9
%NACA 2412
Alpha=[	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5];%	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19	19.25];
Cl=[	-1.0794	-1.1701	-1.2478	-1.296	-1.3192	-1.3315	-1.3379	-1.3401	-1.3391	-1.3358	-1.3307	-1.324	-1.3124	-1.2985	-1.2832	-1.2677	-1.2509	-1.2336	-1.2161	-1.1987	-1.1812	-1.1638	-1.146	-1.128	-1.101	-1.0719	-1.0424	-1.0128	-0.9824	-0.9537	-0.9244	-0.8943	-0.8635	-0.8332	-0.8026	-0.7755	-0.7466	-0.7167	-0.688	-0.6607	-0.632	-0.6058	-0.5787	-0.5522	-0.5252	-0.4983	-0.4712	-0.4443	-0.4173	-0.3901	-0.363	-0.3358	-0.3084	-0.2539	-0.2265	-0.1988	-0.1711	-0.1435	-0.1158	-0.0881	-0.0604	-0.0327	-0.0053	0.0221	0.0495	0.0772	0.1049	0.1326	0.1602	0.1878	0.2154	0.2429	0.2703	0.2973	0.3241	0.3514	0.3787	0.406	0.4333	0.4603	0.4868	0.5126	0.5377	0.563	0.5886	0.6162	0.6472	0.6812	0.7166	0.7495	0.7725	0.796	0.8197	0.8432	0.8665	0.8905	0.9149	0.9395	0.9643	0.9886	1.0133	1.0375	1.0619	1.0858	1.1102	1.1341	1.1573	1.1807	1.2042	1.2271	1.2494	1.2709	1.2926	1.3144	1.3354	1.3557	1.3747	1.3909	1.4084	1.4252	1.4412	1.4565	1.4704	1.485	1.4989	1.5117	1.5234	1.5346	1.5453	1.555	1.5633	1.5702	1.5777	1.584	1.5887	1.5922	1.5941	1.5943];%	1.5942	1.5938	1.5917	1.5884	1.5834	1.5767	1.5682	1.5581	1.5463	1.5324	1.5173];
Cd=[	0.09958	0.0808	0.06422	0.05172	0.04372	0.03825	0.03439	0.03162	0.02955	0.02794	0.02666	0.0256	0.02464	0.02376	0.02296	0.02214	0.02143	0.0208	0.02021	0.01967	0.01913	0.01861	0.01815	0.01772	0.01725	0.01681	0.01636	0.01585	0.01543	0.01504	0.01467	0.01433	0.014	0.01355	0.01312	0.01279	0.01247	0.01217	0.01189	0.01163	0.01132	0.01098	0.0107	0.01045	0.01023	0.01002	0.00983	0.00962	0.00941	0.00923	0.00908	0.0089	0.00874	0.00845	0.00832	0.0082	0.00811	0.008	0.00794	0.00786	0.00777	0.00768	0.00754	0.00742	0.00731	0.00726	0.00724	0.00722	0.00719	0.00718	0.00716	0.0071	0.00705	0.00693	0.00681	0.00677	0.00677	0.00678	0.00675	0.00673	0.00671	0.00673	0.00681	0.00689	0.00705	0.00726	0.00757	0.00799	0.00832	0.00861	0.00889	0.00916	0.00944	0.00976	0.01014	0.01047	0.01078	0.01106	0.01134	0.01166	0.01195	0.01228	0.01258	0.01293	0.01322	0.01354	0.01391	0.01426	0.01458	0.01494	0.01533	0.01577	0.01617	0.01654	0.01694	0.01738	0.01786	0.0184	0.01884	0.01933	0.01987	0.02047	0.02117	0.02184	0.02257	0.02339	0.02432	0.02532	0.02638	0.02757	0.02892	0.03044	0.03198	0.03369	0.0356	0.03774	0.0401	0.04273];%	0.0455	0.0484	0.05157	0.05503	0.05882	0.06291	0.06736	0.07213	0.07724	0.08276	0.08857];
Cdp=[	0.0969	0.07785	0.06099	0.04823	0.04003	0.03439	0.03039	0.0275	0.02534	0.02364	0.02228	0.02114	0.0201	0.01913	0.01825	0.01738	0.01663	0.01595	0.01532	0.01471	0.01411	0.01356	0.01308	0.01262	0.0121	0.01159	0.01109	0.01056	0.01012	0.0097	0.00928	0.0089	0.00853	0.00804	0.00759	0.00724	0.0069	0.00656	0.00625	0.00596	0.00561	0.00525	0.00494	0.00466	0.0044	0.00415	0.00391	0.00366	0.00344	0.00323	0.00303	0.00283	0.00266	0.00234	0.0022	0.00207	0.00196	0.00184	0.00175	0.00165	0.00156	0.00148	0.00141	0.00134	0.00129	0.00126	0.00123	0.0012	0.00119	0.00118	0.00118	0.00119	0.0012	0.00123	0.00127	0.00131	0.00136	0.00141	0.00147	0.00154	0.00162	0.00171	0.00182	0.00193	0.00205	0.00218	0.00234	0.00255	0.00275	0.00294	0.00311	0.0033	0.0035	0.00372	0.00398	0.00424	0.00449	0.00474	0.00499	0.00527	0.00553	0.00584	0.00613	0.00645	0.00675	0.00708	0.00745	0.0078	0.00816	0.00853	0.00892	0.00937	0.0098	0.01021	0.01064	0.0111	0.0116	0.01216	0.01265	0.01318	0.01376	0.01438	0.0151	0.01583	0.0166	0.01747	0.01842	0.01948	0.0206	0.02185	0.02324	0.02481	0.02643	0.02821	0.0302	0.03241	0.03484	0.03755];%	0.04041	0.04341	0.04668	0.05024	0.05414	0.05833	0.06288	0.06776	0.07299	0.07862	0.08454];
Cm=[	-0.0264	-0.0372	-0.0472	-0.0561	-0.0624	-0.0666	-0.0688	-0.0696	-0.0692	-0.0681	-0.0664	-0.0644	-0.0629	-0.0616	-0.0604	-0.0591	-0.0579	-0.0566	-0.0552	-0.0538	-0.0523	-0.0507	-0.0492	-0.0476	-0.0479	-0.0486	-0.0494	-0.0502	-0.0511	-0.0516	-0.0522	-0.0529	-0.0537	-0.0546	-0.0554	-0.0554	-0.0558	-0.0563	-0.0566	-0.0566	-0.0568	-0.0564	-0.0563	-0.056	-0.0557	-0.0555	-0.0552	-0.055	-0.0547	-0.0545	-0.0543	-0.0541	-0.0539	-0.0536	-0.0534	-0.0533	-0.0532	-0.0531	-0.053	-0.0529	-0.0528	-0.0527	-0.0526	-0.0525	-0.0524	-0.0523	-0.0522	-0.0522	-0.0521	-0.052	-0.0519	-0.0518	-0.0518	-0.0516	-0.0514	-0.0513	-0.0512	-0.051	-0.0508	-0.0506	-0.0502	-0.0496	-0.0489	-0.0482	-0.0476	-0.0475	-0.0482	-0.0498	-0.0516	-0.0529	-0.052	-0.0512	-0.0504	-0.0496	-0.0488	-0.0481	-0.0476	-0.047	-0.0465	-0.0459	-0.0454	-0.0449	-0.0443	-0.0437	-0.0432	-0.0426	-0.0419	-0.0412	-0.0406	-0.0399	-0.0391	-0.0382	-0.0373	-0.0365	-0.0355	-0.0345	-0.0333	-0.0315	-0.03	-0.0285	-0.0269	-0.0253	-0.0236	-0.0221	-0.0206	-0.019	-0.0175	-0.016	-0.0147	-0.0133	-0.0121	-0.0109	-0.0099	-0.009	-0.0083	-0.0077	-0.0073	-0.007];%	-0.007	-0.0071	-0.0075	-0.0081	-0.0089	-0.0099	-0.0112	-0.0127	-0.0144	-0.0164	-0.0186];
tdc=.12;

elseif Subsonic_airfoil==10
%NACA 4412
Alpha=[	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6	-5.75	-5.5	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75];
Cl=[	-0.8374	-0.9127	-1.0965	-1.1161	-1.121	-1.1215	-1.1181	-1.099	-1.0711	-1.0462	-1.0163	-0.9847	-0.9549	-0.926	-0.8954	-0.8682	-0.8365	-0.8038	-0.7707	-0.7369	-0.707	-0.6803	-0.6491	-0.6172	-0.585	-0.5547	-0.5287	-0.5023	-0.4769	-0.4513	-0.4243	-0.3979	-0.3715	-0.3442	-0.3183	-0.2921	-0.2649	-0.2377	-0.1825	-0.1549	-0.1275	-0.0718	-0.0441	-0.0162	0.0117	0.0394	0.0674	0.0954	0.1232	0.1512	0.1791	0.2069	0.2346	0.2625	0.2903	0.318	0.3456	0.3734	0.4012	0.4288	0.4562	0.4833	0.5102	0.5366	0.5622	0.5842	0.6163	0.6525	0.6788	0.7055	0.7325	0.7592	0.7865	0.8405	0.8672	0.8941	0.921	0.9473	0.9734	0.9993	1.0254	1.0518	1.0777	1.1031	1.128	1.1523	1.1761	1.1988	1.2208	1.2417	1.2614	1.2793	1.2973	1.3164	1.3346	1.3514	1.3676	1.3835	1.4004	1.4171	1.4317	1.4484	1.4653	1.4805	1.4938	1.5061	1.5221	1.5369	1.55	1.5608	1.5688	1.5831	1.5959	1.6066	1.6141	1.6213	1.6325	1.6414	1.6474	1.6508	1.6585	1.6638	1.6661	1.6666	1.6698	1.6706];%	1.6692	1.6638	1.6605	1.6584	1.6548	1.6497	1.643	1.6346	1.6237	1.6097];
Cd=[	0.08373	0.06837	0.03328	0.0312	0.02977	0.02857	0.02751	0.02637	0.02533	0.02365	0.02288	0.02237	0.02183	0.02126	0.02078	0.01946	0.01894	0.01852	0.01808	0.01769	0.01745	0.01619	0.01569	0.01529	0.01491	0.01456	0.01426	0.01345	0.01285	0.01249	0.01214	0.01184	0.01155	0.01127	0.0108	0.01033	0.01003	0.00977	0.00935	0.00912	0.0088	0.00845	0.00824	0.0081	0.00793	0.0078	0.00769	0.00761	0.00752	0.00745	0.00737	0.00729	0.00723	0.00715	0.00709	0.00703	0.00701	0.00697	0.00694	0.00691	0.00686	0.00678	0.00658	0.00635	0.00617	0.00594	0.00588	0.00598	0.00611	0.00622	0.00633	0.00646	0.00656	0.00681	0.00696	0.00709	0.00722	0.00739	0.00758	0.00778	0.00797	0.00813	0.00834	0.00857	0.00884	0.00914	0.00948	0.00989	0.01036	0.01089	0.01149	0.0122	0.01288	0.01345	0.01404	0.01469	0.01527	0.01581	0.01631	0.01682	0.01746	0.01797	0.01849	0.01911	0.01986	0.02069	0.02129	0.02199	0.02282	0.02382	0.02506	0.02588	0.02683	0.02796	0.02939	0.03089	0.03209	0.03354	0.03528	0.03731	0.03899	0.04096	0.0433	0.04589	0.04827	0.05099];%	0.05402	0.05759	0.06101	0.06435	0.06793	0.07175	0.07583	0.08024	0.08507	0.0904];
Cdp=[	0.08141	0.06591	0.03022	0.02803	0.02651	0.02523	0.02407	0.02282	0.02165	0.01985	0.01905	0.0185	0.0179	0.01724	0.01666	0.01525	0.01471	0.01424	0.01375	0.01328	0.01297	0.01161	0.01109	0.01065	0.01022	0.00981	0.00944	0.00855	0.00791	0.00752	0.00713	0.00678	0.00644	0.00609	0.00556	0.00505	0.00471	0.00441	0.00389	0.0036	0.00325	0.00283	0.00259	0.00243	0.00228	0.00213	0.00201	0.00191	0.0018	0.00171	0.00163	0.00156	0.0015	0.00145	0.00142	0.00141	0.00139	0.00138	0.00137	0.00138	0.00139	0.00141	0.00146	0.00153	0.0016	0.0017	0.00177	0.00181	0.00186	0.00192	0.00199	0.00206	0.00213	0.00231	0.0024	0.0025	0.0026	0.00272	0.00284	0.00297	0.00311	0.00326	0.00342	0.00359	0.00379	0.00401	0.00426	0.00455	0.00488	0.00526	0.00569	0.00621	0.00672	0.00719	0.00769	0.00823	0.00875	0.00926	0.00976	0.01028	0.01091	0.01147	0.01203	0.01267	0.01343	0.0143	0.01495	0.0157	0.01656	0.01758	0.01888	0.01975	0.02075	0.02192	0.02338	0.02494	0.0262	0.0277	0.02947	0.03156	0.03332	0.03533	0.03772	0.04037	0.04284	0.04562];%	0.04871	0.05235	0.05587	0.05931	0.06298	0.06689	0.07106	0.07555	0.08047	0.0859];
Cm=[	-0.0585	-0.0687	-0.0993	-0.0956	-0.0926	-0.0896	-0.087	-0.0871	-0.0885	-0.0903	-0.0918	-0.0933	-0.0943	-0.0952	-0.0963	-0.0976	-0.099	-0.1004	-0.1019	-0.1035	-0.1041	-0.1049	-0.106	-0.1071	-0.1082	-0.1089	-0.1085	-0.1085	-0.1082	-0.1078	-0.1076	-0.1073	-0.107	-0.1068	-0.1064	-0.1061	-0.1059	-0.1057	-0.1054	-0.1052	-0.1051	-0.1049	-0.1047	-0.1046	-0.1045	-0.1044	-0.1044	-0.1043	-0.1042	-0.1041	-0.104	-0.104	-0.1039	-0.1038	-0.1038	-0.1037	-0.1036	-0.1035	-0.1035	-0.1034	-0.1033	-0.1032	-0.1031	-0.1029	-0.1024	-0.1009	-0.1014	-0.1033	-0.1029	-0.1026	-0.1024	-0.1022	-0.102	-0.1016	-0.1014	-0.1012	-0.101	-0.1007	-0.1004	-0.1001	-0.0998	-0.0995	-0.0992	-0.0988	-0.0983	-0.0978	-0.0971	-0.0963	-0.0954	-0.0943	-0.0931	-0.0915	-0.09	-0.0887	-0.0872	-0.0854	-0.0836	-0.0817	-0.0799	-0.0782	-0.0762	-0.0746	-0.0731	-0.0714	-0.0695	-0.0676	-0.0662	-0.0647	-0.0631	-0.0614	-0.0594	-0.0582	-0.0569	-0.0555	-0.054	-0.0526	-0.0516	-0.0505	-0.0493	-0.0482	-0.0474	-0.0466	-0.0458	-0.0451	-0.0447	-0.0443];%	-0.044	-0.0439	-0.0439	-0.0441	-0.0444	-0.0448	-0.0453	-0.0461	-0.047	-0.0482];
tdc=.12;

elseif Subsonic_airfoil==11
%NACA 6412
Alpha=[	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19	19.25];
Cl=[	-0.2381	-0.5884	-0.5846	-0.5665	-0.5449	-0.5247	-0.5028	-0.4796	-0.4574	-0.4341	-0.4093	-0.3853	-0.364	-0.3398	-0.3144	-0.2883	-0.2621	-0.236	-0.2093	-0.1831	-0.1577	-0.1307	-0.1038	-0.0767	-0.0493	-0.0233	0.0038	0.0312	0.058	0.0851	0.1122	0.1392	0.1663	0.1934	0.2204	0.2476	0.2742	0.3014	0.3281	0.3548	0.3817	0.4079	0.4346	0.4611	0.4871	0.5136	0.5399	0.5658	0.5922	0.6183	0.6438	0.6703	0.696	0.7219	0.7477	0.7732	0.7981	0.8176	0.8914	0.9181	0.9439	0.9712	0.9976	1.0234	1.0507	1.0771	1.1026	1.1297	1.1564	1.1821	1.2078	1.2344	1.259	1.2853	1.3097	1.3354	1.3592	1.3845	1.4079	1.4306	1.4528	1.4739	1.4931	1.5101	1.5244	1.5375	1.5493	1.5596	1.5686	1.5768	1.5846	1.5927	1.6003	1.6092	1.618	1.637	1.6458	1.6538	1.6624	1.6687	1.6767	1.6824	1.6879	1.6937	1.6955	1.7003	1.7043	1.7056	1.7028	1.7063	1.7079	1.7084];%	1.7073	1.7035	1.696	1.6929	1.6922	1.6896	1.6858	1.6817	1.6776	1.673	1.6669	1.6597	1.6514	1.6422	1.6307];
Cd=[	0.12675	0.02989	0.02502	0.02313	0.02194	0.02101	0.02003	0.0191	0.01829	0.01753	0.01689	0.01631	0.01502	0.01429	0.01381	0.01336	0.01294	0.01252	0.01216	0.01172	0.01115	0.01089	0.01059	0.01034	0.01015	0.00968	0.0095	0.00932	0.00909	0.00886	0.00874	0.00852	0.00842	0.00827	0.00818	0.00807	0.00799	0.0079	0.00784	0.00777	0.00773	0.00769	0.00766	0.00764	0.00763	0.00761	0.00762	0.00766	0.00766	0.00768	0.00773	0.00775	0.00778	0.00781	0.00783	0.00785	0.00775	0.00727	0.00694	0.00706	0.00723	0.00731	0.00745	0.00761	0.0077	0.00783	0.00802	0.00812	0.00824	0.0084	0.00857	0.00868	0.00889	0.00901	0.00922	0.00937	0.0096	0.00976	0.01002	0.0103	0.01061	0.01096	0.01135	0.01185	0.01249	0.01323	0.01405	0.01496	0.01597	0.01703	0.01815	0.01928	0.02047	0.02162	0.0228	0.02519	0.02647	0.02786	0.02925	0.03086	0.03236	0.03412	0.03595	0.03781	0.0401	0.04218	0.0444	0.04695	0.05	0.05244	0.05513	0.05801];%	0.06113	0.0646	0.06861	0.07215	0.0754	0.07891	0.08262	0.08641	0.09024	0.09416	0.09832	0.10266	0.10721	0.11196	0.11712];
Cdp=[	0.12497	0.02711	0.02188	0.01984	0.01854	0.01752	0.01643	0.01537	0.01445	0.01356	0.01278	0.01209	0.01061	0.0098	0.00926	0.00872	0.00822	0.00772	0.00725	0.00672	0.0061	0.00577	0.00543	0.0051	0.00483	0.00432	0.00408	0.00386	0.00355	0.0033	0.00312	0.00288	0.00273	0.00256	0.00243	0.00228	0.00217	0.00206	0.00197	0.00188	0.00181	0.00175	0.0017	0.00167	0.00165	0.00163	0.00162	0.00162	0.00164	0.00166	0.00169	0.00173	0.00178	0.00183	0.0019	0.00197	0.00205	0.00223	0.00244	0.00251	0.00261	0.00268	0.00278	0.00289	0.00298	0.00309	0.00323	0.00334	0.00346	0.0036	0.00375	0.00387	0.00404	0.00417	0.00434	0.00449	0.00468	0.00486	0.00507	0.0053	0.00557	0.00586	0.00618	0.00658	0.00708	0.00766	0.00833	0.00908	0.00993	0.01085	0.01184	0.01286	0.01396	0.01506	0.01621	0.01858	0.01988	0.02128	0.02269	0.02431	0.02586	0.02764	0.02952	0.03143	0.03375	0.0359	0.03819	0.04079	0.04391	0.04643	0.0492	0.05215];%	0.05533	0.05887	0.06297	0.0666	0.06994	0.07354	0.07733	0.08122	0.08513	0.08913	0.09337	0.09779	0.10243	0.10727	0.11251];
Cm=[	-0.0613	-0.1461	-0.1496	-0.1503	-0.1507	-0.1503	-0.1501	-0.1501	-0.1497	-0.1493	-0.1491	-0.1486	-0.1481	-0.1478	-0.1475	-0.1473	-0.147	-0.1468	-0.1465	-0.1463	-0.1459	-0.1457	-0.1455	-0.1453	-0.1451	-0.1447	-0.1445	-0.1443	-0.144	-0.1437	-0.1435	-0.1432	-0.1429	-0.1427	-0.1424	-0.1421	-0.1417	-0.1415	-0.1411	-0.1408	-0.1405	-0.1401	-0.1397	-0.1394	-0.1389	-0.1386	-0.1382	-0.1377	-0.1373	-0.1369	-0.1364	-0.1361	-0.1356	-0.1352	-0.1348	-0.1343	-0.1338	-0.1322	-0.1428	-0.1425	-0.1421	-0.142	-0.1417	-0.1414	-0.1413	-0.141	-0.1406	-0.1405	-0.1403	-0.14	-0.1396	-0.1395	-0.1389	-0.1387	-0.1382	-0.1379	-0.1372	-0.1369	-0.1362	-0.1354	-0.1345	-0.1334	-0.132	-0.1302	-0.128	-0.1257	-0.1232	-0.1206	-0.1178	-0.1151	-0.1124	-0.1098	-0.1073	-0.1051	-0.103	-0.0993	-0.0975	-0.0957	-0.0941	-0.0924	-0.0909	-0.0894	-0.088	-0.0867	-0.0853	-0.0842	-0.0832	-0.0822	-0.0812	-0.0805	-0.0799	-0.0793];%	-0.0788	-0.0784	-0.0781	-0.0779	-0.0778	-0.0778	-0.0779	-0.0781	-0.0784	-0.0787	-0.0792	-0.0798	-0.0806	-0.0815	-0.0826];
tdc=.12;

else
    
Alpha=	[-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75];	%14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17];
Cl=[	-0.5187	-0.5473	-0.5826	-0.6085	-0.604	-0.5974	-0.5885	-0.5668	-0.3175	-0.2861	-0.2581	-0.2391	-0.2133	-0.1847	-0.1623	-0.1357	-0.1098	-0.0848	-0.0578	-0.0307	-0.005	0.0104	0.0331	0.06	0.0863	0.1125	0.1663	0.1933	0.2199	0.2458	0.2697	0.2914	0.3119	0.3327	0.3556	0.3796	0.4042	0.4286	0.4531	0.4778	0.5024	0.5275	0.5546	0.585	0.6171	0.6498	0.6858	0.7206	0.756	0.7915	0.8164	0.8375	0.8591	0.8806	0.902	0.9242	0.9465	0.9689	0.9922	1.0148	1.0363	1.0585	1.0804	1.104	1.126	1.1488	1.1708	1.1926	1.215	1.2355	1.2533	1.2719	1.2892	1.3058	1.323	1.3379	1.3501	1.3534	1.3674	1.3795	1.3901	1.399	1.407	1.4116];	%1.4107	1.4017	1.386	1.3859	1.3841	1.3794	1.3705	1.3594	1.3447	1.3256	1.3019	1.2745	1.2436];
Cd=[	0.08084	0.07319	0.06929	0.06586	0.06065	0.05622	0.05206	0.04741	0.01467	0.01374	0.01202	0.01118	0.01053	0.00994	0.00951	0.00912	0.00879	0.00847	0.00819	0.00798	0.00771	0.00593	0.00548	0.00531	0.0052	0.00507	0.00497	0.00495	0.00494	0.00498	0.00509	0.00532	0.00564	0.006	0.00626	0.00647	0.00663	0.00678	0.00692	0.00701	0.00711	0.00723	0.00738	0.00756	0.00776	0.00798	0.0082	0.00843	0.00867	0.00891	0.00913	0.00933	0.00953	0.00975	0.01001	0.01024	0.01049	0.01078	0.01102	0.01135	0.0118	0.01221	0.01266	0.01294	0.01339	0.01374	0.01417	0.01461	0.01498	0.01551	0.01628	0.01696	0.01772	0.0185	0.01912	0.01983	0.0207	0.02218	0.02288	0.0237	0.02462	0.02567	0.0268	0.02819];	%0.03005	0.03267	0.03609	0.0384	0.04107	0.04433	0.04845	0.05341	0.05955	0.06688	0.07521	0.08401	0.09332];
Cdp=[	0.07925	0.07159	0.06766	0.06416	0.05881	0.05424	0.04991	0.04498	0.00951	0.0085	0.00669	0.00581	0.00511	0.00448	0.00401	0.00359	0.00321	0.00285	0.00252	0.00227	0.00201	0.0015	0.00143	0.00135	0.00129	0.00123	0.00113	0.0011	0.00106	0.00103	0.00101	0.00103	0.00111	0.00121	0.00132	0.00141	0.0015	0.0016	0.00171	0.00182	0.00193	0.00206	0.00219	0.00233	0.00248	0.00263	0.00278	0.00295	0.00312	0.00331	0.00348	0.00365	0.00383	0.00402	0.00422	0.00445	0.00469	0.00494	0.0052	0.00548	0.00583	0.0062	0.00662	0.00694	0.00736	0.00775	0.00817	0.00863	0.00902	0.00954	0.01023	0.01095	0.01175	0.01262	0.01332	0.0141	0.01505	0.01668	0.01746	0.01838	0.01939	0.02053	0.02174	0.02323];	%0.02522	0.028	0.03162	0.03405	0.03684	0.04023	0.0445	0.04961	0.05591	0.06341	0.0719	0.08086	0.09031];
Cm=[	-0.0402	-0.0451	-0.0442	-0.0432	-0.0473	-0.0497	-0.051	-0.0516	-0.0533	-0.054	-0.0542	-0.0524	-0.052	-0.0522	-0.0509	-0.0505	-0.05	-0.0494	-0.0491	-0.0488	-0.0483	-0.0467	-0.0458	-0.0455	-0.0451	-0.0447	-0.0441	-0.0438	-0.0435	-0.043	-0.042	-0.0407	-0.0391	-0.0378	-0.0368	-0.0361	-0.0356	-0.0349	-0.0343	-0.0336	-0.0329	-0.0323	-0.0321	-0.0328	-0.0339	-0.0352	-0.0373	-0.039	-0.041	-0.043	-0.0427	-0.0414	-0.0403	-0.0391	-0.038	-0.037	-0.036	-0.0352	-0.0344	-0.0336	-0.0327	-0.0319	-0.031	-0.0304	-0.0296	-0.0289	-0.0282	-0.0274	-0.0267	-0.0257	-0.0244	-0.0231	-0.0216	-0.02	-0.0185	-0.0167	-0.0145	-0.0111	-0.0093	-0.0074	-0.0054	-0.0034	-0.0013	0.0009];	%0.0034	0.0061	0.0084	0.0093	0.0098	0.0098	0.0089	0.0071	0.004	0.0001	-0.0043	-0.0086	-0.013];
tdc=.1537;

end

if aircraft_case==5
tdc=.12;
end

end


zzz=1;
zz=1E10;
for alt=min_alt:(max_alt-min_alt)/alt_points:max_alt
zzz=zzz+1;
alt_zzz(zzz)=alt; 
W_cc_q(1)=TOGW_i(1);
W_ii=1E20;

for q=2:q_max

    
qqq(q)=q;

% % --- Atmospheric temperature model (piecewise approximation) ---
% if alt<=11000
%     T=288.15-.0065*alt;
% else
%     T=216.65;
% end

% --- Atmospheric temperature model (piecewise approximation) ---
if alt<=11000
    T=288.15-.00649*alt;
elseif alt>11000 && alt<25000
    T=216.65;
else
    T=141.89+.00299*alt;
end

% --- Atmospheric temperature model (piecewise approximation) ---
% if alt<=11000
% rho=rho_sl*exp(-alt/8500);
% elseif alt>11000 && alt<20000
% rho=.3639*exp(-(alt-11000)/6500);
% else
% rho=.088*exp(-(alt-20000)/7000);
% end

if alt<=11000
P=101290*(T/288.08)^5.256;
elseif alt>11000 && alt<25000
P=22650*exp(1.73-.000157*alt);
else
P=2488*(T/216.6)^-11.388;
end

rho=P/(R*T);

% ---- Dynamic viscosity of air: Sutherland's Law ----                        % Temperature [K]

mu  = mu_0 * (T/T_0)^(3/2) * ((T_0 + Su)/(T + Su));   % [Pa*s]

% --- Speed of sound: a = sqrt(gamma*R*T) ---
a=(y*R*T)^.5;
% --- Speed of sound: a = sqrt(gamma*R*T) ---
a_sl=(y*R*T_sl)^.5;
% --- Freestream velocity: V = M*a ---
V_inf=M*a;
% --- Ideal-gas pressure: p = rho*R*T ---
p_inf=rho*R*T;
W_ep(q)=W_e(q-1)+W_payload;
W_l=1.1*W_ep(q);

C_p_0=-2*tdc; %Zero lift pressure coefficient, approximate, can be found using tables instead
M_cr_low=0;
M_cr_high=1;
M_cr=(M_cr_low+M_cr_high)/2;
for i=2:30
% --- Critical Mach-number / supersonic compressibility relation ---
aa=C_p_0/(1-M_cr^2)^.5;
bb=2/(y*M_cr^2)*(((2+(y-1)*M_cr^2)/(y+1))^(y/(y-1))-1);

if aa>bb
    
   M_cr=M_cr+(M_cr_high-M_cr_low)/2^i;
   
else
    
   M_cr=M_cr-(M_cr_high-M_cr_low)/2^i;
   
end

end

if sweep_solver==1
if M<M_cr
sweep=0;
else
sweep=acos(M_cr/M); %may need to look at drag divergence mach number instead
end

end

M_cr_1=M_cr+(1-M_cr)*sin(sweep);

% --- Logarithmic mean aircraft weight ---
if aircraft_case==8 && q==2
W_mean=(W_c0-W_cf)/log(W_c0/W_cf);
else
W_mean=(TOGW_i(q-1)-W_l)/log(TOGW_i(q-1)/W_l);
end

sweep_d=sweep*180/(pi);


if delta==1
% --- Delta-wing aspect ratio ---
AR=4/tan(sweep); %Delta wing
end

% --- Wing reference area from stall-speed constraint ---
if M<1
CL_max=.9*max(Cl)*cos(sweep);
end

S(q)=2*W_cc_q(q-1)/(V_stall_sl^2*rho_sl*CL_max);
% --- Delta-wing span from area and sweep ---
if delta==1
b=sqrt(4*S(q)/tan(sweep));
else
b=(AR*S(q))^.5;
end

% --- Quarter-chord sweep angle ---
if delta==0
sweep_q = atan(tan(sweep) - (S(q)/b^2)*(1-lambda)/(1+lambda)); %Derived
sweep_xc = atan(tan(sweep) - (4*S(q)*x_c/b^2)*(1-lambda)/(1+lambda)); %Derived
else
sweep_q=atan(.75*tan(sweep)); %Derived  
sweep_xc=atan((1-x_c)*tan(sweep)); %Derived
end


% --- Oswald efficiency factor correlation ---
if M>1
e=(4.61*(1-.045*AR^.68)*(cos(sweep))^.15-3.1);
else
e=1.78*(1-.045*AR^.68)-.64;
end

if delta==1
% --- Delta-wing root chord length ---
c_r=S(q)/(b/2);
else
c_r=2*S(q)/(b*(1+lambda));
end

c_t=lambda*c_r;

% --- Mach-angle parameter ---
if M>1
beta = sqrt((M)^2 - 1);
else
beta = sqrt(1-M^2);
end
% --- lift-curve slope ---
if M*cos(sweep)>1
C_l_alpha=4/beta;
% C_L_alpha=C_l_alpha*.9*cos(sweep_q);
C_L_alpha=4/beta;
else
C_l_alpha=2*pi/beta;
% C_L_alpha=C_l_alpha*tan(pi/2-sweep)/(1-beta^2*(tan(pi/2-sweep))^2)^.5;
% C_L_alpha=C_l_alpha*.9*cos(sweep_q);
eta_s = C_l_alpha/(2*pi/beta);
F = 1.07*(1 + D_f/b)^2;
C_L_alpha = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep)^2/beta^2))) * F;
end

% --- Engine inlet wave/form drag: D = q*S_inlet*CD ---
if type==0
    D_engine=.5*rho*V_inf^2*(pi*N_w^2*.25)*Cd_engine*n;
else
    D_engine=.5*rho*V_inf^2*(pi*N_w^2*.25)*Cd_engine*n; 
end

Cd_engine_s=D_engine/(.5*rho*V_inf^2*S(q));
    
if M>=1

beta_low=0;
beta_high=pi/4;
beta_sh=(beta_low+beta_high)/2;
for xx=2:xx_max

% --- Oblique-shock theta-beta-M relation ---
nose_angle_i=atan(2*cot(beta_sh)*(M^2*(sin(beta_sh))^2-1)/(M^2*(y+cos(2*beta_sh))+2));

if nose_angle_i<nose_angle*pi/180
    beta_sh=beta_sh+(beta_high-beta_low)/2^xx;
else
    beta_sh=beta_sh-(beta_high-beta_low)/2^xx;
end

end

end

if M>=1
% --- Pressure immediately behind oblique shock ---
p_shock=(1+2*y/(y+1)*(M^2*(sin(beta_sh))^2-1))*p_inf;
else
p_shock=0;
end

% --- Waverider compression lift: L = Delta-p*S ---
L_comp=(p_shock-p_inf)*S(q);
L_comp_max=(p_shock-p_inf)*S(q);

if L_comp>W_mean
L_comp=W_mean;
end
 
if L_comp_max>TOGW_i(q-1)
L_comp_max=TOGW_i(q-1);
end

if Waverider==0
L_comp=0;
L_comp_max=0;
end

% --- Wing Volume Calculation ---
if M>=1
Aw=2/3;
else
Aw=.685;
end

if delta==0
V_wing=Aw*tdc*(c_r*b-(c_r-c_t)*b/2);
else
V_wing=Aw*tdc*c_r*b/2;
end
% Wing Volume Calculation
V_fuel_wing = V_wing*.5;
% --- Fuselage Volume Calculation ---
% V_fus=1/4*pi*D_f^2*L;
% --- Fuel Volume Calculation ---
V_fuel=W_f(q-1)/(g*rho_f);
% --- Fuselage Volume Calculation ---
V_fus=V_fuel-V_fuel_wing;
% --- Fuselage Length Calculation ---
if L==0
L=V_fus*4/(pi*D_f^2)+10;
end
% --- Wetted surface area
S_wet=2*S(q)+pi*D_f*L;
% --- Wing form-factor correlation ---
FF_wing=1+.6/x_c*tdc+100*tdc^4*(1.34*M^.18*cos(sweep_xc)^.28);
% --- Fuselage form-factor correlation ---
FF_fus=1+60/(L/D_f)^3+(L/D_f)/400;
% --- Nacelle form-factor correlation ---
FF_nac=1+.35/(L/D_f);
% --- Nacelle surface area ---
S_fus=pi*D_f*L;
% --- Total form-factor correlation ---
FF=FF_wing*S(q)*2/S(q)+FF_fus*(S_fus)/S(q)+FF_nac*(S_n)/S(q);
% --- Sutherland-law dynamic viscosity ---
u=u_0*((T/T_sl)^(3/2))*((T_sl+Su)/(T+Su));
% --- Reynolds number ---
Re=rho*M*a*L/u;
% --- Reynolds number ---
Re_to=rho*V_lof*L/u_0;
% --- cutoff reynolds number
k_m=.06E-3; %in
if M<1
R_cutoff = 38.21*(L*3.28*12/k_m)^1.053;
else
R_cutoff = 44.62*(L*3.28*12/k_m)^1.053* M^1.16;
end

c_f_to=.455/(((log10(Re_to))^2.58)*(1+.144*(V_lof/a_sl)^2)^.65);
c_f_to=.074/Re_to^.2;
% --- Supersonic turbulent skin-friction coefficient ---
if R_cutoff>Re
if M>=1
c_f=.455/(((log10(Re))^2.58)*(1+.144*M^2)^.65);
else
c_f=.074/Re^.2;
end
else
if M>=1
c_f=.455/(((log10(R_cutoff))^2.58)*(1+.144*M^2)^.65);
else
c_f=.074/R_cutoff^.2;
end
end
% --- Skin-friction drag coefficient ---
C_d_skin = c_f * FF;
% --- Skin-friction drag coefficient ---
C_d_skin_to = c_f_to * FF;
% --- Zero-lift wing wave-drag coefficient ---
% C_d_wave_wing_0 = 4 * (tdc^2) * (cos(sweep))^2 / beta;
% --- Mach number normal to the leading edge of the wing ---
% 


M_n = M*cos(sweep); %Normal wing component Mach number


% --- Zero-lift wing wave-drag coefficient ---
if M_n >= 1

    C_d_wave_wing_0 = 4*(tdc^2)/beta;

else 

    C_d_wave_wing_0 = 0;

end

if M>=1
% --- Fuselage wave-drag coefficient ---
C_d_wave_body_0 = 128*(V_fuel)^2/(pi*L^4*S(q));
else
C_d_wave_body_0 = 0;
end

% --- Fuselage wave-drag coefficient ---
% C_d_wave_body_0 = 128*V_inf^2/(pi*L^4*S(q));
% --- Zero-lift drag coefficient ---
if M>=1
C_d_0 = C_d_skin + C_d_wave_wing_0 + C_d_wave_body_0;
C_d_0_to=C_d_skin_to;
else
C_d_0 = min(Cd);
C_d_0_to = min(Cd);
end

% --- Lift coefficient including waverider compression lift ---
C_L=(W_mean-L_comp)/(.5*rho*V_inf^2*S(q)); %Do W-mean-compression lift for waverider
% --- Max Lift coefficient including waverider compression lift ---
C_L_max_W=(TOGW_i(q-1)-L_comp_max)/(.5*rho*V_inf^2*S(q));

if M<1
L_s_2d=C_l_alpha; 
L_s_3d=C_L_alpha;
C_l=C_L/(cos(sweep))^2; %The wing "feels" the normal freestream component through the dynamic pressure f(V^2) using Betz's wing sweep theory which is why cos(sweep)^2 is used

lowerIdx = find(Cl <= C_l, 1, 'last');
if lowerIdx == length(Cl)
upperIdx = lowerIdx;
else
upperIdx = lowerIdx + 1;
end
lowerIdx_alpha=Alpha(lowerIdx);
upperIdx_alpha=Alpha(upperIdx);
lowerIdx_Cl=Cl(lowerIdx);
upperIdx_Cl=Cl(upperIdx);
lowerIdx_Cd=Cd(lowerIdx);
upperIdx_Cd=Cd(upperIdx);
sl_Cl=(upperIdx_Cl-lowerIdx_Cl)/(upperIdx_alpha-lowerIdx_alpha);
sl_Cd=(upperIdx_Cd-lowerIdx_Cd)/(upperIdx_alpha-lowerIdx_alpha);
co_Cl=lowerIdx_Cl-sl_Cl*lowerIdx_alpha;
co_Cd=lowerIdx_Cd-sl_Cd*lowerIdx_alpha;
% alpha=(C_l-co_Cl)/sl_Cl;
alpha=((C_l-lowerIdx_Cl)/(upperIdx_Cl-lowerIdx_Cl))*.25+lowerIdx_alpha;
if lowerIdx_alpha==upperIdx_alpha
C_d=Cd(lowerIdx);
else
C_d=sl_Cd*alpha+co_Cd;
end
C_D=C_d+C_L^2/(pi*AR*e);
D_w=C_D*.5*rho*V_inf^2*S(q);
alpha=alpha*pi/180;

% C_l_Dmax=C_L_max_W/(.9*cos(sweep));
% C_l_Dmax=C_L_max_W/(1-C_L_max_W/(pi*e*AR*2*pi));
C_l_Dmax=C_L_max_W*(1-M^2)^.5*(1+L_s_2d/(pi*e*AR));
% C_l_Dmax=C_L_max_W*L_s_2d/L_s_3d;
lowerIdx_Dmax = find(Cl <= C_l_Dmax, 1, 'last');
if lowerIdx_Dmax == length(Cl)
upperIdx_Dmax = lowerIdx_Dmax;
else
upperIdx_Dmax = lowerIdx_Dmax + 1;
end

lowerIdx_alpha_Dmax=Alpha(lowerIdx_Dmax);
upperIdx_alpha_Dmax=Alpha(upperIdx_Dmax);
lowerIdx_Cl_Dmax=Cl(lowerIdx_Dmax);
upperIdx_Cl_Dmax=Cl(upperIdx_Dmax);
lowerIdx_Cd_Dmax=Cd(lowerIdx_Dmax);
upperIdx_Cd_Dmax=Cd(upperIdx_Dmax);
sl_Cl_Dmax=(upperIdx_Cl_Dmax-lowerIdx_Cl_Dmax)/(upperIdx_alpha_Dmax-lowerIdx_alpha_Dmax);
sl_Cd_Dmax=(upperIdx_Cd_Dmax-lowerIdx_Cd_Dmax)/(upperIdx_alpha_Dmax-lowerIdx_alpha_Dmax);
co_Cl_Dmax=lowerIdx_Cl_Dmax-sl_Cl_Dmax*lowerIdx_alpha_Dmax;
co_Cd_Dmax=lowerIdx_Cd_Dmax-sl_Cd_Dmax*lowerIdx_alpha_Dmax;
alpha_Dmax=((C_l_Dmax-lowerIdx_Cl_Dmax)/(upperIdx_Cl_Dmax-lowerIdx_Cl_Dmax))*.25+lowerIdx_alpha_Dmax;
if lowerIdx_alpha_Dmax==upperIdx_alpha_Dmax
C_d_Dmax=Cd(lowerIdx_Dmax);
else
C_d_Dmax=sl_Cd_Dmax*alpha_Dmax+co_Cd_Dmax;
end
C_D_Dmax=C_d_Dmax+C_L_max_W^2/(pi*AR*e);
D_max_0=C_D_Dmax*.5*rho*V_inf^2*S(q);
alpha_Dmax=alpha_Dmax*pi/180;
else
L_s_2d=4/(M^2-1)^.5;                % Ackeret 2-D lift slope
% L_s_3d=L_s_2d/(1+L_s_2d/(pi*AR*e));
L_s_3d=L_s_2d;
C_l=C_L;
% C_l=C_L/(cos(sweep))^2;
% C_l_Dmax=C_L_max_W/(cos(sweep))^2;
C_l_Dmax=C_L_max_W;
end

% --- Angle of attack from linear lift relation ---
if M>1
alpha=C_L/C_L_alpha;
alpha_max=C_L_max_W/C_L_alpha;
end
% % --- Induced angle of attack ---
% alpha_i=C_L/(pi*AR*e);




% % --- induced wing wave-drag coefficient ---
% if M >= 1.1
%     beta_n = sqrt(M^2 - 1);
%     K = beta_n/(pi*AR);
% elseif M <= 1
%     K = 1/(pi*AR*e);
% else
%     % Transonic transition region
%     beta_n = sqrt(1.1^2 - 1);
%     K = beta_n/(pi*AR);
% end
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


% % --- Induced wing wave-drag coefficient ---
% if M >= 1
%     beta_n = sqrt(M_n^2 - 1);
%     K = AR*(M^2-1)/((4*AR*(M^2-1)^.5)-2)*cos(sweep);
% %     K = beta_n/(4);
% else 
%     K = 1/(pi*AR*e);
% end

K=S_ww_t*K100+(1-S_ww_t)*K0; %Raymer
% else
%     % Transonic transition region
%     beta_n = sqrt(1.1^2 - 1);
%     K = beta_n/(4);
% end






% if M*cos(sweep)<1
% K=0;
% else
% K=1/((((M*cos(sweep))^2-1)^.5)*4);
% end

% --- Induced drag coefficient ---
% if droop==0
c_d_i=K*C_L^2;
% else
% c_d_i=C_L*tan(alpha)*cos(alpha_i);
% c_d_i=K*C_L^2-C_L^2/(pi*AR*e);
% c_d_i=0;
% end

D_p=.5*rho*V_inf^2*S_wet*c_f;
% S_fus=pi*D_f^2*(L/D_f-1.7);
D_p_f=.5*rho*V_inf^2*S_fus*c_f;

if M>1
% --- Maximum induced drag coefficient ---
c_d_i_max=K*C_L_max_W^2;
% --- Parasite drag force ---
D_p=.5*rho*V_inf^2*S_wet*c_f;
% --- Induced drag force ---
D_i=c_d_i*.5*rho*V_inf^2*S(q);
% --- Maximum induced drag force ---
D_i_max=c_d_i_max*.5*rho*V_inf^2*S(q);
% --- Lift-dependent supersonic wave drag ---
% C_d_wave_wing_i=4*alpha^2/((M^2-1)^.5);


if M_n >= 1
    C_d_wave_wing_i = (C_L^2/4)*sqrt(M_n^2 - 1);
    C_d_wave_wing_i_max=(C_L_max_W^2/4)*sqrt(M_n^2 - 1);
else 
    C_d_wave_wing_i = 0;    % do not use Ackeret
    C_d_wave_wing_i_max=0;
end


% % --- Max lift-dependent supersonic wave drag ---
% C_d_wave_wing_i_max=4*alpha_max^2/((M^2-1)^.5);
% --- Maximum total wave drag ---
D_wave_max = (C_d_wave_wing_i_max) * 0.5*rho*V_inf^2*S(q);   
% --- Lift-dependent wave-drag force ---
D_wave_i=C_d_wave_wing_i*.5*rho*V_inf^2*S(q);
% --- Max lift-dependent wave-drag force ---
D_wave_i_max=C_d_wave_wing_i_max*.5*rho*V_inf^2*S(q);
end




%% ================= SUBSONIC TAIL DRAG =================

%% ---- Horizontal tail ----


if S_H > 0

    % Reynolds number
    Re_H = rho*V_inf*c_H/mu;

    % Turbulent skin friction
    Cf_H = 0.455 / ...
        ((log10(Re_H))^2.58 * ...
        (1 + 0.144*M^2)^0.65);

    % Form factor
    FF_H = 1 + 0.6/(x_t_H)*(t_c_H) ...
        + 100*(t_c_H)^4;

    % Interference factor
    Q_H = 1.0;

    % Wetted area
    Swet_H = 2*S_H;

    % Zero-lift parasite drag coefficient
    CD0_H = Cf_H*FF_H*Q_H*(Swet_H/S(q));

else

    CD0_H = 0;

end


%% ---- Vertical tail ----

if S_V > 0

    % Reynolds number
    Re_V = rho*V_inf*c_V/mu;

    % Turbulent skin friction
    Cf_V = 0.455 / ...
        ((log10(Re_V))^2.58 * ...
        (1 + 0.144*M^2)^0.65);

    % Form factor
    FF_V = 1 + 0.6/(x_t_V)*(t_c_V) ...
        + 100*(t_c_V)^4;

    % Interference factor
    Q_V = 1.0;

    % Wetted area
    Swet_V = 2*S_V;

    % Zero-lift parasite drag coefficient
    CD0_V = Cf_V*FF_V*Q_V*(Swet_V/S(q));

else

    CD0_V = 0;

end


q_d = 0.5*rho*V_inf^2;

% ---- Horizontal tail ----
AR_H = b_H^2/S_H;

% CDi_H = CL_H^2/(pi*e_H*AR_H);
CDi_H = 0;

CD_H = CD0_H + CDi_H;

D_H = q_d*S_H*CD_H;


% ---- Vertical tail ----
AR_V = b_V^2/S_V;

% CDi_V = CL_V^2/(pi*e_V*AR_V);
CDi_V = 0;

CD_V = CD0_V + CDi_V;

D_V = q_d*S_V*CD_V;


% ---- Total tail drag ----
D_tail = D_H + D_V;











% --- Cruise time: t = R/V ---
time=Range_cruise/V_inf;

if M>2
if TPS==1
% ==================== AEROTHERMAL PARAMETERS ====================

T_r=T*1.8; %Atmospheric temperature (Rankine) 
R_s=R*0.06857/287.05; %Gas constant [Btu/lbm*R]
Zw=2; %Compressibility factor
H_inf=0.241*T_r; %freestream enthalpy [Btu/lbm]
u_inf=(1.05*10^-7)*T_r^0.75; %Dynamic viscosity [Lbm/ft*s]
rho_s=rho*0.062428; %Atmospheric density [lb/ft^3]
p_s=p_inf*.020885; %Atmospheric pressure [lbf/ft^2]

%conditions behind normal shockwave at nose tip
% --- Normal-shock static pressure relation ---
P2=p_s*(2*y*M*M-(y-1))/(y+1);
% --- Normal-shock density relation ---
rho2=rho_s*M*M*(y+1)/((y-1)*M*M+2);
% --- Normal-shock temperature relation ---
T2=T_r*(2*y*M*M-(y-1))*((y-1)*M*M+2)/((y+1)^2*M*M);
% --- Normal-shock downstream Mach number ---
M2=(((y-1)*M*M+2)/(2*y*M*M-(y-1)))^0.5;
% --- Normal shock enthalpy ---
H2=(((-1*10^-9)*T2^3)+((1.5*10^-5)*T2^2)+(0.2201*T2)); 
% --- Normal shock speed of sound ---
a2=(y*P2/rho2)^0.5;
% --- Normal shock velocity 
V2=M2*a2;
% --- Stagnation pressure behind nose shock ---
P_st=P2*(1+(y-1)*M2*M2/2)^(y/(y-1));
% --- Stagnation density behind nose shock ---
rho_st=rho2*(1+(y-1)*M2*M2/2)^(1/(y-1));
% --- Leading-edge streamline velocity-gradient relation ---
dudx=(1/R_wle)*(2*(P_st-p_s)*g_s/rho_st);
% --- Stagnation enthalpy ---
H_st=H2+V2^2/(2*g_s*J);
% --- Recovery enthalpy ---
r   = Pr^(1/3);
H_r = H2 + r * (V2^2) / (2 * g_s * J);
%The for loop below solves for the stagnation temperature (T_st) from H_st. A
%formula that determines enthalpy (J/Kg) as a function of temperature was
%previously determined using excel and AFProp. 
 T_st=5000; %Stagnation temperature initial value for loop 
 T_st_initial=T_st; %Stagnation temperature initial value for loop 
for iii=1:iii_max
% --- Enthalpy-temperature polynomial ---
    H_st_i=(((-1*10^-9)*T_st^3)+((1.5*10^-5)*T_st^2)+(0.2201*T_st)); 
    
    if H_st_i>H_st
        T_st=T_st-(T_st_initial/(2^iii)); 
    else
        T_st=T_st+(T_st_initial/(2^iii)); 
    end
end
T_st=T_st*1; %Stagnation temperature [Rankine]

%Stagnation dynamic viscosity [Lbm/ft*s]
u_st=(1.05*10^-7)*T_st^0.75;

    Tw=10000; %Stagnation temperature initial value for loop 
    Tw_initial=Tw;

    for i=1:1000
% --- Wall air enthalpy
    H_w=(((-1*10^-9)*Tw^3)+((1.5*10^-5)*Tw^2)+(0.2201*Tw)); 
% --- Wall air density from ideal-gas relation ---
    rho_w=P_st/(Zw*R_s*Tw);
% --- Wall air dynamic viscosity ---
    u_w=(1.05*10^-7)*Tw^0.75;
% % --- Convective heat-transfer coefficient correlation ---
%     h=0.94*((rho_st*u_st)^0.4)*((rho_w*u_w)^0.1)*((dudx)^0.5);
% % --- Convective heat flux ---
%     q_conv= h*(H_st-H_w);
% --- Stephan - Boltzmann constant ---
    emmissivity=.8;
    beta_a=emmissivity*0.481*10^-12; %BTU/ft^2*R^4
% --- Radiative heat flux: q_rad = beta*T_w^4 ---
    q_rad= beta_a*(Tw)^4;
    Re_x = rho2 * V2 * x / u_inf;
    Cf   = 0.0592 / Re_x^0.2;          % simple turbulent Cf
    St   = Cf / (2 * Pr^(2/3));
    h_ls    = St * rho2 * V2;
    q_conv_B= h_ls*(H_r-H_w);
    
    if q_conv_B<q_rad
        Tw=Tw-(Tw_initial/(2^i)); 
    else
        Tw=Tw+(Tw_initial/(2^i)); 
    end
 
    end 

% --- Total thermal-protection-system weight ---
W_tps=S_cooling*rho_cooling*g+rho_ins*g*S_wet*t_ins;

end
end

% % --- Skin weight ---
% if skin==0
%     W_skin=S_wet*rho_titanium*t_tit*g;
% else
%     W_skin=S_wet*rho_aluminum*t_aluminum*g;
% end

% --- Zero-lift drag force ---
c_d_0_t=(C_d_0+Cd_engine_s+CD0_H+CD0_V);
D_0= c_d_0_t*.5*rho*V_inf^2*S(q); 

if M>=1
% --- Total cruise drag ---
D = D_0 + D_i + D_wave_i;
CD=D/(.5*rho*V_inf^2*S(q));
% D = 1.553E5;
% --- Maximum drag including drag margin ---
D_max = D_0 + D_i_max + D_wave_max;
else
D=D_w+D_p_f+D_engine+D_tail;
D_max=D_max_0+D_p_f+D_engine+D_tail;
CD=D/(.5*rho*V_inf^2*S(q));
end

% --- Lift-to-drag ratio ---
LdD(q)=W_mean/D;

% --- Propulsion equations --- %

if type==1
 % --- Inlet total temperature ---
T_0_2=T*(1+(y-1)/2*M^2);
T_0_2_sl=T_sl*(1+(y-1)/2*0^2);
% --- Inlet total pressure including inlet efficiency ---
P_0_2=eta_inlet*p_inf*(1+(y-1)/2*M^2)^(y/(y-1));
P_0_2_sl=eta_inlet*p_sl*(1+(y-1)/2*0^2)^(y/(y-1));
for i=2:i_max+2
CPR=(CPR_max-1)*i/i_max+1; %Pt3/Pt2
CPR=(CPR_max-CPR_min)*(i-2)/i_max+CPR_min;

TSFC_i(1)=0;
TSFC_k(1)=0;
for j=2:j_max+2
% BR=BR_max*j/j_max; %Bypass ratio
BR=(BR_max-BR_min)*(j-2)/j_max+BR_min;
% --- Ideal compressor exit total temperature ---
T_0_3s = T_0_2 * CPR^((y-1)/y);
T_0_3s_sl = T_0_2_sl * CPR^((y-1)/y);
% --- Actual compressor exit temperature using eta_c ---
T_0_3  = T_0_2 + (T_0_3s-T_0_2)/eta_c;
% T_0_3_sl  = T_0_2_sl + (T_0_3s_sl-T_0_2_sl)/eta_c;
% --- Compressor specific work ---
W_c=Cp*(T_0_3-T_0_2);
W_c_sl=Cp*(T_0_3_sl-T_0_2_sl);
% --- Compressor total pressure relations ---
P_0_3=P_0_2*CPR;
P_0_3_sl=P_0_2_sl*CPR;

TIT_d=(TIT_d_low+TIT_d_high)/2;
TIT_d_Dmax=(TIT_d_low+TIT_d_high)/2;
for k=2:k_max

% --- Fan pressure ratio derived from available cycle energy ---
% if BR>0
%     FPR=((1+(TIT_d-(TIT_d - (W_c + (1+BR)*(Cp*((T_0_2 + (TIT_d - T_0_3)/(1+BR))-T_0_2)))/(Cp*(1+(Cp*(TIT_d-T_0_3)+(1+BR)*(Cp*((T_0_2 + (TIT_d - T_0_3)/(1+BR))-T_0_2)))/E_sp)))-(T_0_3-T_0_2))/((1+BR)*T_0_2))^(y/(y-1)));
% else
%     FPR=0;
% end

if BR==0
    FPR=0;
end

% if BR>0
%     FPR_Dmax=((1+(TIT_d_Dmax-(TIT_d_Dmax - (W_c + (1+BR)*(Cp*((T_0_2 + (TIT_d_Dmax - T_0_3)/(1+BR))-T_0_2)))/(Cp*(1+(Cp*(TIT_d_Dmax-T_0_3)+(1+BR)*(Cp*((T_0_2 + (TIT_d_Dmax - T_0_3)/(1+BR))-T_0_2)))/E_sp)))-(T_0_3-T_0_2))/((1+BR)*T_0_2))^(y/(y-1)));
% else
%     FPR_Dmax=0;
% end

if BR==0
    FPR_Dmax=0;
else
    FPR_Dmax=FPR;
end

% --- Fan total pressure relations ---
P_0_3_f=P_0_2*FPR;
P_0_3_f_Dmax=P_0_2*FPR_Dmax;
P_0_3_f_sl=P_0_2_sl*FPR;

% --- Ideal fan exit total temperature ---
T_0_3s_f = T_0_2 * FPR^((y-1)/y);
T_0_3s_sl_f = T_0_2_sl * FPR^((y-1)/y);
% --- Actual fan exit temperature using eta_F ---
T_0_3_f  = T_0_2 + (T_0_3s_f-T_0_2)/eta_f;
T_0_3_f_sl  = T_0_2_sl + (T_0_3s_sl_f-T_0_2_sl)/eta_f;

% % --- Actual fan exit temperature relations ---
% T_0_3_f = T_0_2 * (1 + (FPR^((y-1)/y)-1)/eta_f);

T_0_3_f_Dmax = T_0_2 * (1 + (FPR_Dmax^((y-1)/y)-1)/eta_f);
% T_0_3_f_sl = T_0_2_sl * (1 + (FPR^((y-1)/y)-1)/eta_f);


% --- Fan specific work ---
if BR>0
    W_ff=(Cp*(T_0_3_f-T_0_2));
    W_f_Dmax=(Cp*(T_0_3_f_Dmax-T_0_2));
    W_f_sl=(Cp*(T_0_3_f_sl-T_0_2_sl));
else
    W_ff=0;
    W_f_Dmax=0;
    W_f_sl=0;
end

% --- Main combustor fuel-air ratio ---
if (Scramjet==0 || Scramjet==3) && CPR>1
f=((Cp*(TIT_d-T_0_3))/(E_sp*eta_b)); %obtained by setting the air flow enthalpy plus the fuel flow enthalpy to the total burner enthalpy. Next, assume the fuel mass flowrate is negligible to the air mass flow rate and solve for f
f_Dmax=((Cp*(TIT_d_Dmax-T_0_3))/(E_sp*eta_b));
f_max=((Cp*(TIT_d_high-T_0_3))/(E_sp*eta_b));
f_max_sl=((Cp*(TIT_d_high-T_0_3_sl))/(E_sp*eta_b));
else
f=0;
f_Dmax=0;
f_max=0;
f_max_sl=0;
end

if (Scramjet==0 || Scramjet==3) && CPR>1
f_sl=((Cp*(TIT_d_high-T_0_3_sl))/(eta_b*E_sp));
else
f_sl=0;
end

% --- Total temperature and pressure behind turbine ---
if (Scramjet==0 || Scramjet==3) && CPR>1
% --- Turbine exit temperature from shaft-work balance ---
T_0_5s = TIT_d - (W_c + BR*W_ff)/(Cp*(1+f)*eta_m);
T_0_5s_Dmax = TIT_d_Dmax - (W_c + BR*W_f_Dmax)/(Cp*(1+f)*eta_m);
T_0_5s_sl = TIT_d - (W_c_sl + BR*W_f_sl)/(Cp*(1+f_sl)*eta_m);
T_0_5s_max = TIT_d_high - (W_c + BR*W_ff)/(Cp*(1+f)*eta_m);
% --- actual turbine exit temperature ---
T_0_5 = TIT_d - (TIT_d-T_0_5s)/eta_t;
T_0_5_Dmax = TIT_d_Dmax - (TIT_d_Dmax-T_0_5s)/eta_t;
T_0_5_sl = TIT_d - (TIT_d-T_0_5s_sl)/eta_t;
T_0_5_max = TIT_d_high - (TIT_d_high-T_0_5s_max)/eta_t;
% --- Turbine exit total pressure ---
P_0_5 = P_0_3 * (T_0_5s/TIT_d)^(y/(y-1));
P_0_5_Dmax = P_0_3 * (T_0_5s_Dmax/TIT_d_Dmax)^(y/(y-1));
P_0_5_sl = P_0_3_sl * (T_0_5s_sl/TIT_d)^(y/(y-1));
P_0_5_max = P_0_3 * (T_0_5s_max/TIT_d_high)^(y/(y-1));
else
T_0_5=T_0_2;
P_0_5=P_0_2;
T_0_5_Dmax=T_0_2;
P_0_5_Dmax=P_0_2;
T_0_5_sl=P_0_2_sl;
P_0_5_sl=P_0_2_sl;
T_0_5_max=T_0_2;
P_0_5_max=P_0_2;
end

% --- Nozzle exit temperature ---
T_e=T_0_5*(p_inf/P_0_5)^((y-1)/y);
T_e_Dmax=T_0_5_Dmax*(p_inf/P_0_5_Dmax)^((y-1)/y);
T_e_sl=T_0_5_sl*(p_sl/P_0_5_sl)^((y-1)/y);
T_e_max=T_0_5_max*(p_inf/P_0_5_max)^((y-1)/y);
V_e_core_max = (sqrt(2*Cp*(T_0_5_max - T_e_max)));


% --- Core exhaust velocity from energy equation ---
if (Scramjet==0 || Scramjet==3) && CPR>1
    if T_0_5<T_e
        V_e_core=0;
    else
        V_e_core = (sqrt(2*Cp*(T_0_5 - T_e)));
    end
    
    if T_0_5_Dmax<T_e_Dmax
        V_e_core_Dmax=0;
    else
        V_e_core_Dmax = (sqrt(2*Cp*(T_0_5 - T_e)));
    end
    
    if T_0_5_sl<T_e_sl
        V_e_core_sl=0;
    else
        V_e_core_sl = (sqrt(2*Cp*(T_0_5_sl - T_e_sl)));
    end
else
V_e_core = V_inf;
V_e_core_Dmax = V_inf;
V_e_core_sl = 0;
end

if V_e_core<1 && V_e_core>-1
   V_e_core=0;
end

% --- Fan-stream exit velocity ---
if BR>0
    V_e_f = (sqrt(2*eta_f*Cp*(T_0_3_f - T)));
else   
    V_e_f=0;
end

if BR>0
    V_e_f_Dmax = (sqrt(2*eta_f*Cp*(T_0_3_f_Dmax - T)));
else   
    V_e_f_Dmax=0;
end

if BR>0
    V_e_f_sl = (sqrt(2*eta_f*Cp*(T_0_3_f_sl - T_sl)));
else   
    V_e_f_sl=0;
end


m_e_f_i_sl=V_e_f_sl*A_inlet*BR/(BR+1);

V_e_f_i_sl=m_e_f_i_sl/(A_inlet*BR/(BR+1)*rho_sl);

% --- Combustor exit density ---
rho_e_sl=p_sl/(R*T_e_sl);
% --- Core mass-flow rate ---
m_core=V_inf*A_inlet/(BR+1)*rho;
% m_core_sl=V_e_core_sl*A_inlet*rho_e_sl/(1+f_sl);

% --- Nozzle choked area 
if Afterburner==0
A_star = 0.75 * A_inlet;          % non-afterburning
else
A_star = 0.95 * A_inlet;          % with afterburner lit
end

m_core_sl_high=1000;
m_core_sl=m_core_sl_high/2;
jj_max=10;
for jj=2:jj_max
% m_core_sl = (P_0_2_sl / sqrt(T_0_2_sl)) * A_star*1/(BR+1) * sqrt(y/R) * ( (y+1)/2 )^(-(y+1)/(2*(y-1)));
% --- Bypass mass-flow rate ---
m_o_f=(V_inf*A_inlet*rho*BR/(BR+1));
m_o_f_sl=(V_e_f_sl*A_inlet*rho_sl*BR/(BR+1));
% --- Fuel mass-flow rate ---
m_f=f*m_core;
m_f_Dmax=f_Dmax*m_core;
m_f_sl=f_sl*m_core_sl;
m_f_max=f_max*m_core;

V_i_core_sl=m_core_sl/(rho_sl*A_inlet/(BR+1));

% ==================== THERMAL EFFICIENCY ====================
eta_th_sl = (0.5*(m_core_sl + m_f_sl)*V_e_core_sl^2 - 0.5*m_core_sl*V_i_core_sl^2) ...
         /(m_f_sl*E_sp);

% % ==================== PROPULSIVE EFFICIENCY ====================
% eta_p = (Thrust*V_inf) ...
%         /(0.5*(m_core + m_f)*V_e^2 - 0.5*m_core*V_inf^2);

% ==================== OVERALL EFFICIENCY ====================


Po_sl=m_f_sl*E_sp*eta_th_sl;

Po_sl_B=.5*(m_core_sl+m_f)*V_e_core_sl^2;

if Po_sl>Po_sl_B
    m_core_sl=m_core_sl+m_core_sl_high/2^jj;
else
    m_core_sl=m_core_sl-m_core_sl_high/2^jj;
end

end


% --- Net engine thrust from momentum balance ---
if BR>0
    Th = (m_core + m_f) * (V_e_core)-m_core*V_inf + m_o_f*(V_e_f - V_inf);
else
    Th = (m_core + m_f) * (V_e_core)-m_core*V_inf;
end

if BR>0
    Th_Dmax = (m_core + m_f_Dmax) * (V_e_core_Dmax)-m_core*V_inf + m_o_f*(V_e_f_Dmax - V_inf);
else
    Th_Dmax = (m_core + m_f_Dmax) * (V_e_core_Dmax)-m_core*V_inf;
end

if BR>0
    Th_sl = (m_core_sl + m_f_sl) * (V_e_core_sl)-m_core_sl*V_i_core_sl + m_o_f_sl*(V_e_f_sl - V_e_f_i_sl);
else
    Th_sl = (m_core_sl + m_f_sl) * (V_e_core_sl)-m_core_sl*V_i_core_sl;
end

if BR>0
    Th_max = (m_core + m_f_max) * (V_e_core_max)-m_core*V_inf + m_o_f*(V_e_f - V_inf);
else
    Th_max = (m_core + m_f_max) * (V_e_core_max)-m_core*V_inf;
end

% --- Iterative adjustment of turbine-inlet temperature ---
if Th<D/n    
    TIT_d=TIT_d+(TIT_d_high-TIT_d_low)/2^k;    
else   
    TIT_d=TIT_d-(TIT_d_high-TIT_d_low)/2^k;    
end

if Th<D_max/n    
    TIT_d_Dmax=TIT_d_Dmax+(TIT_d_high-TIT_d_low)/2^k;    
else   
    TIT_d_Dmax=TIT_d_Dmax-(TIT_d_high-TIT_d_low)/2^k;    
end

end

% --- Including afterburner ---

n_ab=0; %Initial value for afterburner fuel fraction compared to the stoichometric ratio
if (Th+1)<D/n && Afterburner==1
   
n_ab=(n_ab_low+n_ab_high)/2;
for kk=2:kk_max
    
% --- Afterburner fuel-air ratio ---
f_6_k=n_ab*(ratio_fo-f);
% --- Afterburner exit total temperature ---
T_0_6_k = T_0_5 + eta_ab*f_6_k*E_sp/((1+f)*Cp);
T_0_6_k_sl = T_0_6_high;
T_0_6_k_max=T_0_6_high;

% --- Scramjet exit Mach number from total-temperature relation ---
if T_0_6_k>T_0_6_high && (Scramjet==1 || Scramjet==3)
M_6=(2/(y-1)*(T_0_6_k/T_0_6_high-1))^.5;
else
M_6=0;
end


% --- Afterburner total pressure ---
P_0_6_k=P_0_5;
P_0_6_k_max=P_0_5;
P_0_6_k_sl=P_0_5_sl;
% --- Afterburner nozzle exit temperature ---
T_e_ab_k=T_0_6_k*(p_inf/P_0_6_k)^((y-1)/y);
T_e_ab_k_max=T_0_6_k_max*(p_inf/P_0_6_k_max)^((y-1)/y);
T_e_ab_k_sl=T_0_6_k_sl*(p_sl/P_0_6_k_sl)^((y-1)/y);


% --- Afterburner core exhaust velocity ---
if T_0_6_k<T_e_ab_k
V_e_core_ab_k=0;
else
V_e_core_ab_k=(sqrt(2*Cp*(T_0_6_k - T_e_ab_k)));
end

if T_0_6_k_max<T_e_ab_k_max
V_e_core_ab_k_max=0;
else
V_e_core_ab_k_max=(sqrt(2*Cp*(T_0_6_k_max - T_e_ab_k_max)));
end

if T_0_6_k_sl<T_e_ab_k_sl
V_e_core_ab_k_sl=0;
else
V_e_core_ab_k_sl=(sqrt(2*Cp*(T_0_6_k_sl - T_e_ab_k_sl)));
end

f_6_k_sl=(T_0_6_k_sl-T_0_5_sl)*(1+f_sl)*Cp/(eta_ab*E_sp);
f_6_k_max=(T_0_6_k_max-T_0_5)*(1+f)*Cp/(eta_ab*E_sp);


% --- Afterburner core exhaust velocity ---
if V_e_core_ab_k<1 && V_e_core_ab_k>-1
   V_e_core_ab_k=0;
end

if V_e_core_ab_k_sl<1 && V_e_core_ab_k_sl>-1
   V_e_core_ab_k_sl=0;
end

% --- Total fuel mass flow with afterburner ---
m_f_ab_k=((n_ab*(ratio_fo-f))+f)*m_core;
m_f_ab_k_max=(f_6_k_max+f)*m_core;
m_f_ab_k_sl=(f_6_k_sl+f_sl)*m_core_sl;

% --- Thrust including afterburner ---
if BR>0
    T_k = (m_core + m_f_ab_k) * (V_e_core_ab_k)- m_core*V_inf + m_o_f*(V_e_f - V_inf);
    T_k_max = (m_core + m_f_ab_k_max) * (V_e_core_ab_k_max)- m_core*V_inf + m_o_f*(V_e_f - V_inf);
    T_k_sl = (m_core_sl + m_f_ab_k_sl) * (V_e_core_ab_k_sl)- m_core_sl*V_i_core_sl + m_o_f_sl*(V_e_f_sl - V_e_f_i_sl); 
else    
    T_k = (m_core + m_f_ab_k) * (V_e_core_ab_k)-m_core*V_inf;
    T_k_max = (m_core + m_f_ab_k_max) * (V_e_core_ab_k_max)-m_core*V_inf;
    T_k_sl = (m_core_sl + m_f_ab_k_sl) * (V_e_core_ab_k_sl)-m_core_sl*V_i_core_sl; 
end

if T_k<D/n    
    n_ab=n_ab+(n_ab_high-n_ab_low)/2^kk;    
else
    
    n_ab=n_ab-(n_ab_high-n_ab_low)/2^kk;    
end

end   

end








n_ab_Dmax=0;
if (Th_Dmax+1)<D_max/n && Afterburner==1
   
n_ab_Dmax=(n_ab_low+n_ab_high)/2;
for kk_Dmax=2:kk_Dmax_max
    
% --- Afterburner fuel-air ratio ---
f_6_k_Dmax=n_ab_Dmax*(ratio_fo-f_Dmax);
% --- Afterburner exit total temperature ---
T_0_6_k_Dmax = T_0_5_Dmax + eta_ab*f_6_k_Dmax*E_sp/((1+f_Dmax)*Cp);
T_0_6_k_sl_Dmax = T_0_6_k_Dmax;

% --- Scramjet exit Mach number from total-temperature relation ---
if T_0_6_k_Dmax>T_0_6_high && (Scramjet==1 || Scramjet==3)
M_6_Dmax=(2/(y-1)*(T_0_6_k_Dmax/T_0_6_high-1))^.5;
else
M_6_Dmax=0;
end


% --- Afterburner total pressure ---
P_0_6_k_Dmax=P_0_5_Dmax;
% --- Afterburner nozzle exit temperature ---
T_e_ab_k_Dmax=T_0_6_k_Dmax*(p_inf/P_0_6_k_Dmax)^((y-1)/y);

if T_0_6_k_Dmax>T_0_6_high && (Scramjet==1 || Scramjet==3)
M_6_Dmax=(2/(y-1)*(T_0_6_k_Dmax/T_0_6_high-1))^.5;
else
M_6_Dmax=0;
end


% --- Afterburner core exhaust velocity ---
if T_0_6_k_Dmax<T_e_ab_k_Dmax
V_e_core_ab_k_Dmax=0;
else
V_e_core_ab_k_Dmax=(sqrt(2*Cp*(T_0_6_k_Dmax - T_e_ab_k_Dmax)));
end

% --- Afterburner core exhaust velocity ---
if V_e_core_ab_k_Dmax<1 && V_e_core_ab_k_Dmax>-1
   V_e_core_ab_k_Dmax=0;
end


% --- Total fuel mass flow with afterburner ---
m_f_ab_k_Dmax=((n_ab_Dmax*(ratio_fo-f_Dmax))+f_Dmax)*m_core;


% --- Thrust including afterburner ---
if BR>0
    T_k_Dmax = (m_core + m_f_ab_k_Dmax) * (V_e_core_ab_k_Dmax)- m_core*V_inf + m_o_f*(V_e_f_Dmax - V_inf);
else    
    T_k_Dmax = (m_core + m_f_ab_k_Dmax) * (V_e_core_ab_k_Dmax)-m_core*V_inf;
end

if T_k_Dmax<D_max/n    
    n_ab_Dmax=n_ab_Dmax+(n_ab_high-n_ab_low)/2^kk_Dmax;    
else
    
    n_ab_Dmax=n_ab_Dmax-(n_ab_high-n_ab_low)/2^kk_Dmax;    
end

end   

end










% --- Inverse Thrust-specific fuel consumption ---
if (Th+1)<D/n && Afterburner==1
    TSFC_k=T_k/m_f_ab_k;    
else
    TSFC_k=Th/m_f;
end

if (Th_Dmax+1)<D_max/n && Afterburner==1
    TSFC_k_Dmax=T_k_Dmax/m_f_ab_k_Dmax;    
else
    TSFC_k_Dmax=Th_Dmax/m_f_Dmax;
end

if Afterburner==1 && type==1
    m_f_m=m_f_ab_k;
    m_f_m_Dmax=m_f_ab_k_Dmax;
else
    m_f_m=m_f;
    m_f_m_Dmax=m_f_Dmax;
end
  
if (Th+1)<D/n && type>0 && Afterburner==1

% --- Thrust specific fuel consumption ---

if D/n<(T_k+1)
TFC=1/TSFC_k;
else
TFC=1/0;
end


elseif type==0 && Afterburner==1  
    
TFC=(m_f_ab_k_e)/(D/n);

elseif type==0 && Afterburner==0

if D/n<(Th_e+1)
    
TFC=(m_f_e)/(D/n);

else

TFC=1/0;

end

else
    
if D/n<(Th+1)
 
TFC=(m_f)/(Th);

else
 
TFC=1/0;

end
        
end


if (Th_Dmax+1)<D_max/n && type>0

% --- Thrust specific fuel consumption ---
if Afterburner==1
if D_max/n<(T_k_Dmax+1)
TFC_Dmax=1/TSFC_k_Dmax;    
else
TFC_Dmax=1/0;
end

else
    
if D_max/n<(Th_Dmax+1)
TFC_Dmax=1/TFC;
else
TFC_Dmax=1/0;

end

end

end

% % --- Empirical engine-weight correlation ---
% if Scramjet==0 || Scramjet==3
%     if Afterburner==1
%         if BR>0
%    W_eng=(.084*(((T_k_sl*0.224809)^1.1)*exp(-.045*BR))/.224809)*n;
%         else
%    W_eng=(.063*(T_k_sl*.224809)^1.1)/.224809*n;
%         end
%     else
%         if BR>0
%    W_eng=(.084*(((Th_sl*0.224809)^1.1)*exp(-.045*BR))/.224809)*n;
% %    W_eng=W_eng_e*n;
%         else
%    W_eng=(.063*(Th_sl*.224809)^1.1)/.224809*n;
% %    W_eng=W_eng_e*n;
%         end
%     end
% elseif Scramjet==2
%    TWR=4.5;
%    k_r=900;
% %    W_eng=k_r*A_inlet*g*n;
% %    W_eng=1.019*D;
%    W_eng=(.043*(D*.225/n)^1.05*M^.35)/.225*n;
% else
%    TWR=3.5;
%    k_r=700;
% %    W_eng=k_r*A_inlet*g*n;
%    W_eng=(.043*(D*.225/n)^1.05*M^1.03)/.225*n;
% end
% 
% if type==0
%    W_eng=W_eng_e*n;
% end

if M>2
if TPS==1
% --- Conductive heat-transfer flux through TPS ---
q_cond=(Tw/1.8-T_sl)/(t_ins/k_insulation+t_tit/k_titanium);
% --- Fuel temperature-rise energy balance ---
T_fuel_B=(q_cond*S_cooling+m_f_m*n*Cp_f*T_sl)/(m_f_m*n*Cp_f);
end
end


% ==================== STRUCTURAL PARAMETERS ====================
% --- Design/dive speed ---
V_d=1.25*M*a;
% --- Fuselage pressurization weight correlation ---
W_press=(11.9*((pi*(D_f*3.28)^2*L*3.28*(p_cabin-p_inf)*0.0208854)/478802)^.271)/.225;
% --- Doors weight ---
W_doors=(.051*(N_doors*S_door*3.28^2)^1.2+.11*(N_cargo_doors*S_cargo_door*3.28^2)^1.15)/.225;
% --- Systems routing ---
W_sys_routing=(K_sys*(L*3.28*D_f*3.28))/.225;
% --- Floors ---
W_floors=(S_floor_main*3.28^2*w_main*0.0208854+S_floor_upper*3.28^2*w_upper*0.0208854)/.225;
% --- Fuselage structural weight correlation ---
if type2==1
W_fus=(.052*(W_cc_q(q-1)*.225*N_z)^.177*(.5*rho*V_inf^2)^.241*(S_fus*3.28^2)^1.086*(L/D_f)^-.072)/.225+W_sys_routing+W_floors+W_doors+W_press;
elseif type2==0
W_fus=(.328*(W_cc_q(q-1)*.225*N_z)^.5*(L/.3048)^.25*(S_fus*3.28^2)^.302*(L/D_f)^.1)/.225+W_sys_routing+W_floors+W_doors+W_press;    
else
W_fus=(.499*W_cc_q(q-1)^.35*.225*N_z^.25*(L/.3048)^.5*(D_f/.3048)^.849*(D_f*3.28)^.685)/.225+W_sys_routing+W_floors+W_doors+W_press;       
end
% --- Landing-gear weight correlation ---
W_gear=(K_gr*(W_l*.225*N_z)^a_g*L_s^b_g)/.225;
% --- Wing structural weight correlation ---
% --- Landing-gear weight correlation ---
W_gear=(K_gr*(W_l*.225*N_z)^a_g*L_s^b_g)/.225;
%% ---- metric -> Raymer English ----
N2lb = 1/4.44822;  lb2N = 4.44822;
m2ft = 1/0.3048;   m2_to_ft2 = m2ft^2;

W_dg_lb  = W_cc_q(q-1) * N2lb;
W_l_lb   = W_l  * N2lb;
S_w_ft   = S(q) * m2_to_ft2;
S_csw_ft = S_csw* m2_to_ft2;
S_ht_ft  = S_ht * m2_to_ft2;
S_vt_ft  = S_vt * m2_to_ft2;
S_cs_ft  = S_cs * m2_to_ft2;
L_ft     = L    * m2ft;
D_ft     = D_f  * m2ft;
W_ft     = W_fuse * m2ft;
L_t_ft   = L_t  * m2ft;
F_w_ft   = F_w  * m2ft;
B_h_ft   = B_h  * m2ft;
L_m_in   = L_m  / 0.0254;
L_n_in   = L_n  / 0.0254;
L_m_ft   = L_m  * m2ft;
L_n_ft   = L_n  * m2ft;
D_e_ft   = D_e  * m2ft;
L_d_ft   = L_d  * m2ft;
L_tp_ft  = L_tp * m2ft;
L_ec_ft  = L_ec * m2ft;

if Afterburner==0
T_lb     = Th_dry * N2lb;
else
T_lb     = Th_wet * N2lb;
end

W_en_lb  = W_eng_e * N2lb;
W_uav_lb = W_uav * N2lb;
V_t_gal  = V_t  * 264.172;
S_f_wet_ft = pi * D_ft * L_ft;
Q_psf    = 0.5*rho*V_inf^2 * 0.0208854;

if type2 == 1
    % GA 15.46, 15.49-15.58
    W_wing_lb = 0.036*S_w_ft^0.758*W_dg_lb^0.0035 ...
        *(AR/cos(sweep_q)^2)^0.6*Q_psf^0.006*max(lambda,0.01)^0.04 ...
        *(100*tdc/cos(sweep_q))^(-0.3)*(N_z*W_dg_lb)^0.49;
    W_fus_lb  = 0.052*S_f_wet_ft^1.086*(N_z*W_dg_lb)^0.177 ...
        *L_t_ft^(-0.051)*(L_ft/D_ft)^(-0.072)*Q_psf^0.241;
    W_gear_lb = 0.095*(N_l*W_l_lb)^0.768*(L_m_in/12)^0.409 ...
              + 0.125*(N_l*W_l_lb)^0.566*(L_n_in/12)^0.845;
    W_HT_lb=0.016*(N_z*W_dg_lb)^.414*Q_psf^.168*S_ht_ft^.896*(100*tdc/cos(sweep))^-.12;
    W_VT_lb   = 0.073*(1+.2*H_t_Hv)^0.225*W_dg_lb^0.376*N_z^0.376*Q_psf^.122 ...
    *S_vt_ft^0.873*(100*tdc/cos(sweep))^-.49;
    W_tail_lb = W_HT_lb+W_VT_lb;
    W_av_lb   = 2.117*W_uav_lb^0.933;
    W_ac_lb   = 0.265*W_dg_lb^0.52*Np^0.68*W_av_lb^0.17*M^0.08;
    b_ft      = sqrt(S_w_ft*AR);
    W_fc_lb   = 0.053*L_ft^1.536*b_ft^0.371*(N_z*W_dg_lb*1e-4)^0.80;
    W_fs_lb   = 2.49*V_t_gal^0.726*(1/(1+Vi_Vt))^0.363*N_t^0.242*N_en^0.157;
    W_ein_lb  = 2.575*W_en_lb^0.922*N_en; 
    
    % ===== GA 15.55-15.58 (15.52-54 already in your block) =====
    W_hyd_lb  = 0.001 * W_dg_lb;                                          % 15.55
%     W_av_lb   = 2.117 * W_uav_lb^0.933;                                   % 15.57
    W_elec_lb = 12.57 * (W_fs_lb + W_av_lb)^0.51;                         % 15.56
    W_furn_lb = 0;                                                        % no separate GA cabin eq
    W_inst_lb = 0;
    W_apu_lb  = 0;
    W_misc_lb = 0;
    
    W_nac_lb = 0;
    
    W_hg=0;


elseif type2 == 0
%     transport 15.25-15.42
    W_wing_lb = 0.0051*(W_dg_lb*N_z)^0.557*S_w_ft^0.649*AR^0.5 ...
        *tdc^(-0.4)*(1+lambda)^0.1*(cos(sweep_q))^(-1.0)*S_csw_ft^0.1;
    W_fus_lb  = 0.3280*K_door*K_Lg*(W_dg_lb*N_z)^0.5*L_ft^0.25 ...
        *S_f_wet_ft^0.302*(1+K_ws)^0.04*(L_ft/D_ft)^0.10;
    Vstall_kts = V_stall_sl*1.94384;
    W_gear_lb = 0.0106*K_mp*W_l_lb^0.888*N_l^0.25*L_m_ft^0.4 ...
        *N_mw^0.321*max(N_mss,1)^(-0.5)*Vstall_kts^0.1 ...
        + 0.032*K_np*W_l_lb^0.646*N_l^0.2*L_n_ft^0.5*N_nw^0.45;
    W_HT_lb   = 0.0379*K_uht*(1+F_w_ft/B_h_ft)^(-0.25)*W_dg_lb^0.639 ...
        *N_z^0.10*S_ht_ft^0.75*L_t_ft^(-1.0)*(cos(sweep_q))^(-1.0);
    W_VT_lb   = 0.0026*(1+H_t_Hv)^0.225*W_dg_lb^0.556*N_z^0.536 ...
        *L_t_ft^(-0.5)*S_vt_ft^0.5;
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_av_lb   = 1.73*W_uav_lb^0.983;
    Vpr_ft3   = (pi*D_ft^2/4)*L_ft;
    W_ac_lb   = 62.36*Np^0.25*(Vpr_ft3/1000)^0.604*W_uav_lb^0.10;
    W_fc_lb   = 145.9*Nf^0.554*(1+Nm/max(Nf,1))^(-1)*S_cs_ft^0.20;
    W_fs_lb   = 2.405*V_t_gal^0.606*(1+Vi_Vt)^(-1)*(1+Vp_Vt)*N_t^0.5;
    W_ein_lb  = N_en*W_en_lb*1.3;
    
%     furnishings (15.41)  — this is the cabin
    W_furn_lb = 0.0577 * Np^0.1 * (W_dg_lb)^0.393 * S_f_wet_ft^0.75;

%     hydraulics (15.38)
    W_hyd_lb  = 0.2673 * Nf * (L_ft + B_h_ft)^0.937;

%     electrical (15.39)  Rkva ~ 60 kVA class
    W_elec_lb = 7.291 * 60^0.782 * L_ft^0.346 * N_en^0.10;

%     instruments (15.37)
    W_inst_lb = 4.509 * 1.0 * 1.0 * N_c^0.341 * N_c * (L_ft + B_h_ft)^0.5;

%     APU installed (15.36)  — ~200 kg uninstalled class
    W_apu_lb  = 2.2 * 440;

%     anti-ice + handling (15.43-15.44)
    W_misc_lb = 0.002*W_dg_lb + 3e-4*W_dg_lb;
    
    W_ec_lb  = 2.331 * W_en_lb^0.901 * K_p * K_tr;     % per nacelle [lb]

    W_nac_lb = 0.6724 * K_ng * N_Lt_ft^0.10 * N_w_ft^0.294 ...
         * N_z^0.119 * W_ec_lb^0.611 * N_e^0.984 * S_n_ft^0.224;
     
    W_hg=3E-4*W_cc_q(q-1);


else
%     fighter/delta 15.1-15.22  (XB-70: type2=2)
    W_wing_lb = 0.0103*K_dw*K_vs*(W_dg_lb*N_z)^0.5*S_w_ft^0.622*AR^0.785 ...
        *tdc^(-0.4)*(1+lambda)^0.05*(cos(sweep_q))^(-1.0)*S_csw_ft^0.04;
    W_fus_lb  = 0.499*K_dwf*W_dg_lb^0.35*N_z^0.25*L_ft^0.5*D_ft^0.849*W_ft^0.685;
    W_MLG_lb  = K_cb*K_tpg*(W_l_lb*N_l)^0.25*L_m_in^0.973;
    W_NLG_lb  = (W_l_lb*N_l)^0.290*L_n_in^0.5*N_nw^0.525;
    W_gear_lb = W_MLG_lb + W_NLG_lb;
    W_HT_lb   = 3.316*(1+F_w_ft/B_h_ft)^(-2.0)*((W_dg_lb*N_z)/1000)^0.260*S_ht_ft^0.806;
    W_VT_lb   = 0.452*K_rht*(1+H_t_Hv)^0.5*(W_dg_lb*N_z)^0.488*S_vt_ft^0.718 ...
        *M^0.341*L_t_ft^(-1.0)*(1+Sr_Svt)^0.348*A_vt^0.223 ...
        *(1+lambda)^0.25*(cos(sweep_vt))^(-0.323);
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*Ls_Ld^(-0.373)*D_e_ft;
    W_mnt_lb  = 0.013*N_en^0.795*T_lb^0.579*N_z;
    W_esc_lb  = 0.01*W_en_lb^0.717*N_en*N_z;
    W_tp_lb   = 3.5*D_e_ft*L_tp_ft*N_en;
    W_ein_lb  = N_en*W_en_lb + W_inl_lb + W_mnt_lb + W_esc_lb + W_tp_lb;
    W_fs_lb   = 7.45*V_t_gal^0.47*(1+Vi_Vt)^(-0.095)*(1+Vp_Vt) ...
        *N_t^0.066*N_en^0.052*(T_lb*SFC/1000)^0.249;
    W_fc_lb   = 36.28*M^0.003*S_cs_ft^0.489*N_s^0.484*N_c^0.127;
 
%     ===== fighter 15.13-15.15, 15.18-15.24 =====
    T_e_lb    = T_lb / max(N_en,1);                                       % thrust per engine
    W_oil_lb  = 37.82 * N_en^1.023;                                       % 15.13
    W_ectrl_lb= 10.5 * N_en^1.008 * L_ec_ft^0.222;                        % 15.14
    W_strt_lb = 0.025 * T_e_lb^0.760 * N_en^0.72;                         % 15.15
    W_inst_lb = 8.0 + 36.37*N_en^0.676*N_t^0.237 + 26.4*(1+N_ci)^1.356;   % 15.18
    W_hyd_lb  = 37.23 * K_vsh * N_u^0.664;   
    L_a_ft    = L/.3048;
    N_gen     = N_en;
    W_elec_lb = 172.2 * K_mc * R_kva^0.152 * N_c^0.10 * L_a_ft^0.10 ...
                * N_gen^0.091;                                            % 15.20
    W_av_lb   = 2.117 * W_uav_lb^0.933;                                   % 15.21
    W_furn_lb = 217.6 * N_c;                                              % 15.22 crew station
    W_ac_lb   = 201.6 * ((W_uav_lb + 200*N_c)/1000)^0.735;                % 15.23
    W_misc_lb = 3.2e-4 * W_dg_lb;                                         % 15.24 handling
    W_apu_lb  = 0;
    W_ein_lb  = W_ein_lb + W_oil_lb + W_ectrl_lb + W_strt_lb;
    W_nac_lb = 0;
    W_hg=0;
    
end

%% ---- fudge factors (keep Raymer exponents; scale constants) ----
switch type2
    case 0
        if M<1
        K_wing = 1.51; K_fuse = 1.51; K_gear = 1.51;
        K_tail = 1.51; K_prop = 1.51; K_fuel = 1.51;
        K_sys  = 1.51; K_furn = 1.51; K_oew  = 1.51;
        else
%         K_wing = 1.15;
%         K_fuse = 1.15;
%         K_gear = 1.10;
%         K_tail = 1.10;
%         K_prop = 1.00;
%         K_fuel = 1.05;
%         K_sys  = 1.10;
%         K_furn = 1.00;
%         K_oew  = 1.05;
%         
%         K_wing = 1.20;    % High structural/aeroelastic loads, slender delta
%         K_fuse = 1.15;    % Supersonic pressure vessel + thermal structure
%         K_gear = 1.10;    % Heavy landing gear for high MTOW
%         K_tail = 1.10;    % Supersonic stability/control requirements
%         K_prop = 1.00;    % No propeller
%         K_fuel = 1.10;    % Extensive fuel system/tanks/transfer system
%         K_sys  = 1.20;    % Complex SST systems
%         K_furn = 1.05;    % Passenger furnishings/interior
%         K_oew  = 1.05;    % Overall OEW allowance
        
        % Concorde / Tu-144  (type2==0 transport eqs)
        K_wing = 1.222;   % slender delta; K_dw=0.768 already in 15.1
        K_fuse = 1.222;   % long intake tunnels + ogive, not a tube airliner
        K_gear = 1.222;   % 8-wheel main trucks, high W_l
        K_tail = 1.222;   % S_ht ~ 0; do not inflate a missing tail
        K_prop = 1.222;   % four Olympus + 2-D ramps / reversers
        K_fuel = 1.222;   % 15.16 under-runs ~96 t of integral fuel
        K_sys  = 1.222;   % hyd + elec + intake control
        K_furn = 1.222;   % 100 pax cabin; not 737 K_furn=2.5
        K_oew  = 1.222;   % leftover ops / unusable / heat structure
        end
    case 1
        if M<1
        % light jet (Citation CJ3 class): pressurized, T-tail, retractable
    % only if you are running the CJ3 — comment the piston block out
    K_wing = 1.294;   % laminar wing + fuel in wing
    K_fuse = 1.294;   % 15.49 without the pressurization add-on under-runs
    K_gear = 1.294;   % retractable
    K_tail = 1.294;   % T-tail (H_t_Hv=1 already helps if coded)
    K_prop = 1.294;   % use book engine weight
    K_fuel = 1.294;   % 15.54 is a light-airplane fuel system
    K_sys  = 1.294;   % hyd 15.55 is ~14 lb on a CJ3; real system is larger
    K_furn = 1.294;   % interior / pressurization leftover
    K_oew  = 1.294;
        else
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        end
    otherwise
        if M<3
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
            
        if skin==1
            K_mat=1;
        elseif skin==2
            K_mat=1.4;
        else
            K_mat=1.5;
        end
        K_wing = K_mat;   % honeycomb + high-temp skins; 15.1 under-runs
        K_fuse = K_mat;   % long inlet tunnels + heat structure
        K_gear = K_mat;   % high W_l, bicycle/truck gear not in 15.4–15.5
        K_tail = K_mat;   % canards / twin fins if S_ht, S_vt are small
        K_prop = K_mat;   % 15.10–15.15 inlet+install light vs 6-engine pack
        K_fuel = K_mat;   % 15.16 vs ~300000 lb fuel system / tanks
        K_sys  = K_mat;   % hyd + elec + ECS at Mach 3
        K_furn = K_mat;   % no airliner cabin; leave 15.22 as-is
        K_oew  = K_mat;   % leftover ops / unusable / TPS not in Ch. 15
        end
end

W_wing     = K_wing * W_wing_lb * lb2N;
W_fus      = K_fuse * W_fus_lb  * lb2N;
W_gear     = K_gear * W_gear_lb * lb2N;
W_tail     = K_tail * W_tail_lb * lb2N;
W_eng_inst = K_prop * W_ein_lb  * lb2N;
W_nac      = K_prop * W_nac_lb  * lb2N;
W_fuel_sys = K_fuel * W_fs_lb   * lb2N;
W_fc       = K_sys  * W_fc_lb   * lb2N;
W_av       = K_sys  * W_av_lb   * lb2N;
W_ac       = K_sys  * W_ac_lb   * lb2N;
W_hyd      = K_sys  * W_hyd_lb  * lb2N;
W_elec     = K_sys  * W_elec_lb * lb2N;
W_inst     = K_sys  * W_inst_lb * lb2N;
W_apu      = K_sys  * W_apu_lb  * lb2N;
W_furn     = K_furn * W_furn_lb * lb2N;
W_misc     = W_misc_lb * lb2N;


W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg;

if M>2
if TPS==1
    
W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_tps;
end
end

%% empty (no payload)
% W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys + W_fc + W_av + W_ac;
% --- Systems weight correlation ---
% W_sys =.10*W_e(q-1);
% % --- tail weight correlation ---
% W_tail=0.04 * W_wing;

% --- Empty-weight buildup ---
% if type2==1
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% end
% elseif type2==0
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% end
% else
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_fc+W_sys;
% end
% end
    

% --- Descent phase ---
% --- Delta altitude for iteration
dalt_dd=alt/dd_max;
% --- Initial value for descent weight
W_dd(1)=W_l;
% --- Initial value for descent distance
x_dd(1)=0;
% --- Indicated airspeed ---
V_a_c=V_inf*(rho/rho_sl)^.5;
% --- Initial value for velocity
V_dd(1)=V_a_c;
for dd = 2:dd_max+1
    % --- Altitude ---
    alt_dd=dalt_dd*dd;
    
% --- Atmospheric temperature model (piecewise approximation) ---
if alt_dd<=11000
    T_dd=288.15-.00649*alt_dd;
elseif alt_dd>11000 && alt_dd<25000
    T_dd=216.65;
else
    T_dd=141.89+.00299*alt_dd;
end

if alt_dd<=11000
P_dd=101290*(T_dd/288.08)^5.256;
elseif alt_dd>11000 && alt_dd<25000
P_dd=22650*exp(1.73-.000157*alt_dd);
else
P_dd=2488*(T_dd/216.6)^-11.388;
end

rho_dd=P_dd/(R*T_dd);

a_dd=(y*R*T_dd)^.5;
V_inf_dd=M*a_dd;


% --- Descent speed under constant dynamic pressure ---
    V_dd(dd)=V_a_c*(rho_sl/rho_dd)^.5;
% --- Descent time increment ---
    dtime_dd=dalt_dd/rate_of_descent;
% --- descent deceleration ---
    acc_dd=(V_dd(dd)-V_dd(dd-1))/dtime_dd;
% --- Descent flight-path angle ---
    theta_dd_B=asin(rate_of_descent/V_dd(dd));
% --- Mach number
    M_dd=V_inf_dd/a_dd;  
% --- Drag
if M_dd<1 && M<1
    D_dd=(1-M^2)^.5/(1-M_dd^2)^.5*D;
elseif M_dd<1 && M>1
    D_dd=(1-M^2)^.5/(M_dd^2-1)^.5*D;
elseif M_dd>1 && M<1
    D_dd=(M^2-1)^.5/(1-M_dd^2)^.5*D;
else
    D_dd=(M^2-1)^.5/(M_dd^2-1)^.5*D;
end
% --- Available thrust at descent altitude ---
    Th_dd=D_dd+W_dd(dd-1)*sin(theta_dd_B)+W_dd(dd-1)/g*acc_dd;
% --- Horizontal velocity ---
    Vx_dd=V_dd(dd)*cos(theta_dd_B);
% --- Horizontal descent distance increment ---
    dx_dd=dtime_dd*Vx_dd;
% --- Horizontal descent total distance ---
    x_dd(dd)=dx_dd+x_dd(dd-1);
% --- Thrust specific fuel consumption
    TSFC_dd(dd)=(T_dd/T)^.5*TFC;
% --- Overall efficiency ---
    no_dd=V_dd(dd)/(TSFC_dd(dd)*E_sp);
% --- Descent fuel weight increment ---
    dW_dd=(Th_dd*V_dd(dd)/no_dd)/E_sp*g*dtime_dd;
% --- Total descent fuel weight ---
    W_dd(dd)=W_dd(dd-1)+dW_dd;     
end


% --- Cruise fuel from Breguet range relation ---
W_f(q)=((exp(Range_cruise*g*TFC/(M*a*LdD(q))))-1)*(W_dd(dd));
% W_f(q)=((exp(Range_cruise*g*.000033/(M*a*7.5)))-1)*(W_dd(dd));
W_dd_q(q)=W_dd(dd);
TOGW_i(q)=W_f(q)+W_dd(dd);

% --- climb phase ---
% --- Delta altitude for iteration
dalt_cc=alt/cc_max;
% --- Initial value for climb weight
W_cc(1)=TOGW_i(q);
% --- Initial value for climb distance
x_cc(1)=0;
% --- Initial value for velocity
V_cc(1)=V_a_c*(rho_sl/rho)^.5;
for cc = 2:cc_max+1
% --- Altitude ---
    alt_cc=dalt_cc*(cc_max-cc+2);
    
% --- Atmospheric temperature model (piecewise approximation) ---
if alt_cc<=11000
    T_cc=288.15-.00649*alt_cc;
elseif alt_cc>11000 && alt_cc<25000
    T_cc=216.65;
else
    T_cc=141.89+.00299*alt_cc;
end


if alt<=11000
P_cc=101290*(T_cc/288.08)^5.256;
elseif alt>11000 && alt<25000
P_cc=22650*exp(1.73-.000157*alt);
else
P_cc=2488*(T_cc/216.6)^-11.388;
end

rho_cc=P_cc/(R*T_cc);
    
    a_cc=(y*R*T_cc)^.5;
   
% --- Climb speed under constant dynamic pressure ---
    V_cc(cc)=V_a_c*(rho_sl/rho_cc)^.5;
% --- Climb time increment ---
    dtime_cc=dalt_cc/rate_of_climb;
% --- climb acceleration ---
    acc_cc=(V_cc(cc)-V_cc(cc-1))/dtime_cc;
% --- Climb flight-path angle ---
    theta_cc_B=asin(rate_of_climb/V_cc(cc));
    
    % --- Mach number
M_cc=V_inf_dd/a_cc;  
% --- Drag
if M_cc<1 && M<1
    D_cc=(1-M^2)^.5/(1-M_cc^2)^.5*D;
elseif M_dd<1 && M>1
    D_cc=(1-M^2)^.5/(M_cc^2-1)^.5*D;
elseif M_dd>1 && M<1
    D_cc=(M^2-1)^.5/(1-M_cc^2)^.5*D;
else
    D_cc=(M^2-1)^.5/(M_cc^2-1)^.5*D;
end

% --- Thrust required for climb ---
    Th_b_cc=D_cc+W_cc(cc-1)*sin(theta_cc_B)+W_cc(cc-1)/g*acc_cc;
% --- Horizontal velocity ---
    Vx_cc=V_cc(cc)*cos(theta_cc_B);
% --- Horizontal climb distance increment ---
    dx_cc=dtime_cc*Vx_cc; 
% --- Horizontal climb total distance ---
    x_cc(cc)=dx_cc+x_cc(cc-1);
% --- Thrust specific fuel consumption
    TSFC_cc(cc)=(T_cc/T)^.5*TFC;
% --- Overall efficiency ---
    no_cc=V_cc(cc)/(TSFC_cc(cc)*E_sp);
% --- Climb fuel/weight increment ---
    dm_cc(cc)=((Th_b_cc*V_cc(cc)/no_cc)/E_sp)*g*dtime_cc; 
% --- Total climb fuel/weight ---
    W_cc(cc)=W_cc(cc-1)+dm_cc(cc);       
end

W_cc_j(j)=W_cc(cc);

W_cc_j_min=min(W_cc_j(2:end));

if W_cc_j(j)==W_cc_j_min
    BR_j=BR;
    W_cc_j_min=W_cc_j(j);
    D_j=D;
    TIT_j=TIT_d;
    Th_j=Th;
    FPR_j=FPR;
    W_l_j=W_l;
    S_j=S(q);
    LdD_j=LdD(q);
    c_l_j=C_l;
    c_r_j=c_r;
    c_t_j=c_t;
    b_j=b;
    sweep_j=sweep;
    if Afterburner==0
    Th_sl_j=Th_sl;
    T_max_j=Th_max;
    T_Dmax_j=Th_Dmax;
    else
    Th_sl_j=T_k_sl;
    T_max_j=T_k_max;
    T_Dmax_j=T_k_Dmax;
    end
    TOGW_i_j=TOGW_i(q);
    W_dd_q_j=W_dd_q(q);
    rho_j=rho;
    V_inf_j=V_inf;
    C_d_0_j=C_d_0;
    K_j=1/(pi*AR*e);
    if Afterburner==1
    T_0_6_k_Dmax_min_j=T_0_6_k_Dmax;
    Th_j=T_k;
    M_6_j=M_6;
    M_6_j_Dmax=M_6_Dmax;
    else
    Th_j=Th;
    end
end

end

W_cc_i(i)=W_cc_j_min;

W_cc_i_min=min(W_cc_i(2:end));

if W_cc_i(i)==W_cc_i_min
    CPR_ii=CPR;
    BR_ii=BR_j;
    TIT_ii=TIT_j;
    W_cc_i_min=W_cc_i(i);
    D_k=D_j;
    Th_ii=Th_j;
    FPR_ii=FPR_j;
    W_l_k=W_l_j;
    S_k=S_j;
    Th_sl_k=Th_sl_j;
    TOGW_i_k=TOGW_i_j;
    TOGW_i_kk=W_dd_q_j;
    rho_k=rho_j;
    V_inf_k=V_inf_j;
    C_d_0_k=C_d_0_j;
    K_k=K_j;
    LdD_k=LdD_j;
    c_l_k=c_l_j;
    c_r_k=c_r_j;
    c_t_k=c_t_j;
    b_k=b_j;
    sweep_k=sweep_j;
    if Afterburner==1
    T_0_6_k_Dmax_min_i=T_0_6_k_Dmax_min_j;
    Th_k=Th_j;
    M_6_k=M_6_j;
    M_6_k_Dmax=M_6_j_Dmax;
    T_max_k=T_max_j;
    T_Dmax_k=T_Dmax_j;
    else
    Th_k=Th_j;
    T_max_k=T_max_j;
    T_Dmax_k=T_Dmax_j;
    end
end

end

W_cc_q(q)=W_cc_i_min;







else
    
    
    
    
    
    
    
 % --- Inlet total temperature ---
T_0_2=T*(1+(y-1)/2*M^2);
% --- Inlet total pressure including inlet efficiency ---
P_0_2=eta_inlet*p_inf*(1+(y-1)/2*M^2)^(y/(y-1));
% --- Ideal compressor exit total temperature ---
T_0_3s_e = (T_0_2 * CPR_e^((y-1)/(y)));
% --- Actual compressor exit total temperature ---
T_0_3_e = T_0_2 + (T_0_3s_e-T_0_2)/eta_c_e;
% --- Compressor specific work ---
W_c_e=(Cp*(T_0_3_e-T_0_2));
% --- Compressor total pressure relations ---
P_0_3_e=P_0_2*CPR_e;
% --- Actual fan exit temperature relations ---
T_0_3_f_e = T_0_2 * (1 + (FPR_e^((y-1)/y)-1)/eta_f_e);
% --- Fan specific work ---
if BR_e>0
    W_f_e=(Cp*(T_0_3_f_e-T_0_2));
else
    W_f_e=0;
end
% --- Main combustor fuel-air ratio ---
if (Scramjet==0 || Scramjet==3) && CPR_e>1
f_e=((Cp*(TIT_e-T_0_3_e))/(eta_b_e*E_sp));
else
f_e=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1
f_e_max=((Cp*(TIT_e-T_0_3_e))/(eta_b_e*E_sp));
else
f_e_max=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1
% --- Total temperature and pressure behind turbine ---
T_0_5s_e=  (TIT_e - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e)*eta_m_e));
T_0_5s_e_max=(TIT_e - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e_max)*eta_m_e));
% --- actual turbine exit temperature ---
T_0_5_e = TIT_e - (TIT_e-T_0_5s_e)/eta_t_e;
T_0_5_e_max = TIT_e - (TIT_e-T_0_5s_e_max)/eta_t_e;
% --- Turbine exit total pressure ---
P_0_5_e = P_0_3_e * (T_0_5s_e/TIT_e)^(y/(y-1));
P_0_5_e_max = P_0_3_e * (T_0_5s_e_max/TIT_e)^(y/(y-1));
else
T_0_5_e=T_0_2;
P_0_5_e=P_0_2;
P_0_5_e_max=P_0_2;
T_0_5_e_max=T_0_2;
end


% --- Nozzle exit temperature ---
T_e_e=T_0_5_e*(p_inf/P_0_5_e)^((y-1)/y);
T_e_e_max=T_0_5_e*(p_inf/P_0_5_e_max)^((y-1)/y);

% --- Core exhaust velocity from energy equation ---
if (Scramjet==0 || Scramjet==3) && CPR_e>1    
    if T_0_5_e<T_e_e
        V_e_core_e=0;
    else
        V_e_core_e = (sqrt(2*Cp*(T_0_5_e - T_e_e)));
    end
else
V_e_core_e = V_inf;
end

if V_e_core_e<1 && V_e_core_e>-1
   V_e_core_e=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1    
    if T_0_5_e_max<T_e_e_max
        V_e_core_e_max=0;
    else
        V_e_core_e_max = (sqrt(2*Cp*(T_0_5_e_max - T_e_e_max)));
    end
else
V_e_core_e_max = V_inf;
end

if V_e_core_e_max<1 && V_e_core_e_max>-1
   V_e_core_e_max=0;
end

if BR_e>0
    V_e_f_e = (sqrt(2*eta_f_e*Cp*(T_0_3_f_e - T)));
else   
    V_e_f_e=0;
end

% --- Core mass-flow rate ---
m_core_e=V_inf*A_inlet_e/(BR_e+1)*rho;
% --- Bypass mass-flow rate ---
m_o_f_e=(V_inf*A_inlet_e*rho*BR_e/(BR_e+1));
% --- Fuel mass-flow rate ---
m_f_e=f_e*m_core_e;
m_f_e_max=f_e*m_core_e;
% --- Main combustor thrust ---
if BR_e>0
    Th_e = (m_core_e + m_f_e) * (V_e_core_e)-m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else
    Th_e = (m_core_e + m_f_e) * (V_e_core_e)-m_core_e*V_inf;
end

if BR_e>0
    Th_e_max = (m_core_e + m_f_e_max) * (V_e_core_e_max)-m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else
    Th_e_max = (m_core_e + m_f_e_max) * (V_e_core_e_max)-m_core_e*V_inf;
end





























if Afterburner==0
% D=600000;
TIT_e_high=TIT_e;
TIT_e_0=(TIT_e_high+TIT_e_low)/2;
TIT_e_0_max=(TIT_e_high+TIT_e_low)/2;
TIT_e_0_Dmax=(TIT_e_high+TIT_e_low)/2;
for ii=2:ii_max
   
% D=50000;

% if BR_e>0
%     FPR_e=((1+(TIT_e_0-(TIT_e_0 - (W_c_e + (1+BR_e)*(Cp*((T_0_2 + (TIT_e_0 - T_0_3_e)/(1+BR_e))-T_0_2)))/(Cp*(1+(Cp*(TIT_e_0-T_0_3_e)+(1+BR_e)*(Cp*((T_0_2 + (TIT_e_0 - T_0_3_e)/(1+BR_e))-T_0_2)))/E_sp)))-(T_0_3_e-T_0_2))/((1+BR_e)*T_0_2))^(y/(y-1)));
% else
%     FPR_e=0;
% end

% FPR_e=1.2;
if (Scramjet==0 || Scramjet==3) && CPR_e>1
f_e_0=((Cp*(TIT_e_0-T_0_3_e))/E_sp);
else
f_e_0=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1
f_e_0_max=((Cp*(TIT_e_high-T_0_3_e))/E_sp);
else
f_e_0_max=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1
f_e_0_Dmax=((Cp*(TIT_e_high-T_0_3_e))/E_sp);
else
f_e_0_Dmax=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1
% --- Total temperature and pressure behind turbine ---
% T_0_5_e=  ((TIT_e_0 - (W_c_e + (BR_e)*W_f_e)/(Cp*(1)*eta_m_e)));
T_0_5_e_0=(TIT_e_0 - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e_0)*eta_m_e));
T_0_5_e_Dmax=(TIT_e_0_Dmax - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e_0_Dmax)*eta_m_e));
T_0_5_e_max=(TIT_e_high - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e_0_max)*eta_m_e));
% --- Isentropic turbine exit temperature ---
T_0_5s_e = TIT_e_0 - (TIT_e_0-T_0_5_e_0)/eta_t_e;
T_0_5s_e_Dmax = TIT_e_0_Dmax - (TIT_e_0_Dmax-T_0_5_e_Dmax)/eta_t_e;
T_0_5s_e_max = TIT_e_high - (TIT_e_high-T_0_5_e_max)/eta_t_e;
% --- Turbine exit total pressure ---
P_0_5_e = P_0_3_e * (T_0_5s_e/TIT_e_0)^(y/(y-1));
P_0_5_e_Dmax = P_0_3_e * (T_0_5s_e_Dmax/TIT_e_0)^(y/(y-1));
P_0_5_e_max = P_0_3_e * (T_0_5s_e_max/TIT_e_high)^(y/(y-1));
else
T_0_5_e_0=T_0_2;
P_0_5_e=P_0_2;
T_0_5_e_max=T_0_2;
P_0_5_e_max=P_0_2;
T_0_5_e_Dmax=T_0_2;
P_0_5_e_Dmax=P_0_2;
end


% --- Nozzle exit temperature ---
T_e_e=T_0_5_e_0*(p_inf/P_0_5_e)^((y-1)/y);
T_e_e_Dmax=T_0_5_e_Dmax*(p_inf/P_0_5_e_Dmax)^((y-1)/y);
T_e_e_max=T_0_5_e_max*(p_inf/P_0_5_e_max)^((y-1)/y);

% --- Core exhaust velocity from energy equation ---
if (Scramjet==0 || Scramjet==3) && CPR_e>1    
    if T_0_5_e_0<T_e_e
        V_e_core_e=0;
    else
        V_e_core_e = (sqrt(2*Cp*(T_0_5_e_0 - T_e_e)));
    end
else
V_e_core_e = V_inf;
end

if V_e_core_e<1 && V_e_core_e>-1
   V_e_core_e=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1    
    if T_0_5_e_max<T_e_e_max
        V_e_core_e_max=0;
    else
        V_e_core_e_max = (sqrt(2*Cp*(T_0_5_e_max - T_e_e_max)));
    end
else
V_e_core_e_max = V_inf;
end

if V_e_core_e<1 && V_e_core_e>-1
   V_e_core_e=0;
end

if V_e_core_e_max<1 && V_e_core_e_max>-1
   V_e_core_e_max=0;
end

if (Scramjet==0 || Scramjet==3) && CPR_e>1    
    if T_0_5_e_Dmax<T_e_e_Dmax
        V_e_core_e_Dmax=0;
    else
        V_e_core_e_Dmax = (sqrt(2*Cp*(T_0_5_e_0 - T_e_e)));
    end
else
V_e_core_e_Dmax = V_inf;
end

if V_e_core_e_Dmax<1 && V_e_core_e_Dmax>-1
   V_e_core_e_Dmax=0;
end


% m_f_e = (D/n + m_core_e* V_inf - m_o_f_e*(V_e_f_e - V_inf)) / (sqrt(2*Cp*(T_0_5_e - T_e_e)))-m_core_e;
% f_e=m_f_e/m_core_e;
% f_e_0=((Cp*(TIT_e_0-T_0_3_e))/E_sp);
% TIT_e=f_e*E_sp/Cp+T_0_3_e;

% --- Core mass-flow rate ---
m_core_e=V_inf*A_inlet_e/(BR_e+1)*rho;
% --- Bypass mass-flow rate ---
m_o_f_e=(V_inf*A_inlet_e*rho*BR_e/(BR_e+1));
% --- Fuel mass-flow rate ---
m_f_e=f_e_0*m_core_e;
m_f_e_Dmax=f_e_0_Dmax*m_core_e;
m_f_e_max=f_e_0_max*m_core_e;
% --- Main combustor thrust ---
if BR_e>0
    Th_e_0 = (m_core_e + m_f_e) * (V_e_core_e)-m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else
    Th_e_0 = (m_core_e + m_f_e) * (V_e_core_e)-m_core_e*V_inf;
end

if BR_e>0
    Th_e_0_Dmax = (m_core_e + m_f_e_Dmax) * (V_e_core_e_Dmax)-m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else
    Th_e_0_Dmax = (m_core_e + m_f_e_Dmax) * (V_e_core_e_Dmax)-m_core_e*V_inf;
end

if BR_e>0
    Th_e_0_max = (m_core_e + m_f_e_max) * (V_e_core_e_max)-m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else
    Th_e_0_max = (m_core_e + m_f_e_max) * (V_e_core_e_max)-m_core_e*V_inf;
end

% TIT_e_0=2273;

if Th_e_0<D/n
    TIT_e_0=TIT_e_0+(TIT_e_high-TIT_e_low)/2^ii;
else
    TIT_e_0=TIT_e_0-(TIT_e_high-TIT_e_low)/2^ii;  
end

if Th_e_0<D_max/n
    TIT_e_0_max=TIT_e_0_max+(TIT_e_high-TIT_e_low)/2^ii;
else
    TIT_e_0_max=TIT_e_0_max-(TIT_e_high-TIT_e_low)/2^ii;  
end

end

end


% --- Afterburner total pressure ---
P_0_6_k_e=P_0_5_e;
P_0_6_k_e_max=P_0_5_e_max;
% --- Afterburner nozzle exit temperature ---
% T_e_ab_k_e=T_0_6_k_e*(p_inf/P_0_6_k_e)^((y-1)/y);
% --- Solving for afterburner fuel mass fraction ---
f_6_k_e_i=(f_6_k_e_i_high+f_6_k_e_i_low)/2;
f_6_k_e_i_Dmax=f_6_k_e_i;
for ff=2:ff_max
    
if BR_e>0
    V_e_core_ab_k_e=(D/n+m_core_e*V_inf- m_o_f_e*(V_e_f_e - V_inf))/(m_core_e*(1+f_e+f_6_k_e_i));
else    
    V_e_core_ab_k_e=(D/n+m_core_e*V_inf)/(m_core_e*(1+f_e+f_6_k_e_i));
end

if BR_e>0
    V_e_core_ab_k_e_Dmax=(D_max/n+m_core_e*V_inf- m_o_f_e*(V_e_f_e - V_inf))/(m_core_e*(1+f_e+f_6_k_e_i_Dmax));
else    
    V_e_core_ab_k_e_Dmax=(D_max/n+m_core_e*V_inf)/(m_core_e*(1+f_e+f_6_k_e_i_Dmax));
end

% --- Existing-engine afterburner total temperature ---
% T_0_6_k_e=V_e_core_ab_k_e^2/(2*Cp)+T_0_6_k_e*(p_inf/P_0_6_k_e)^((y-1)/y);
T_0_6_k_e = (V_e_core_ab_k_e^2) / (2 * Cp * (1 - (p_inf/P_0_6_k_e)^((y-1)/y)));
V_e_core_ab_k_e_max=(T_0_6_high*(2*Cp*(1-(p_inf/P_0_6_k_e_max)^((y-1)/y))))^.5;

if T_0_6_k_e>T_0_6_high && (Scramjet==1 || Scramjet==3)
M_6_e=(2/(y-1)*(T_0_6_k_e/T_0_6_high-1))^.5;
else
M_6_e=0;
end

T_0_6_k_e_Dmax = (V_e_core_ab_k_e_Dmax^2) / (2 * Cp * (1 - (p_inf/P_0_6_k_e)^((y-1)/y)));

if T_0_6_k_e>T_0_6_high && (Scramjet==1 || Scramjet==3)
M_6_Dmax=(2/(y-1)*(T_0_6_k_e_Dmax/T_0_6_high-1))^.5;
else
M_6_Dmax=0;
end

% --- Existing-engine afterburner fuel-air ratio ---
f_6_k_e=(T_0_6_k_e-T_0_5_e)*(1+f_e)*Cp/(eta_ab_e*E_sp);
f_6_k_e_Dmax=(T_0_6_k_e_Dmax-T_0_5_e)*(1+f_e)*Cp/(eta_ab_e*E_sp);
f_6_k_e_max=(T_0_6_high-T_0_5_e_max)*(1+f_e)*Cp/(eta_ab_e*E_sp);

if f_6_k_e>f_6_k_e_i
   f_6_k_e_i=f_6_k_e_i+(f_6_k_e_i_high-f_6_k_e_i_low)/2^ff;
else
   f_6_k_e_i=f_6_k_e_i-(f_6_k_e_i_high-f_6_k_e_i_low)/2^ff;
end

if f_6_k_e_Dmax>f_6_k_e_i
   f_6_k_e_i_Dmax=f_6_k_e_i_Dmax+(f_6_k_e_i_high-f_6_k_e_i_low)/2^ff;
else
   f_6_k_e_i_Dmax=f_6_k_e_i_Dmax-(f_6_k_e_i_high-f_6_k_e_i_low)/2^ff;
end

end

% --- Afterburner core exhaust velocity ---
if V_e_core_ab_k_e<1 && V_e_core_ab_k_e>-1
   V_e_core_ab_k_e=0;
end

% --- Afterburner core exhaust velocity ---
if V_e_core_ab_k_e_Dmax<1 && V_e_core_ab_k_e_Dmax>-1
   V_e_core_ab_k_e=0;
end

% --- Total fuel mass flow with afterburner ---
m_f_ab_k_e=(f_6_k_e+f_e)*m_core_e;   

% --- Total fuel mass flow with afterburner ---
m_f_ab_k_e_max=(f_6_k_e_max+f_e)*m_core_e; 
        
if Afterburner==1
    m_f_m=m_f_ab_k_e;
else
    m_f_m=m_f_e;
end  


% if BR_e>0
%     T_k_e = (m_core_e + m_f_ab_k_e) * (V_e_core_ab_k_e)- m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
% else    
%     T_k_e = (m_core_e + m_f_ab_k_e) * (V_e_core_ab_k_e)-m_core_e*V_inf;
% end


if BR_e>0
    T_k_e_max = (m_core_e + m_f_ab_k_e_max) * (V_e_core_ab_k_e_max)- m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else    
    T_k_e_max = (m_core_e + m_f_ab_k_e_max) * (V_e_core_ab_k_e_max)-m_core_e*V_inf;
end
      

if Afterburner==1
  
TFC=(m_f_ab_k_e)/(D/n);

else
    
if D/n<(Th_e_0+1)
    
TFC=(m_f_e)/(D/n);

else
    
TFC=1/0;

end

end











if type==0 && Afterburner==1  


TFC=(m_f_ab_k_e)/(D/n);

elseif type==0 && Afterburner==0

if D/n<(Th_e+1)
    
TFC=(m_f_e)/(D/n);

else

TFC=1/0;

end

else
    
if D/n<(Th_e+1)
 
TFC=(m_f)/(D/n);

else
 
TFC=1/0;

end
        
end




































% --- Empirical engine-weight correlation ---
if type==0
   W_eng=W_eng_e*n;
end

if M>2
if TPS==1
% --- Conductive heat-transfer flux through TPS ---
q_cond=(Tw/1.8-T_sl)/(t_ins/k_insulation+t_tit/k_titanium);
% --- Fuel temperature-rise energy balance ---
T_fuel_B=(q_cond*S_cooling+m_f_m*n*Cp_f*T_sl)/(m_f_m*n*Cp_f);
end
end


% ==================== STRUCTURAL PARAMETERS ====================
% --- Design/dive speed ---
V_d=1.25*M*a;
% --- Fuselage pressurization weight correlation ---
W_press=11.9*((pi*(D_f-2)^2*L*(p_cabin-p_inf))/478802)^.271;

% --- Fuselage structural weight correlation ---

% W_cc_q(q-1)=4300000;

% --- Fuselage pressurization weight correlation ---
W_press=(11.9*((pi*(D_f*3.28)^2*L*3.28*(p_cabin-p_inf)*0.0208854)/478802)^.271)/.225;
% --- Doors weight ---
W_doors=.051*(N_doors*S_door)^1.2+.11*(N_cargo_doors*S_cargo_door)^1.15;
% --- Systems routing ---
W_sys_routing=K_sys*(L*3.28*D_f*3.28);
% --- Floors ---
W_floors=S_floor_main*w_main+S_floor_upper*w_upper;
% --- Fuselage structural weight correlation ---
if type2==1
W_fus=(.052*(W_cc_q(q-1)*.225*N_z)^.177*(.5*rho*V_inf^2)^.241*(S_fus*3.28^2)^1.086*(L/D_f)^-.072)/.225+W_sys_routing+W_floors+W_doors+W_press;
elseif type2==0
W_fus=(.328*(W_cc_q(q-1)*.225*N_z)^.5*(L/.3048)^.25*(S_fus*3.28^2)^.302*(L/D_f)^.1)/.225+W_sys_routing+W_floors+W_doors+W_press;    
else
W_fus=((.499*W_cc_q(q-1)^.35*.225*N_z^.25*(L/.3048)^.5*(D_f/.3048)^.849*(D_f*3.28)^.685)/.225+W_sys_routing+W_floors+W_doors+W_press)*1.3;       
end

% --- Landing-gear weight correlation ---
W_gear=(K_gr*(W_l*.225*N_z)^a_g*L_s^b_g)/.225;
% --- Wing structural weight correlation ---
%% ---- metric -> Raymer English ----
N2lb = 1/4.44822;  lb2N = 4.44822;
m2ft = 1/0.3048;   m2_to_ft2 = m2ft^2;

W_dg_lb  = W_cc_q(q-1) * N2lb;
W_l_lb   = W_l  * N2lb;
S_w_ft   = S(q) * m2_to_ft2;
S_csw_ft = S_csw* m2_to_ft2;
S_ht_ft  = S_ht * m2_to_ft2;
S_vt_ft  = S_vt * m2_to_ft2;
S_cs_ft  = S_cs * m2_to_ft2;
L_ft     = L    * m2ft;
D_ft     = D_f  * m2ft;
W_ft     = W_fuse * m2ft;
L_t_ft   = L_t  * m2ft;
F_w_ft   = F_w  * m2ft;
B_h_ft   = B_h  * m2ft;
L_m_in   = L_m  / 0.0254;
L_n_in   = L_n  / 0.0254;
L_m_ft   = L_m  * m2ft;
L_n_ft   = L_n  * m2ft;
D_e_ft   = D_e  * m2ft;
L_d_ft   = L_d  * m2ft;
L_tp_ft  = L_tp * m2ft;
L_ec_ft  = L_ec * m2ft;

if Afterburner==0
T_lb     = Th_dry * N2lb;
else
T_lb     = Th_wet * N2lb;
end

W_en_lb  = W_eng_e * N2lb;
W_uav_lb = W_uav * N2lb;
V_t_gal  = V_t  * 264.172;
S_f_wet_ft = pi * D_ft * L_ft;
Q_psf    = 0.5*rho*V_inf^2 * 0.0208854;

if type2 == 1
    % GA 15.46, 15.49-15.58
    W_wing_lb = 0.036*S_w_ft^0.758*W_dg_lb^0.0035 ...
        *(AR/cos(sweep_q)^2)^0.6*Q_psf^0.006*max(lambda,0.01)^0.04 ...
        *(100*tdc/cos(sweep_q))^(-0.3)*(N_z*W_dg_lb)^0.49;
    W_fus_lb  = 0.052*S_f_wet_ft^1.086*(N_z*W_dg_lb)^0.177 ...
        *L_t_ft^(-0.051)*(L_ft/D_ft)^(-0.072)*Q_psf^0.241;
    W_gear_lb = 0.095*(N_l*W_l_lb)^0.768*(L_m_in/12)^0.409 ...
              + 0.125*(N_l*W_l_lb)^0.566*(L_n_in/12)^0.845;
    W_HT_lb=0.016*(N_z*W_dg_lb)^.414*Q_psf^.168*S_ht_ft^.896*(100*tdc/cos(sweep))^-.12;
    W_VT_lb   = 0.073*(1+.2*H_t_Hv)^0.225*W_dg_lb^0.376*N_z^0.376*Q_psf^.122 ...
    *S_vt_ft^0.873*(100*tdc/cos(sweep))^-.49;
    W_tail_lb = W_HT_lb+W_VT_lb;
    W_av_lb   = 2.117*W_uav_lb^0.933;
    W_ac_lb   = 0.265*W_dg_lb^0.52*Np^0.68*W_av_lb^0.17*M^0.08;
    b_ft      = sqrt(S_w_ft*AR);
    W_fc_lb   = 0.053*L_ft^1.536*b_ft^0.371*(N_z*W_dg_lb*1e-4)^0.80;
    W_fs_lb   = 2.49*V_t_gal^0.726*(1/(1+Vi_Vt))^0.363*N_t^0.242*N_en^0.157;
    W_ein_lb  = 2.575*W_en_lb^0.922*N_en; 
    
    % ===== GA 15.55-15.58 (15.52-54 already in your block) =====
    W_hyd_lb  = 0.001 * W_dg_lb;                                          % 15.55
%     W_av_lb   = 2.117 * W_uav_lb^0.933;                                   % 15.57
    W_elec_lb = 12.57 * (W_fs_lb + W_av_lb)^0.51;                         % 15.56
    W_furn_lb = 0;                                                        % no separate GA cabin eq
    W_inst_lb = 0;
    W_apu_lb  = 0;
    W_misc_lb = 0;
    
    W_nac_lb = 0;
    
    W_hg=0;


elseif type2 == 0
%     transport 15.25-15.42
    W_wing_lb = 0.0051*(W_dg_lb*N_z)^0.557*S_w_ft^0.649*AR^0.5 ...
        *tdc^(-0.4)*(1+lambda)^0.1*(cos(sweep_q))^(-1.0)*S_csw_ft^0.1;
    W_fus_lb  = 0.3280*K_door*K_Lg*(W_dg_lb*N_z)^0.5*L_ft^0.25 ...
        *S_f_wet_ft^0.302*(1+K_ws)^0.04*(L_ft/D_ft)^0.10;
    Vstall_kts = V_stall_sl*1.94384;
    W_gear_lb = 0.0106*K_mp*W_l_lb^0.888*N_l^0.25*L_m_ft^0.4 ...
        *N_mw^0.321*max(N_mss,1)^(-0.5)*Vstall_kts^0.1 ...
        + 0.032*K_np*W_l_lb^0.646*N_l^0.2*L_n_ft^0.5*N_nw^0.45;
    W_HT_lb   = 0.0379*K_uht*(1+F_w_ft/B_h_ft)^(-0.25)*W_dg_lb^0.639 ...
        *N_z^0.10*S_ht_ft^0.75*L_t_ft^(-1.0)*(cos(sweep_q))^(-1.0);
    W_VT_lb   = 0.0026*(1+H_t_Hv)^0.225*W_dg_lb^0.556*N_z^0.536 ...
        *L_t_ft^(-0.5)*S_vt_ft^0.5;
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_av_lb   = 1.73*W_uav_lb^0.983;
    Vpr_ft3   = (pi*D_ft^2/4)*L_ft;
    W_ac_lb   = 62.36*Np^0.25*(Vpr_ft3/1000)^0.604*W_uav_lb^0.10;
    W_fc_lb   = 145.9*Nf^0.554*(1+Nm/max(Nf,1))^(-1)*S_cs_ft^0.20;
    W_fs_lb   = 2.405*V_t_gal^0.606*(1+Vi_Vt)^(-1)*(1+Vp_Vt)*N_t^0.5;
    W_ein_lb  = N_en*W_en_lb*1.3;
    
%     furnishings (15.41)  — this is the cabin
    W_furn_lb = 0.0577 * Np^0.1 * (W_dg_lb)^0.393 * S_f_wet_ft^0.75;

%     hydraulics (15.38)
    W_hyd_lb  = 0.2673 * Nf * (L_ft + B_h_ft)^0.937;

%     electrical (15.39)  Rkva ~ 60 kVA class
    W_elec_lb = 7.291 * 60^0.782 * L_ft^0.346 * N_en^0.10;

%     instruments (15.37)
    W_inst_lb = 4.509 * 1.0 * 1.0 * N_c^0.341 * N_c * (L_ft + B_h_ft)^0.5;

%     APU installed (15.36)  — ~200 kg uninstalled class
    W_apu_lb  = 2.2 * 440;

%     anti-ice + handling (15.43-15.44)
    W_misc_lb = 0.002*W_dg_lb + 3e-4*W_dg_lb;
    
    W_ec_lb  = 2.331 * W_en_lb^0.901 * K_p * K_tr;     % per nacelle [lb]

    W_nac_lb = 0.6724 * K_ng * N_Lt_ft^0.10 * N_w_ft^0.294 ...
         * N_z^0.119 * W_ec_lb^0.611 * N_e^0.984 * S_n_ft^0.224;
     
    W_hg=3E-4*W_cc_q(q-1);


else
%     fighter/delta 15.1-15.22  (XB-70: type2=2)
    W_wing_lb = 0.0103*K_dw*K_vs*(W_dg_lb*N_z)^0.5*S_w_ft^0.622*AR^0.785 ...
        *tdc^(-0.4)*(1+lambda)^0.05*(cos(sweep_q))^(-1.0)*S_csw_ft^0.04;
    W_fus_lb  = 0.499*K_dwf*W_dg_lb^0.35*N_z^0.25*L_ft^0.5*D_ft^0.849*W_ft^0.685;
    W_MLG_lb  = K_cb*K_tpg*(W_l_lb*N_l)^0.25*L_m_in^0.973;
    W_NLG_lb  = (W_l_lb*N_l)^0.290*L_n_in^0.5*N_nw^0.525;
    W_gear_lb = W_MLG_lb + W_NLG_lb;
    W_HT_lb   = 3.316*(1+F_w_ft/B_h_ft)^(-2.0)*((W_dg_lb*N_z)/1000)^0.260*S_ht_ft^0.806;
    W_VT_lb   = 0.452*K_rht*(1+H_t_Hv)^0.5*(W_dg_lb*N_z)^0.488*S_vt_ft^0.718 ...
        *M^0.341*L_t_ft^(-1.0)*(1+Sr_Svt)^0.348*A_vt^0.223 ...
        *(1+lambda)^0.25*(cos(sweep_vt))^(-0.323);
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*Ls_Ld^(-0.373)*D_e_ft;
    W_mnt_lb  = 0.013*N_en^0.795*T_lb^0.579*N_z;
    W_esc_lb  = 0.01*W_en_lb^0.717*N_en*N_z;
    W_tp_lb   = 3.5*D_e_ft*L_tp_ft*N_en;
    W_ein_lb  = N_en*W_en_lb + W_inl_lb + W_mnt_lb + W_esc_lb + W_tp_lb;
    W_fs_lb   = 7.45*V_t_gal^0.47*(1+Vi_Vt)^(-0.095)*(1+Vp_Vt) ...
        *N_t^0.066*N_en^0.052*(T_lb*SFC/1000)^0.249;
    W_fc_lb   = 36.28*M^0.003*S_cs_ft^0.489*N_s^0.484*N_c^0.127;
 
%     ===== fighter 15.13-15.15, 15.18-15.24 =====
    T_e_lb    = T_lb / max(N_en,1);                                       % thrust per engine
    W_oil_lb  = 37.82 * N_en^1.023;                                       % 15.13
    W_ectrl_lb= 10.5 * N_en^1.008 * L_ec_ft^0.222;                        % 15.14
    W_strt_lb = 0.025 * T_e_lb^0.760 * N_en^0.72;                         % 15.15
    W_inst_lb = 8.0 + 36.37*N_en^0.676*N_t^0.237 + 26.4*(1+N_ci)^1.356;   % 15.18
    W_hyd_lb  = 37.23 * K_vsh * N_u^0.664;   
    L_a_ft    = L/.3048;
    N_gen     = N_en;
    W_elec_lb = 172.2 * K_mc * R_kva^0.152 * N_c^0.10 * L_a_ft^0.10 ...
                * N_gen^0.091;                                            % 15.20
    W_av_lb   = 2.117 * W_uav_lb^0.933;                                   % 15.21
    W_furn_lb = 217.6 * N_c;                                              % 15.22 crew station
    W_ac_lb   = 201.6 * ((W_uav_lb + 200*N_c)/1000)^0.735;                % 15.23
    W_misc_lb = 3.2e-4 * W_dg_lb;                                         % 15.24 handling
    W_apu_lb  = 0;
    W_ein_lb  = W_ein_lb + W_oil_lb + W_ectrl_lb + W_strt_lb;
    W_nac_lb = 0;
    W_hg=0;
    
end

%% ---- fudge factors (keep Raymer exponents; scale constants) ----
switch type2
    case 0
        if M<1
        K_wing = 1.51; K_fuse = 1.51; K_gear = 1.51;
        K_tail = 1.51; K_prop = 1.51; K_fuel = 1.51;
        K_sys  = 1.51; K_furn = 1.51; K_oew  = 1.51;
        else
%         K_wing = 1.15;
%         K_fuse = 1.15;
%         K_gear = 1.10;
%         K_tail = 1.10;
%         K_prop = 1.00;
%         K_fuel = 1.05;
%         K_sys  = 1.10;
%         K_furn = 1.00;
%         K_oew  = 1.05;
%         
%         K_wing = 1.20;    % High structural/aeroelastic loads, slender delta
%         K_fuse = 1.15;    % Supersonic pressure vessel + thermal structure
%         K_gear = 1.10;    % Heavy landing gear for high MTOW
%         K_tail = 1.10;    % Supersonic stability/control requirements
%         K_prop = 1.00;    % No propeller
%         K_fuel = 1.10;    % Extensive fuel system/tanks/transfer system
%         K_sys  = 1.20;    % Complex SST systems
%         K_furn = 1.05;    % Passenger furnishings/interior
%         K_oew  = 1.05;    % Overall OEW allowance
        
        % Concorde / Tu-144  (type2==0 transport eqs)
        K_wing = 1.222;   % slender delta; K_dw=0.768 already in 15.1
        K_fuse = 1.222;   % long intake tunnels + ogive, not a tube airliner
        K_gear = 1.222;   % 8-wheel main trucks, high W_l
        K_tail = 1.222;   % S_ht ~ 0; do not inflate a missing tail
        K_prop = 1.222;   % four Olympus + 2-D ramps / reversers
        K_fuel = 1.222;   % 15.16 under-runs ~96 t of integral fuel
        K_sys  = 1.222;   % hyd + elec + intake control
        K_furn = 1.222;   % 100 pax cabin; not 737 K_furn=2.5
        K_oew  = 1.222;   % leftover ops / unusable / heat structure
        end
    case 1
        if M<1
        % light jet (Citation CJ3 class): pressurized, T-tail, retractable
    % only if you are running the CJ3 — comment the piston block out
    K_wing = 1.294;   % laminar wing + fuel in wing
    K_fuse = 1.294;   % 15.49 without the pressurization add-on under-runs
    K_gear = 1.294;   % retractable
    K_tail = 1.294;   % T-tail (H_t_Hv=1 already helps if coded)
    K_prop = 1.294;   % use book engine weight
    K_fuel = 1.294;   % 15.54 is a light-airplane fuel system
    K_sys  = 1.294;   % hyd 15.55 is ~14 lb on a CJ3; real system is larger
    K_furn = 1.294;   % interior / pressurization leftover
    K_oew  = 1.294;
        else
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        end
    otherwise
        if M<3
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
        if skin==1
            K_mat=1;
        elseif skin==2
            K_mat=1.4;
        else
            K_mat=1.5;
        end
        K_wing = K_mat;   % honeycomb + high-temp skins; 15.1 under-runs
        K_fuse = K_mat;   % long inlet tunnels + heat structure
        K_gear = K_mat;   % high W_l, bicycle/truck gear not in 15.4–15.5
        K_tail = K_mat;   % canards / twin fins if S_ht, S_vt are small
        K_prop = K_mat;   % 15.10–15.15 inlet+install light vs 6-engine pack
        K_fuel = K_mat;   % 15.16 vs ~300000 lb fuel system / tanks
        K_sys  = K_mat;   % hyd + elec + ECS at Mach 3
        K_furn = K_mat;   % no airliner cabin; leave 15.22 as-is
        K_oew  = K_mat;   % leftover ops / unusable / TPS not in Ch. 15
        end
end


W_wing     = K_wing * W_wing_lb * lb2N;
W_fus      = K_fuse * W_fus_lb  * lb2N;
W_gear     = K_gear * W_gear_lb * lb2N;
W_tail     = K_tail * W_tail_lb * lb2N;
W_eng_inst = K_prop * W_ein_lb  * lb2N;
W_nac      = K_prop * W_nac_lb  * lb2N;
W_fuel_sys = K_fuel * W_fs_lb   * lb2N;
W_fc       = K_sys  * W_fc_lb   * lb2N;
W_av       = K_sys  * W_av_lb   * lb2N;
W_ac       = K_sys  * W_ac_lb   * lb2N;
W_hyd      = K_sys  * W_hyd_lb  * lb2N;
W_elec     = K_sys  * W_elec_lb * lb2N;
W_inst     = K_sys  * W_inst_lb * lb2N;
W_apu      = K_sys  * W_apu_lb  * lb2N;
W_furn     = K_furn * W_furn_lb * lb2N;
W_misc     = W_misc_lb * lb2N;


W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg;

if M>2
if TPS==1
    
W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_tps;

end
end



% % --- Empty-weight buildup ---
% if type2==1
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% end
% elseif type2==0
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_av+W_ac+W_fc+W_sys;
% end
% else
% if M>2
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_tps+W_skin+W_payload+W_fc+W_sys;
% else
% W_e(q)=W_fus+W_gear+W_eng+W_wing+W_tail+W_skin+W_payload+W_fc+W_sys;
% end
% end

% --- Descent phase ---
% --- Delta altitude for iteration
dalt_dd=alt/dd_max;
% --- Initial value for descent weight
W_dd(1)=W_l;
% --- Initial value for descent distance
x_dd(1)=0;
% --- Indicated airspeed ---
V_a_c=V_inf*(rho/rho_sl)^.5;
% --- Initial value for velocity
V_dd(1)=V_a_c;
for dd = 2:dd_max+1
    % --- Altitude ---
    alt_dd=dalt_dd*dd;
    
% --- Atmospheric temperature model (piecewise approximation) ---
if alt_dd<=11000
    T_dd=288.15-.00649*alt_dd;
elseif alt_dd>11000 && alt_dd<25000
    T_dd=216.65;
else
    T_dd=141.89+.00299*alt_dd;
end

if alt_dd<=11000
P_dd=101290*(T_dd/288.08)^5.256;
elseif alt_dd>11000 && alt_dd<25000
P_dd=22650*exp(1.73-.000157*alt_dd);
else
P_dd=2488*(T_dd/216.6)^-11.388;
end

rho_dd=P_dd/(R*T_dd);
    
    a_dd=(y*R*T)^.5;
    
    V_inf_dd=(y*R*T_dd)^.5*M;

% --- Descent speed under constant dynamic pressure ---
    V_dd(dd)=V_a_c*(rho_sl/rho_dd)^.5;
% --- Descent time increment ---
    dtime_dd=dalt_dd/rate_of_descent;
% --- descent deceleration ---
    acc_dd=(V_dd(dd)-V_dd(dd-1))/dtime_dd;
% --- Descent flight-path angle ---
    theta_dd_B=asin(rate_of_descent/V_dd(dd));
    
% --- Mach number
    M_dd=V_inf_dd/a_dd;  
% --- Drag
if M_dd<1 && M<1
    D_dd=(1-M^2)^.5/(1-M_dd^2)^.5*D;
elseif M_dd<1 && M>1
    D_dd=(1-M^2)^.5/(M_dd^2-1)^.5*D;
elseif M_dd>1 && M<1
    D_dd=(M^2-1)^.5/(1-M_dd^2)^.5*D;
else
    D_dd=(M^2-1)^.5/(M_dd^2-1)^.5*D;
end
% --- Available thrust at descent altitude ---
    Th_dd=D_dd+W_dd(dd-1)*sin(theta_dd_B)+W_dd(dd-1)/g*acc_dd;
% --- Horizontal velocity ---
    Vx_dd=V_dd(dd)*cos(theta_dd_B);
% --- Horizontal descent distance increment ---
    dx_dd=dtime_dd*Vx_dd;
% --- Horizontal descent total distance ---
    x_dd(dd)=dx_dd+x_dd(dd-1);
% --- Thrust specific fuel consumption
    TSFC_dd(dd)=(T_dd/T)^.5*TFC;
% --- Overall efficiency ---
    no_dd=V_dd(dd)/(TSFC_dd(dd)*E_sp);
% --- Descent fuel weight increment ---
    dW_dd=(Th_dd*V_dd(dd)/no_dd)/E_sp*g*dtime_dd;
% --- Total descent fuel weight ---
    W_dd(dd)=W_dd(dd-1)+dW_dd;     
end

% --- Cruise fuel from Breguet range relation ---
W_f(q)=((exp(Range_cruise*g*TFC/(M*a*LdD(q))))-1)*(W_dd(dd));
W_dd_q(q)=W_dd(dd);
TOGW_i(q)=W_f(q)+W_dd(dd);

% --- climb phase ---
% --- Delta altitude for iteration
dalt_cc=alt/cc_max;
% --- Initial value for climb weight
W_cc(1)=TOGW_i(q);
% --- Initial value for climb distance
x_cc(1)=0;
% --- Initial value for velocity
V_cc(1)=V_a_c*(rho_sl/rho)^.5;
for cc = 2:cc_max+1
% --- Altitude ---
    alt_cc=dalt_cc*(cc_max-cc+2);
    
% --- Atmospheric temperature model (piecewise approximation) ---
if alt_cc<=11000
    T_cc=288.15-.00649*alt_cc;
elseif alt_cc>11000 && alt_cc<25000
    T_cc=216.65;
else
    T_cc=141.89+.00299*alt_cc;
end

if alt<=11000
P_cc=101290*(T_cc/288.08)^5.256;
elseif alt>11000 && alt<25000
P_cc=22650*exp(1.73-.000157*alt);
else
P_cc=2488*(T_cc/216.6)^-11.388;
end

rho_cc=P_cc/(R*T_cc);
    
    a_cc=(y*R*T_cc)^.5;
    

% --- Climb speed under constant dynamic pressure ---
    V_cc(cc)=V_a_c*(rho_sl/rho_cc)^.5;
% --- Climb time increment ---
    dtime_cc=dalt_cc/rate_of_climb;
% --- climb acceleration ---
    acc_cc=(V_cc(cc)-V_cc(cc-1))/dtime_cc;
% --- Climb flight-path angle ---
    theta_cc_B=asin(rate_of_climb/V_cc(cc));
    
% --- Mach number
M_cc=V_inf_dd/a_cc;  
% --- Drag
if M_cc<1 && M<1
    D_cc=(1-M^2)^.5/(1-M_cc^2)^.5*D;
elseif M_dd<1 && M>1
    D_cc=(1-M^2)^.5/(M_cc^2-1)^.5*D;
elseif M_dd>1 && M<1
    D_cc=(M^2-1)^.5/(1-M_cc^2)^.5*D;
else
    D_cc=(M^2-1)^.5/(M_cc^2-1)^.5*D;
end
% --- Thrust required for climb ---
    Th_b_cc=D_cc+W_cc(cc-1)*sin(theta_cc_B)+W_cc(cc-1)/g*acc_cc;
% --- Horizontal velocity ---
    Vx_cc=V_cc(cc)*cos(theta_cc_B);
% --- Horizontal climb distance increment ---
    dx_cc=dtime_cc*Vx_cc; 
% --- Horizontal climb total distance ---
    x_cc(cc)=dx_cc+x_cc(cc-1);
% --- Thrust specific fuel consumption
    TSFC_cc(cc)=(T_cc/T)^.5*TFC;
% --- Overall efficiency ---
    no_cc=V_cc(cc)/(TSFC_cc(cc)*E_sp);
% --- Climb fuel/weight increment ---
    dm_cc(cc)=((Th_b_cc*V_cc(cc)/no_cc)/E_sp)*g*dtime_cc; 
% --- Total climb fuel/weight ---
    W_cc(cc)=W_cc(cc-1)+dm_cc(cc);       
end

W_cc_q(q)=W_cc(cc);

end

% W_cc_q(q)=W_cc_i_min;
% 
% W_cc_q_min=min(W_cc_q(q));
% 
% if W_cc_q(q)==W_cc_i_min
%     CPR_ii=CPR;
%     W_cc_q_min=W_cc_i(i);
% end


end


TOGW_alt(zzz)=W_cc_q(q);

if (TOGW_alt(zzz)<zz && TOGW_alt(zzz)>0) || TOGW_alt(zzz-1)==0
    zz=TOGW_alt(zzz);
    alt_1_BB=alt_zzz(zzz);
end

if TOGW_alt(zzz)>0
    var(zzz-1)=TOGW_alt(zzz);
end

var_min=min(var);

if Afterburner==0
    
if TOGW_alt(zzz)==var_min
    if type==0
        D_max_i=D_max;
        LdD_1=LdD(q);
        TIT_i=TIT_e;
        CPR_i=CPR_e;
        BR_i=BR_e;
        FPR_i=FPR_e;
        S_1=S(q);
        rho_1=rho;
        V_inf_1=V_inf;
        C_d_0_1=C_d_0;
        W_l_1=W_l;
        W_cc_1=W_cc_q(q);
        K_1=1/(pi*AR*e);
        TOGW_11=TOGW_i(q);
        TOGW_111=W_dd_q(q);
        Th_ee=Th_e_0_max;
        Th_i_Dmax=Th_e_0_Dmax;
        c_l_i=C_l;
        c_r_i=c_r;
        c_t_i=c_t;
        b_i=b;
        sweep_i=sweep;
        if M<1
        Th_i=Th_e_0;
        Th_i_max=D_max/n;
        else
        Th_i=Th_e;
        end
    else
        D_max_i=D_max;
        LdD_1=LdD_k;
        TIT_i=TIT_ii;
        CPR_i=CPR_ii;
        BR_i=BR_ii; 
        Th_i=Th_ii;
        FPR_i=FPR_ii;
        S_1=S_k;
        C_d_0_1=C_d_0_k;
        rho_1=rho_k;
        W_l_1=W_l_k;
        V_inf_1=V_inf_k;
        W_cc_1=W_cc_i_min;
        K_1=K_k;
        TOGW_11=TOGW_i_k;
        TOGW_111=TOGW_i_kk;
        Th_sl_i=Th_sl_k;
        Th_max_1=T_max_k;
        Th_i_Dmax=D_max_i/n;
        c_l_i=c_l_k;
        c_r_i=c_r_k;
        c_t_i=c_t_k;
        b_i=b_k;
        sweep_i=sweep_k;
    end
    alt_i=alt;
    TOGW_1=TOGW_alt(zzz);
    Drag=D;
    LdD_i=LdD(q);
    W_e_1=W_e(q);
    TFC_1=TFC;
    m_f_m_i=m_f_m;
%     if M>2
%     T_f_i=T_fuel_B;
%     end
    V_i=V_inf;
    V_a=V_a_c;
    x_dd_i=x_dd(dd);
    W_cc_i=W_cc;
    x_cc_i=x_cc(cc);
    A_i=A_inlet; 
    thrust_1=Th_i;
end

else
    
if TOGW_alt(zzz)==var_min
    if type==0
        LdD_1=LdD(q);
        TIT_i=TIT_e;
        CPR_i=CPR_e;
        BR_i=BR_e;
        FPR_i=FPR_e;
        S_1=S(q);
        rho_1=rho;
        V_inf_1=V_inf;
        C_d_0_1=C_d_0;
        W_l_1=W_l;
        W_cc_1=W_cc_q(q);
        K_1=1/(pi*AR*e);
        TOGW_11=TOGW_i(q);
        TOGW_111=W_dd_q(q);
        Th_ee=Th_e_max;
        Th_ab=T_k_e_max;
        Drag=D;
        Th_i_Dmax=D_max/n;
        c_l_i=C_l;
        c_r_i=c_r;
        c_t_i=c_t;
        b_i=b;
        sweep_i=sweep;
        if M<1
        Th_i=Drag/n;
        Th_i_max=D_max/n;
        else
        Th_i=Drag/n;
        Th_i_max=D_max/n;
        end
    else
        LdD_1=LdD_k;
        Drag=D;
        TIT_i=TIT_ii;
        CPR_i=CPR_ii;
        BR_i=BR_ii;
        Th_i=Drag/n;
        FPR_i=FPR_ii;
        S_1=S_k;
        C_d_0_1=C_d_0_k;
        rho_1=rho_k;
        W_l_1=W_l_k;
        V_inf_1=V_inf_k;
        W_cc_1=W_cc_i_min;
        K_1=K_k;
        TOGW_11=TOGW_i_k;
        TOGW_111=TOGW_i_kk;
        Th_sl_i=Th_sl_k;
        Th_max_1=T_max_k;
        Th_i_max=D_max/n;
        Th_i_Dmax=D_max/n;
        c_l_i=c_l_k;
        c_r_i=c_r_k;
        c_t_i=c_t_k;
        b_i=b_k;
        sweep_i=sweep_k;
    end
    alt_i=alt;
    LdD_i=LdD(q);
    
%     if M>2
%     T_f_i=T_fuel_B;
%     end
    
    T_0_6_i=T_0_6_k_e;
    W_e_1=W_e(q);
    TFC_1=TFC;
    V_i=V_inf;
    V_a=V_a_c;
    x_dd_i=x_dd(dd);
    W_cc_i=W_cc;
    x_cc_i=x_cc(cc);
    A_i=A_inlet_e; 
    thrust_1=Th_i;
    if type==0
        f_6_k_e_ii=f_6_k_e;
    end
end

end

end


fprintf('TOGW=%f\nW_e=%f\nRange=%f\nLift to drag ratio=%f\nDrag_per_engine=%f\nS=%f\nAlt=%f\nV=%f\nV_a=%f\nc_r=%f\nc_t=%f\nb=%f\nSweep=%f\n',var_min,W_e_1,Range_cruise+x_dd_i+x_cc_i,LdD_i,Drag/n,S_1,alt_i,V_i,V_a,c_r_i, c_t_i,b_i, sweep_i*180/pi)

if M>1
if Afterburner==1 && type==0
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet_e,Drag/n,TIT_i,T_0_6_i,TFC_1,T_0_6_k_e_Dmax,M_6_e,M_6_Dmax)
elseif Afterburner==1 && type==1
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Drag/n,TIT_i,T_0_6_i,TFC_1,T_0_6_k_Dmax_min_j,M_6_k,M_6_k_Dmax)
elseif Afterburner==0 && type==0   
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet_e,Drag/n,TIT_i,TFC_1,Th_i,Drag/n)
else
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Drag/n,TIT_i,TFC_1,Th_i,Drag/n)   
end

else
if Afterburner==1 && type==0
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nThrust_afterburner_per_engine=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,D/n,TIT_i,T_0_6_i,TFC_1,T_0_6_k_e_Dmax,M_6_e,M_6_Dmax)
elseif Afterburner==1 && type==1
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nThrust_afterburner_per_engine=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,D_k/n,TIT_i,T_0_6_i,TFC_1,T_0_6_k_Dmax_min_j,M_6_k,M_6_k_Dmax)
else   
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,TIT_i,TFC_1,Th_i,Drag/n)
end
end
















S_err=(S0-S_1)/S0*100;
b_err=(b0-b_i)/b0*100;
c_r_err=(c_r0-c_r_i)/c_r0*100;

TOGW_err=(W_cc_1-TOGW_i(1))/W_cc_q(1)*100;
We_err=((W_e_1)-W_e(1))/(W_e(1))*100;

if aircraft_case==2 || aircraft_case==4
Wf_err=(abs(TOGW_i(1)-W_e(1)-W_payload)-abs(TOGW_i(1)-W_e_1-W_payload))/abs(TOGW_i(1)-W_e(1)-W_payload)*100;
else
Wf_err=(abs(TOGW_i(1)-W_e(1)-W_payload)-abs(W_cc_1-W_e_1-W_payload))/abs(TOGW_i(1)-W_e(1)-W_payload)*100;
end

alt_err=(Alt0-alt_i)/Alt0*100;
R_err=(R0-(Range_cruise+x_dd_i+x_cc_i))/R0*100;
L_D_err=(L_D-LdD_1)/L_D*100;
TSFC_err=(TSFC0-TFC_1)/TSFC0*100;

if M<1
sweep_err=(sweep_i-sweep0)/sweep0*100;
fprintf('\n\nS_error=%f\nb_error=%f\nc_r_error=%f\nTOGW_error=%f\nWe_error=%f\nWf_error=%f\nalt_error=%f\nR_error=%f\nL/D_error=%f\nTSFC_error=%f\nSweep_error=%f\n',S_err,b_err,c_r_err,TOGW_err,We_err,Wf_err,alt_err,R_err,L_D_err,TSFC_err,sweep_err)
else
fprintf('\n\nS_error=%f\nb_error=%f\nc_r_error=%f\nTOGW_error=%f\nWe_error=%f\nWf_error=%f\nalt_error=%f\nR_error=%f\nL/D_error=%f\nTSFC_error=%f\n',S_err,b_err,c_r_err,TOGW_err,We_err,Wf_err,alt_err,R_err,L_D_err,TSFC_err)
end










figure(1)
plot(alt_zzz(2:end),TOGW_alt(2:end),'.r')
xlabel ('Altitude [m]')
ylabel ('TOGW [N]')
title ('Design Point')

% sr_dd=2000/60; %Descent sink rate
% cr_cc=2000/60; %Climb rate
% Takeoff_field_length=7000/3.28; %Takeoff field length
% Landing_field_length=7000/3.28; %Takeoff field length
% g_turn=1.2;
V_s=1.3*V_stall_sl;
beta_land=W_l_1/TOGW_alt(zzz);


beta_cruise_0=TOGW_11/W_cc_1;
beta_cruise_1=TOGW_111/W_cc_1;


if alt_0<=11000
rho_0=rho_sl*exp(-alt_0/8500);
elseif alt_0>11000 && alt_0<20000
rho_0=.3639*exp(-(alt_0-11000)/6500);
else
rho_0=.088*exp(-(alt_0-20000)/7000);
end

if type==0
    W_ll=W_l;
else
    W_ll=W_l_k;
end

C_L_land=W_ll/(.5*rho_0*V_s^2*S_1);
C_L_to=W_cc_1/(.5*rho_0*V_lof^2*S_1);
nn=.7;
for zzz=1:zzz_max
    
   
    
    if type==0 && Afterburner==1
    Th_0=Th_wet*(rho_0/rho_sl)^nn;
    alpha_0=Th_0/Th_wet;
    Th_climb=Drag/n+W_cc_1*rate_of_climb/V_lof/n;
    alpha_0_climb=Th_climb/Th_wet;
    elseif type==0 && Afterburner==0
    Th_0=Th_dry*(rho_0/rho_sl)^nn;
    alpha_0=Th_0/Th_dry;
    Th_climb=Drag/n+W_cc_1*rate_of_climb/V_lof/n;
    alpha_0_climb=Th_climb/Th_dry;
    else
    Th_0=Th_sl_i*(rho_0/rho_sl)^nn;
    alpha_0=Th_0/Th_sl_i;
    Th_climb=Drag/n+W_cc_1*rate_of_climb/V_lof/n;
    alpha_0_climb=Th_climb/Th_sl_i;
    end
    
    WdS_min=0;
    WdS(zzz)=(WdS_max-WdS_min)*zzz/zzz_max+WdS_min;
    WdS_stall=.5*rho_sl*V_stall_sl^2*CL_max;
%     TdW_to(zzz)=1/alpha_0*(V_lof^2/(2*g*Takeoff_field_length)+.5*rho_0*V_lof^2*C_d_0_to/(WdS(zzz))+mu*(1-.5*rho_0*V_lof^2*C_L_to/(WdS(zzz))));    
    
    if type==0 && Afterburner==1
    Tu=Th_wet*rate_of_climb/V_lof;
    elseif type==0 && Afterburner==0
    Tu=Th_dry*rate_of_climb/V_lof;
    else
    Tu=Th_sl_i*rate_of_climb/V_lof;
    end
    
    CL_liftoff=.7*CL_max;
    L_liftoff=.5*rho_0*V_lof^2*CL_liftoff*S_1;
    acc_v=Tu/(W_cc_1/g);
    acc_v=L_liftoff/(W_cc_1/g);
    ttt=(2*50/3.28/acc_v)^.5;
%     ttt=50/3.28/rate_of_climb;
%     TdW_to(zzz)=((V_lof^2+2*(Takeoff_field_length-V_lof*ttt)*mu_2*g+2*(Takeoff_field_length-V_lof*ttt)*C_d_0_to*.5*rho_0*V_lof^2/W_cc_1)*W_cc_1/(2*(Takeoff_field_length-V_lof*ttt)*g))/W_cc_1*WdS(zzz)/(W_cc_1/S_1);
    TdW_to(zzz)=((V_lof^2+2*(Takeoff_field_length-V_lof*ttt)*mu_2*g+2*(Takeoff_field_length-V_lof*ttt)*C_d_0_to*.5*rho_0*V_lof^2/W_cc_1)*W_cc_1/(2*(Takeoff_field_length-V_lof*ttt)*g))/W_cc_1*WdS(zzz)/(W_cc_1/S_1);
    WdS_land=rho_0*g*Landing_field_length*W_l_1/(.5*rho_0*V_s^2*S_1)/(1.69*(beta_land));
    
    if type==0 && Afterburner==1
%     TdW_cruise_0(zzz)=beta_cruise_0/(Th_i/Th_wet)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_0(zzz)=Th_wet_cruise*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_0(zzz)=Th_i_Dmax*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));

%     TdW_cruise_1(zzz)=beta_cruise_1/(Th_i/Th_wet)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_1*WdS(zzz))+K_1/(beta_cruise_1*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_1(zzz)=Th_wet_cruise*n/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    TdW_cruise_1(zzz)=Th_wet_cruise*n/TOGW_111*TOGW_111/S_1/(WdS(zzz));

    elseif type==0 && Afterburner==0
%     TdW_cruise_0(zzz)=beta_cruise_0/(Th_i/Th_dry)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_0(zzz)=Th_dry_cruise*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_0(zzz)=Th_i_Dmax*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));

%     TdW_cruise_1(zzz)=beta_cruise_1/(Th_i/Th_dry)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_1*WdS(zzz))+K_1/(beta_cruise_1*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_1(zzz)=Th_dry_cruise*n/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    else
%     TdW_cruise_0(zzz)=beta_cruise_0/(Th_i/Th_sl_i)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_0(zzz)=Th_max_1*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_0(zzz)=Th_i_Dmax*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
%     TdW_cruise_1(zzz)=beta_cruise_1/(Th_i/Th_sl_i)*((.5*rho_1*V_inf_1^2)*C_d_0_1/(beta_cruise_1*WdS(zzz))+K_1/(beta_cruise_1*(.5*rho_1*V_inf_1^2))*WdS(zzz));
    TdW_cruise_1(zzz)=Th_max_1*n/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    end
    
    
    if type==0 && Afterburner==1
%     TdW_climb_0(zzz)=1/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_0*V_lof^2*C_d_0_to/WdS(zzz)+K_1/(.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_climb_0(zzz)=(Th_wet-(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
%     TdW_climb_1(zzz)=beta_cruise_0/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_1*V_inf_1^2*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*rho_1*V_inf_1^2)*WdS(zzz));
%     TdW_climb_1(zzz)=(Th_wet_cruise-(TOGW_11/n*rate_of_climb/V_inf_1))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_climb_1(zzz)=(Th_i_Dmax-(TOGW_11/n*rate_of_climb/V_inf_1))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));

    elseif type==0 && Afterburner==0
%     TdW_climb_0(zzz)=1/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_0*V_lof^2*C_d_0_to/WdS(zzz)+K_1/(.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_climb_0(zzz)=(Th_dry-(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));

%     TdW_climb_1(zzz)=beta_cruise_0/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_1*V_inf_1^2*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*rho_1*V_inf_1^2)*WdS(zzz));
    TdW_climb_1(zzz)=(Th_i_Dmax-(TOGW_11/n*rate_of_climb/V_inf_1))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
%     TdW_climb_1(zzz)=(Th_i_Dmax-(TOGW_11/n*rate_of_climb/V_inf_1))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));

    else
%     TdW_climb_0(zzz)=1/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_0*V_lof^2*C_d_0_to/WdS(zzz)+K_1/(.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_climb_0(zzz)=(Th_sl_i-(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/n*rate_of_climb/V_lof))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
%     TdW_climb_1(zzz)=beta_cruise_0/alpha_0_climb*(rate_of_climb/V_inf_1+.5*rho_1*V_inf_1^2*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1/(beta_cruise_0*rho_1*V_inf_1^2)*WdS(zzz));
%     TdW_climb_1(zzz)=(Th_max_1-(TOGW_11/n*rate_of_climb/V_inf_1))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_climb_1(zzz)=(Th_i_Dmax+(TOGW_11/n*rate_of_climb/V_inf_1))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));

    end
        
    if type==0 && Afterburner==1
%     TdW_turn_0(zzz)=1/alpha_0*(.5*rho_0*V_lof^2*C_d_0_to/WdS(zzz)+K_1*N_z^2/(.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_turn_0(zzz)=(Th_wet)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_0(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));

%     TdW_turn_1(zzz)=beta_cruise_0/alpha_0*(.5*rho_0*V_lof^2*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1*N_z^2/(beta_cruise_0*.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_turn_1(zzz)=(Th_wet_cruise)*n/(TOGW_11*N_z)*TOGW_11/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    elseif type==0 && Afterburner==0
%     TdW_turn_0(zzz)=(Th_dry)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_0(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
%     TdW_turn_1(zzz)=(Th_dry_cruise)*n/(TOGW_11*N_z)*TOGW_11/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    else
%     TdW_turn_0(zzz)=(Th_sl_i)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_0(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
%     TdW_turn_1(zzz)=(Th_max_1)*n/(TOGW_11*N_z)*TOGW_11/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*n/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    end
     
    if type==0 && Afterburner==1
%     TdW_a_0(zzz)=1/alpha_0*(acc/g+.5*rho_0*V_lof^2*C_d_0_to/WdS(zzz)+K_1*N_z^2/(.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_a_0(zzz)=(Th_wet-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
%     TdW_a_1(zzz)=beta_cruise_0/alpha_0*(acc/g+.5*rho_0*V_lof^2*C_d_0_1/(beta_cruise_0*WdS(zzz))+K_1*N_z^2/(beta_cruise_0*.5*rho_0*V_lof^2)*WdS(zzz));
%     TdW_a_1(zzz)=(Th_wet_cruise-(TOGW_11/n/g*acc))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    elseif type==0 && Afterburner==0
%     TdW_a_0(zzz)=(Th_dry-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
%     TdW_a_1(zzz)=(Th_dry_cruise-(TOGW_11/n/g*acc))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    else
%     TdW_a_0(zzz)=(Th_sl_i-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
%     TdW_a_1(zzz)=(Th_max_1-(TOGW_11/n/g*acc))*n/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/n/g*acc))*n/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    end
    
    
    TdW_comb(zzz)=TdW_climb_1(zzz)+TdW_turn_0(zzz);

    
end

if type==0 && Afterburner==1
    Th_iii=Th_ab;
elseif type==0 && Afterburner==0
    Th_iii=Th_ee;
else
    Th_iii=Th_i;
end

figure(2)
xline(WdS_stall)
hold on
xline(WdS_land)
hold on
plot(WdS,TdW_to)
hold on
plot(WdS,TdW_cruise_0)
hold on
plot(WdS,TdW_climb_1)
hold on
plot(WdS,TdW_turn_0)
% hold on
% plot(WdS,TdW_a_0)
hold on
plot(WdS,TdW_comb)
if Afterburner==0 && type==0
plot((TOGW_11/S_1),(Th_dry_cruise*n/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_dry*n/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
elseif Afterburner==1 && type==0
plot((TOGW_11/S_1),(Th_wet_cruise*n/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_wet*n/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
else
plot((TOGW_11/S_1),(Th_max_1*n/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_sl_i*n/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
end
legend('stall','land','takeoff','cruise','climb_1','turn','combined','end of climb','start of climb','location','N')
xlim([0 WdS_max])
% ylim([0 2])
xlabel('Wing loading (W/S) [N/m^2]')
ylabel('Thrust to weight ratio (T/W) [-]')
title('Constraint Analysis')

figure(3)
plot(qqq,W_cc_q,'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5)
xlabel('Iteration number')
ylabel('Takeoff gross weight [N]')
title('Convergence Check')



% figure(3)
% xline(WdS_stall)
% hold on
% xline(WdS_land)
% hold on
% plot(WdS,TdW_cruise_1)
% hold on
% plot(WdS,TdW_climb_1)
% hold on
% plot(WdS,TdW_turn_1)
% hold on
% plot(WdS,TdW_a_1)
% hold on
% plot((TOGW_11/S_1),(Th_iii*n/TOGW_11),'.r','Markersize',10)
% legend('stall','land','cruise','climb_0','turn','acceleration','design')










