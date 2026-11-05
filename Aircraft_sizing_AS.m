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
%    aircraft_case = 10  Lockheed L2000
%    aircraft_case = 11  B707-120
%    aircraft_case = 12  Lockheed C5 Galaxy

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

%The initial empty, takeoff, fuel, and payload weights of the verification
%aircrafts are equal to their actual values.

aircraft_case=1; %0=custom inputs, 1=Concorde, 2=TU-144D,
%3=B737-200 4=B777-200ER 5=A320-200 6=CJ-3 7=2000EX 8=XB-70 9=SR-71
%10=L2000 %11=B707-120 %12=C5A

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
N_en        = 4;               % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 1;               % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;               % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;               % Waverider configuration: 0 = disabled, 1 = enabled
Cd_engine   = 0.18;           % Engine/nacelle drag coefficient [-]

% ============================================================
% ================ IDEAL ENGINE DATA =====================
% ============================================================
A_inlet     = .951;            % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;            % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;            % High design turbine inlet temperature [K]
T_0_6_high  = 1800;            % High afterburner total temperature [K]
T_0_3_sl    = 550;             % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 15.5;            % Minimum compressor pressure ratio [-]
CPR_max     = 15.5;            % Maximum compressor pressure ratio [-]
BR_min      = 0;               % Minimum bypass ratio [-]
BR_max      = 0;               % Maximum bypass ratio [-]
FPR         = 0;               % Fan pressure ratio [-]
n_ab_low    = 0;               % Low afterburner setting [-]
n_ab_high   = 0.6;             % High afterburner setting [-]
eta_inlet   = 0.92;            % Inlet efficiency [-]
eta_c       = 0.85;            % Compressor efficiency [-]
eta_f       = 0.88;            % Fan efficiency [-]
eta_t       = 0.88;            % Turbine efficiency [-]
eta_b       = 0.98;            % Burner/combustor efficiency [-]
eta_ab      = 0.85;            % Afterburner efficiency [-]
eta_m       = 0.98;            % Mechanical efficiency [-]
eta_n       = 0.94;            % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency

% ============================================================
% ================ EXISTING ENGINE DATA =====================
% ============================================================
Th_wet         = 169200;       % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 140000;       % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 142300;       % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 140000;       % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 1200;         % Engine low turbine inlet temperature [K]
TIT_e          = 1400;         % Engine high turbine inlet temperature [K]
CPR_e          = 15.5;         % Engine compressor pressure ratio [-]
BR_e           = 0;            % Engine bypass ratio [-]
FPR_e          = 0;            % Engine fan pressure ratio [-]
eta_t_e        = 0.88;         % Engine turbine efficiency [-]
eta_f_e        = 0.88;         % Engine fan efficiency [-]
eta_ab_e       = 0.85;         % Engine afterburner efficiency [-]
eta_c_e        = 0.85;         % Engine compressor efficiency [-]
eta_m_e        = 0.98;         % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = .951;   % Existing engine inlet area [m^2]
T_0_6_k_e      = 1704+273;     % Existing engine turbine temperature [K]
W_eng_e        = 3175*9.81;    % Engine weight per engine [N] 

% ============================================================
% ==================== AERODYNAMICS ==========================
% ============================================================
Subsonic_airfoil = 5;          % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;              % 0=no delta wing, 1=delta wing
sweep_solver = 0;              % 0=use input, 1=solve for (wing)
x_c          = 0.5;            % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max       = 1.35;           % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;            % Aircraft maximum lift coefficient [-]
tdc          = 0.03;           % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 80;             % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;      % Reference dynamic viscosity [Pa*s]
nose_angle   = 5;              % Nose half angle (useful for waverider configuration)
sweep        = 55*pi/180;      % Wing leading edge sweep angle [rad]
lambda       = 0.15;           % Wing taper ratio [-]
droop        = 0;              % Drooped wing: 0 = no droop, 1=droop applied
AR           = 1.83;           % Wing aspect ratio [-]

% ============================================================
% ==================== TAIL INPUTS =================
% ============================================================

% -------------------- Horizontal tail ----------------------
S_H       = 0;                 % Horizontal-tail planform area [m^2]
b_H       = 0;                 % Horizontal-tail span [m]
sweep_H   = 0;                 % Horizontal-tail leading edge sweep angle [rad]
c_H       = 5.0;               % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;              % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.08;              % Horizontal-tail thickness-to-chord ratio [-]

% ---------------------- Vertical tail ----------------------
S_V       = 33.91;             % Vertical-tail/fin planform area [m^2]
b_V       = 11.32;             % Vertical-fin height/span [m]
sweep_V   = 51*pi/180;         % Vertical-fin leading-edge sweep angle [rad]
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
D_f    = 3.10;                 % Fuselage maximum diameter [m]
L      = 61.66;                % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.88;                 % Fuselage width [m]
S_csw  = 28.0;                 % Elevon/control-surface area [m^2]
F_w    = 2.88;                 % Fuselage width [m]
Ht_Hv = 0;                     % 0 for conventional tail, 1 for T tail
L_t    = 22.0;                 % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.35;                 % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;    % Total control-surface area [m^2]
S_fw   = 0;                     % Firewall surface area [m^2]

% --------------------- Landing gear -------------------------
L_m  = 3.50;                   % length of main landing gear, [m in this deck]
L_n  = 2.80;                   % nose gear length, [m in this deck]
N_mw = 8;                      % number of main wheels
N_nw = 2;                      % number of nose wheels
N_mss = 2;                     % number of main gear shock struts

% ---------------- Raymer correction factors -----------------
K_dw = 0.768;                  % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                    % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 0.774;                 % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                   % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                   % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                 % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;                   % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                    % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                   % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                    % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                    % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                    % 1.62 for variable geometry; = 1.0 otherwise
K_d = 2.75;                    % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6                    
Ls_Ld = 0;                  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 1.21;                % engine diameter, [m in this deck]
L_d     = 8.0;                 % duct length, [m in this deck]
L_tp    = 2.0;                 % length of tailpipe, [m in this deck]
L_sh    = 1.5;                 % length of engine shroud, [m in this deck]
L_a    = 25.0;                % electrical routing distance, generators to avionics to cockpit, [m in this deck]

% ---------------------- Fuel system -------------------------
Vi_Vt = 0.90;                  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;                  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 17;                    % number of fuel tanks

% ------------------- Systems / equipment -------------------
N_c = 3;                       % number of crew
N_ci = 2.0;                    % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
N_s = 2;                       % number of flight control systems
N_u = 8;                       % number of hydraulic utility functions (typically 5-15)
Nf = 5;                        % number of functions performed by controls (typically 4-7)
Nm = 2;                        % number of mechanical functions (typically 0-2)
W_uav = 800*9.81;              % uninstalled avionics weight, [N in this deck]

% -------------------- Electrical systems -------------------
K_vsh = 1.0;                   % 1.425 if variable sweep wing; = 1.0 otherwise
K_mc  = 1.0;                   % 1.45 if mission completion required after failure; = 1.0 otherwise
R_kva = 80;                    % system electrical rating, kv·A (typically 40-60, or 160)

% ============================================================
% ================= NACELLE / INTAKE =========================
% ============================================================
K_ng     = 1.0;                % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;                % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;               % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 8.0;                % nacelle length, [m in this deck]
N_w      = 1.40;               % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;        % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;            % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;        % nacelle length,
N_w_ft   = N_w/0.3048;         % nacelle width,
S_n_ft   = S_n/0.3048^2;       % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;    % engine weight, each,

% ============================================================
% ================= PERFORMANCE / TRAJECTORY ================
% ============================================================
rate_of_climb    = 5000/3.28/60;     % Rate of climb [m/s]
rate_of_descent  = 3000/3.28/60;     % Rate of descent [m/s]
Takeoff_field_length = 3400;         % Takeoff field length [m]
Landing_field_length = 2200;         % Landing field length [m]
g_turn = 1.2;                        % Turning/acceleration load factor [-]
mu_2 = 0.03;                         % Ground friction coefficient [-]
alt_0 = 1000;                        % Initial altitude used in carpet plot [m]
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
k_wall = 21;                     % Aircraft wall thermal conductivity [W/(m*K)]
k_ins = .3;                   % Insulation thermal conductivity [W/(m*K)]
t_ins = .051;                        % Insulation thickness [m]
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

min_alt = 14000;                     % Minimum search altitude [m]
max_alt = 18000;                     % Maximum search altitude [m]
WdS_max = 6000;                      % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 200;                    % Number of altitude points shown in design point plot [-]
i_max = 20;                          % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                         % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;                          % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                          % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;                           % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
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

elseif aircraft_case==1

% ==================== MISSION REQUIREMENTS ====================
M = 2;  % Cruise Mach number [-]
Range_cruise = 5600000;  % Cruise range [m]
TOGW_i(1) = 185000*9.81;     % Initial takeoff gross weight [N]
W_e(1)    = 78700*9.81;    % Initial empty operating weight [N]
W_payload = 13400*9.81;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 100;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% CONCORDE VERIFICATION AIRCRAFT DATA
% ============================================================

S0      = 358.25;  % Reference wing area [m^2]
b0      = 25.56;  % Reference wingspan [m]
c_r0    = 27.66;  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]
Alt0    = 16000;  % Reference cruise altitude [m]
R0      = 6200e3;  % Reference range [m]
L_D     = 7.14;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 33.8e-6;  % Reference cruise TSFC [kg/(N*s)]

% ==================== PROPULSION ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 4;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = .951;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1800;  % High afterburner total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 15.5;  % Minimum compressor pressure ratio [-]
CPR_max     = 15.5;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.6;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.18;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA (Olympus 593) ====================
Th_wet         = 169200;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 140000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 142300;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 140000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 15.5;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = .951;1.125;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1704+273;  % Existing engine turbine temperature [K]
W_eng_e        = 3175*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
% Subsonic_airfoil = 5;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.5;  % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max       = 1.35;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;  % Aircraft maximum lift coefficient [-]
tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 80;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 5;  % Nose half angle (useful for waverider configuration)
sweep        = 55*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.15;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]

%% ================= CONCORDE TAIL INPUTS =================
% Horizontal tail
S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail leading edge sweep angle [rad]
c_H  = 5.0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H = 0.08;  % Horizontal-tail thickness-to-chord ratio [-]

% Vertical tail / fin
S_V       = 33.91;  % Vertical-tail/fin planform area [m^2]
b_V       = 11.32;  % Vertical-fin height/span [m]
sweep_V   = 51*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V  = 5.5;  % Vertical-tail mean aerodynamic chord [m]
x_t_V = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V = 0.08;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                        % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                   % Sea-level air density [kg/m^3]
T_sl = 288.15;                    % Sea-level standard temperature [K]
R = 287.05;                       % Specific gas constant for air [J/(kg*K)]
y = 1.4;                          % Ratio of specific heats [-]
g = 9.81;                         % Gravitational acceleration [m/s^2]
p_sl = 101300;                    % Sea-level standard pressure [Pa]
radius_e = 6.371e6;               % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY (Concorde) ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 3.10;  % Fuselage maximum diameter [m]
L      = 61.66;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.88;  % Fuselage width [m]
S_csw  = 28.0;  % Elevon/control-surface area [m^2]
F_w    = 2.88;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 22.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 3.50;  % length of main landing gear, [m in this deck]
L_n  = 2.80;  % nose gear length, [m in this deck]
N_mw = 8;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 0.768;                     % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                       % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 0.774;                    % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                      % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                      % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                    % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;                         % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                       % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                      % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                       % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                       % 1.62 for variable geometry; = 1.0 otherwise
K_d = 2.75;                       % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;                     % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable) 
D_e     = 1.21;  % engine diameter, [m in this deck]
L_d     = 8.0;  % duct length, [m in this deck]
L_tp    = 2.0;  % length of tailpipe, [m in this deck]
L_sh    = 1.5;  % length of engine shroud, [m in this deck]
L_a    = 25.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 17;  % number of fuel tanks
N_c = 3;                          % number of crew
% N_ci = 2.0;                       % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;                          % number of flight control systems
% N_u = 8;  % number of hydraulic utility functions (typically 5-15)
Nf = 5;                           % number of functions performed by controls (typically 4-7)
Nm = 2;                           % number of mechanical functions (typically 0-2)
W_uav = 800*9.81;  % uninstalled avionics weight, [N in this deck]
% K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
% R_kva = 80;  % system electrical rating, kv·A (typically 40-60, or 160)

%% ---- nacelle / intake (4 integrated 2-D ramps) ----
K_ng     = 1.0;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 8.0;  % nacelle length, [m in this deck]
N_w      = 1.40;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 5000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 3000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3400;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude used in carpet plot [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS=0;  % Thermal protection system: 0=no, 1=yes
% R_wle = 0.344/12;  % Wing leading-edge radius [ft]
% g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
% J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity=0.8;  % Radiative emissivity [-]
% Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
% k_wall=21;  % Aircraft wall thermal conductivity [W/(m*K)]
% k_ins=.3;  % Insulation thermal conductivity [W/(m*K)]
% t_ins=.051;  % Insulation thickness [m]
% t_wall=.001;.0015;  % Wall skin thickness [m]
% Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
% rho_ins=112;270;  % Insulation density [kg/m^3]
% S_cooling=39;20;39;  % Active cooling surface area [m^2]
% rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
% x = 4;  % Heating-analysis longitudinal location [m]
% Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt=12000;  % Minimum search altitude [m]
max_alt=18000;  % Maximum search altitude [m]
WdS_max=6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points=200;  % Number of altitude points shown in design point plot [-]
i_max=20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;  % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max=20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;  % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max=50;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
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

elseif aircraft_case==2

% ==================== MISSION REQUIREMENTS (Tu-144D) ====================
M = 2.15;  % Cruise Mach number [-]
Range_cruise = 5500000;  % Cruise range [m]
TOGW_i(1) = 207000*9.81;  % Initial takeoff gross weight [N]
W_e(1)    = 99200*9.81;  % Initial empty operating weight [N]
W_payload = 2000*9.81;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 140;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% TU-144D VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 507.0;  % Reference wing area [m^2]
b0      = 28.80;  % Reference wingspan [m]
c_r0    = 31.90;  % Reference root chord [m]
Lambda0 = 57*pi/180;  % Reference wing sweep [rad]
Alt0    = 18900;  % Reference cruise altitude [m]
R0      = 6200000;  % Reference range [m]
L_D     = 7.1;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 4.0e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 57*pi/180;

% ==================== PROPULSION (4 x RD-36-51) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 4;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 1.20;  % Inlet capture area per engine [m^2]
TIT_d_low   = 800;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1700;  % High afterburner total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 14.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 14.0;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.87;  % Turbine efficiency [-]
eta_b       = 0.97;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.93;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.18;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 196000;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 196000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 196000;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 180000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 14.0;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.87;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 1.2;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1600;  % Existing engine turbine temperature [K]
W_eng_e        = 4000*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
% Subsonic_airfoil = 5;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.5;  % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max       = 0.90;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 1.3;  % Aircraft maximum lift coefficient [-]
tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 71;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 6;  % Nose half angle (useful for waverider configuration)
sweep        = 57*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.12;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]

% ============================================================
% TU-144D TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 16.5;  % Horizontal-tail span [m]
sweep_H   = 30*pi/180;  % Horizontal-tail leading edge sweep angle [rad]
e_H       = 0.80;  % Horizontal-tail Oswald efficiency [-]
c_H       = 4.2;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.08;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 48.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 10.5;  % Vertical-fin height/span [m]
sweep_V   = 50*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 4.8;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.08;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                        % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                   % Sea-level air density [kg/m^3]
T_sl = 288.15;                    % Sea-level standard temperature [K]
R = 287.05;                       % Specific gas constant for air [J/(kg*K)]
y = 1.4;                          % Ratio of specific heats [-]
g = 9.81;                         % Gravitational acceleration [m/s^2]
p_sl = 101300;                    % Sea-level standard pressure [Pa]
radius_e = 6.371e6;               % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 3.50;  % Fuselage maximum diameter [m]
L      = 65.70;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 3.30;  % Fuselage width [m]
S_csw  = 35.0;  % Elevon/control-surface area [m^2]
F_w    = 3.30;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 24.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 3.60;  % length of main landing gear, [m in this deck]
L_n  = 2.90;  % nose gear length, [m in this deck]
N_mw = 16;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 0.768;                     % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                       % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 0.774;                    % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                      % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                      % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                    % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;                         % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                       % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                      % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                       % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                       % 1.62 for variable geometry; = 1.0 otherwise
K_d = 2.75;                       % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;                     % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 1.30;  % engine diameter, [m in this deck]
L_d     = 9.0;  % duct length, [m in this deck]
L_tp    = 2.2;  % length of tailpipe, [m in this deck]
L_sh    = 1.6;  % length of engine shroud, [m in this deck]
L_a    = 28.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 18;  % number of fuel tanks
N_c = 3;                          % number of crew
% N_ci = 2.0;                       % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;                          % number of flight control systems
% N_u = 8;                          % number of hydraulic utility functions (typically 5-15)
Nf = 5;                           % number of functions performed by controls (typically 4-7)
Nm = 2;                           % number of mechanical functions (typically 0-2)
W_uav = 900*9.81;                 % uninstalled avionics weight, [N in this deck]

%% ---- nacelle / intake (4 integrated 2-D ramps) ----
K_ng     = 1.00;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.00;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 8.0;  % nacelle length, [m in this deck]
N_w      = 1.35;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 9840/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 3000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3600;  % Takeoff field length [m]
Landing_field_length = 2500;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude used in carpet plot [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS=0;  % Thermal protection system: 0=no, 1=yes
% R_wle = 0.344/12;  % Wing leading-edge radius [ft]
% g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
% J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity=0.8;  % Radiative emissivity [-]
% Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
% k_wall=21;  % Aircraft wall thermal conductivity [W/(m*K)]
% k_ins=.3;  % Insulation thermal conductivity [W/(m*K)]
% t_ins=.051;  % Insulation thickness [m]
% t_wall=.001;.0015;  % Wall skin thickness [m]
% Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
% rho_ins=112;270;  % Insulation density [kg/m^3]
% S_cooling=39;20;39;  % Active cooling surface area [m^2]
% rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
% x = 4;  % Heating-analysis longitudinal location [m]
% Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 14000;  % Minimum search altitude [m]
max_alt = 20000;  % Maximum search altitude [m]
WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20;                       % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                      % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;                       % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                       % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;                        % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;                      % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;                      % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;                     % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;                      % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max = 30;                     % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max = 30;                      % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;                      % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;                 % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low = 0;                % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;             % Upper bound for existing engine afterburner fuel fraction flow rate

elseif aircraft_case==3

% ==================== MISSION REQUIREMENTS (737-200) ====================
M = 0.74;  % Cruise Mach number [-]
Range_cruise = 4600000;  % Cruise range [m]
TOGW_i(1) = 58740*9.81;  % Initial takeoff gross weight [N]
W_payload = 10000*9.81;  % Payload weight [N]
W_e(1)    = 27200*9.81;  % Initial empty operating weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 130;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% B737-200 VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 91;102.00;  % Reference wing area [m^2]
b0      = 28.35;  % Reference wingspan [m]
c_r0    = 5.9;7.32;  % Reference root chord [m]
Lambda0 = 25*pi/180;  % Reference wing sweep [rad]
Alt0    = 11300;  % Reference cruise altitude [m]
R0      = 4800e3;  % Reference range [m]
L_D     = 14;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 22.7e-6;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 25*pi/180;

% ==================== PROPULSION (2 x JT8D-15) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.85;  % Inlet capture area per engine [m^2]
TIT_d_low   = 800;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1350;  % High design turbine inlet temperature [K]
T_0_6_high  = 1350;  % High afterburner total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 12.5;  % Minimum compressor pressure ratio [-]
CPR_max     = 12.5;  % Maximum compressor pressure ratio [-]
BR_min      = 1.0;  % Minimum bypass ratio [-]
BR_max      = 1.0;  % Maximum bypass ratio [-]
FPR         = 1.65;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.97;  % Inlet efficiency [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = .32;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 69000;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 25000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 69000;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 25000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1350;  % Engine high turbine inlet temperature [K]
CPR_e          = 12.5;  % Engine compressor pressure ratio [-]
BR_e           = 1.0;  % Engine bypass ratio [-]
FPR_e          = 1.65;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.8;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.82;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 0.83;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1350;  % Existing engine turbine temperature [K]
W_eng_e        = 1455*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.323;  % Airfoil maximum thickness location as a fraction of the chord length [-]
% cl_max       = 2.6;
% CL_max       = 2.4;          % landing flaps
tdc          =.1537; % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 95;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 12;  % Nose half angle (useful for waverider configuration)
sweep        = 25*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.27;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
C_p_0        = -1.1;    %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil using a design sweep C_L=.35 (less than the maximum cruise distance case)
% C_p_0        = -.8498;   %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil
% ============================================================
% B737-200 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 32.8;  % Horizontal-tail planform area [m^2]
b_H       = 12.9;  % Horizontal-tail span [m]
sweep_H   = 32*pi/180;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 2.55;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 19.5;  % Vertical-tail/fin planform area [m^2]
b_V       = 6.0;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 3.25;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                        % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                   % Sea-level air density [kg/m^3]
T_sl = 288.15;                    % Sea-level standard temperature [K]
R = 287.05;                       % Specific gas constant for air [J/(kg*K)]
y = 1.4; g = 9.81;                % Ratio of specific heats [-]
p_sl = 101300;                    % Sea-level standard pressure [Pa]
radius_e = 6.371e6;               % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 3.76;  % Fuselage maximum diameter [m]
L      = 30.53;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 3.76;  % Fuselage width [m]
S_csw  = 18.0;  % Elevon/control-surface area [m^2]
F_w    = 3.76;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 14.5;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 2.40;  % length of main landing gear, [m in this deck]
L_n  = 1.80;  % nose gear length, [m in this deck]
N_mw = 4;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 1.0;                       % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                       % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;                      % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                      % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                      % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                    % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1.0;                       % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                       % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                      % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                       % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                       % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;                        % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 1.03;  % engine diameter, [m in this deck]
L_d     = 2.0;  % duct length, [m in this deck]
L_tp    = 2.0;  % length of tailpipe, [m in this deck]
L_sh    = 1.0;  % length of engine shroud, [m in this deck]
L_a    = 16.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.85;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.15;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 4;  % number of fuel tanks
N_c = 2;                          % number of crew
% N_ci = 2.0;                       % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;                          % number of flight control systems
% N_u = 8;                          % number of hydraulic utility functions (typically 5-15)
Nf = 4;                           % number of functions performed by controls (typically 4-7)
Nm = 2;                           % number of mechanical functions (typically 0-2)
W_uav = 600*9.81;  % uninstalled avionics weight, [N in this deck]

%% ---- nacelle inputs (metric) ----
K_ng     = 1.017;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 4.0;  % nacelle length, [m in this deck]
N_w      = 1.30;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = 13.12;  % nacelle length,
N_w_ft   = 4.20;  % nacelle width,
S_n_ft   = 194;  % nacelle wetted area,
W_en_lb  = W_eng_e*.225;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 3000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 2000;  % Takeoff field length [m]
Landing_field_length = 1400;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude used in carpet plot [m]
acc = 1.5;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=42.8E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 10000;  % Minimum search altitude [m]
max_alt = 12000;  % Maximum search altitude [m]
WdS_max = 7000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20;                       % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                      % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;                       % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                       % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;                        % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;                      % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;                      % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;                     % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;                      % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max = 30;                     % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max = 30;                      % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;                      % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;                 % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low = 0;                % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;             % Upper bound for existing engine afterburner fuel fraction flow rate

elseif aircraft_case==4

% ==================== MISSION REQUIREMENTS (777-200ER) ====================
M = 0.84;  % Cruise Mach number [-]
Range_cruise = 13080000;  % Cruise range [m]
TOGW_i(1) = 297550*9.81;  % Initial takeoff gross weight [N]
W_payload = 15000*9.81;  % Payload weight [N]
W_e(1)    = 145150*9.81;  % Initial empty operating weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload;  % Initial fuel weight [N]
Np        = 301;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% B777-200ER VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 427.8;  % Reference wing area [m^2]
b0      = 60.93;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.21));  % Reference root chord [m]
Lambda0 = 31.6*pi/180;  % Reference wing sweep [rad]
Alt0    = 11550;  % Reference cruise altitude [m]
R0      = 13080e3;  % Reference range [m]
L_D     = 18.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 0.54*2.832e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 31.64*pi/180;

% ==================== PROPULSION (2 x GE90-94B class) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 7.66;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1400;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1750;  % High design turbine inlet temperature [K]
T_0_6_high  = 1750;  % High afterburner total temperature [K]
T_0_3_sl    = 700;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 40;  % Minimum compressor pressure ratio [-]
CPR_max     = 40;  % Maximum compressor pressure ratio [-]
BR_min      = 8.4;  % Minimum bypass ratio [-]
BR_max      = 8.4;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.98;  % Inlet efficiency [-]
eta_c       = 0.88;  % Compressor efficiency [-]
eta_f       = 0.91;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.995;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.98;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.03;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 417000;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 70000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 417000;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 70000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 800;  % Engine low turbine inlet temperature [K]
TIT_e          = 1750;  % Engine high turbine inlet temperature [K]
CPR_e          = 40;  % Engine compressor pressure ratio [-]
BR_e           = 8.4;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]
eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.91;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.88;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 7.66;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1750;  % Existing engine turbine temperature [K]
W_eng_e        = 7550*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.38;  % Airfoil maximum thickness location as a fraction of the chord length [-]
% cl_max       = 2.6;          % 2-D section CLmax (flaps)
% CL_max       = 2.5;          % 3-D wing CLmax landing — sizes S
tdc          = 0.12;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 101;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 12;  % Nose half angle (useful for waverider configuration)
sweep        = 31.6*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.21;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
C_p_0        = -1.5;    %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil


% ============================================================
% B777-200ER TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 75.0;  % Horizontal-tail planform area [m^2]
b_H       = 28.0;  % Horizontal-tail span [m]
sweep_H   = 31*pi/180;  % Horizontal-tail leading edge sweep angle [rad]

c_H       = 5.5;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 52.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 12.8;  % Vertical-fin height/span [m]
sweep_V   = 37*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
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
D_f    = 6.20;  % Fuselage maximum diameter [m]
L      = 63.73;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 6.20;  % Fuselage width [m]
S_csw  = 55.0;  % Elevon/control-surface area [m^2]
F_w    = 6.20;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 32.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 4.20;  % length of main landing gear, [m in this deck]
L_n  = 3.20;  % nose gear length, [m in this deck]
N_mw = 12;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 1.0;  % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;  % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;  % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;  % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;  % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;  % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1.0;  % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;  % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;  % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;  % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;  % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;  % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;  % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 3.12;  % engine diameter, [m in this deck]
L_d     = 3.5;  % duct length, [m in this deck]
L_tp    = 3.0;  % length of tailpipe, [m in this deck]
L_sh    = 2.0;  % length of engine shroud, [m in this deck]
L_a    = 28.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 4;  % number of fuel tanks
N_c = 2;  % number of crew
% N_ci = 2.0;  % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;  % number of flight control systems
% N_u = 10;  % number of hydraulic utility functions (typically 5-15)
Nf = 5;  % number of functions performed by controls (typically 4-7)
Nm = 2;  % number of mechanical functions (typically 0-2)
W_uav = 1200*9.81;  % uninstalled avionics weight, [N in this deck]
% K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
% R_kva = 120;  % system electrical rating, kv·A (typically 40-60, or 160)

%% ---- nacelle 15.31 (pylon GE90) ----
K_ng     = 1.017;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 7.3;  % nacelle length, [m in this deck]
N_w      = 3.90;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 2500/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3200;  % Takeoff field length [m]
Landing_field_length = 1700;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude used in carpet plot [m]
acc = 1.2;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 10000;  % Minimum search altitude [m]
max_alt = 12000;  % Maximum search altitude [m]
WdS_max = 8000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20;                       % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                      % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;                       % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                       % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;                        % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;                      % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;                      % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;                     % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;                      % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max = 30;                     % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max = 30;                      % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;                      % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low = 0;  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate
 
elseif aircraft_case==5
 
% ==================== MISSION REQUIREMENTS (A320-200) ====================
M = 0.82;  % Cruise Mach number [-]
Range_cruise = 5800000;  % Cruise range [m]
TOGW_i(1) = 77000*9.81;  % Initial takeoff gross weight [N]
W_payload = 8000*9.81;  % Payload weight [N]
W_e(1)    = 42200*9.81;  % Initial empty operating weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 180;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% A320-200 VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 122.6;  % Reference wing area [m^2]
b0      = 34.10;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.24));  % Reference root chord [m]
Lambda0 = 25*pi/180;  % Reference wing sweep [rad]
Alt0    = 11550;  % Reference cruise altitude [m]
R0      = 6100e3;  % Reference range [m]
L_D     = 16.5;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 0.60*2.832e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0=27*pi/180;

% ==================== PROPULSION (2 x CFM56-5B class) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 2.35;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1550;  % High design turbine inlet temperature [K]
T_0_6_high  = 1550;  % High afterburner total temperature [K]
T_0_3_sl    = 600;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 32;  % Minimum compressor pressure ratio [-]
CPR_max     = 32;  % Maximum compressor pressure ratio [-]
BR_min      = 5.5;  % Minimum bypass ratio [-]
BR_max      = 5.5;  % Maximum bypass ratio [-]
FPR         = 1.7;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.98;  % Inlet efficiency [-]
eta_c       = 0.87;  % Compressor efficiency [-]
eta_f       = 0.90;  % Fan efficiency [-]
eta_t       = 0.89;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.98;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.07;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 120100;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 28000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 120100;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 28000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1550;  % Engine high turbine inlet temperature [K]
CPR_e          = 32;  % Engine compressor pressure ratio [-]
BR_e           = 5.5;  % Engine bypass ratio [-]
FPR_e          = 1.7;  % Engine fan pressure ratio [-]
eta_t_e        = 0.89;  % Engine turbine efficiency [-]
eta_f_e        = 0.90;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.87;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 2.35;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1550;  % Existing engine turbine temperature [K]
W_eng_e        = 2380*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.37;  % Airfoil maximum thickness location as a fraction of the chord length [-]
% cl_max       = 2.6;          % 2-D flapped
% CL_max       = 2.5;          % 3-D landing — sizes S
tdc          = 0.125;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 92;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 12;  % Nose half angle (useful for waverider configuration)
sweep        = 27*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.24;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
C_p_0        = -1.4;    %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil


% ============================================================
% A320-200 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 31.0;  % Horizontal-tail planform area [m^2]
b_H       = 12.0;  % Horizontal-tail span [m]
sweep_H   = 28*pi/180;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 2.75;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 21.5;  % Vertical-tail/fin planform area [m^2]
b_V       = 7.2;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 3.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                        % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                   % Sea-level air density [kg/m^3]
T_sl = 288.15;                    % Sea-level standard temperature [K]
R = 287.05;                       % Specific gas constant for air [J/(kg*K)]
y = 1.4;                          % Ratio of specific heats [-]
g = 9.81;                         % Gravitational acceleration [m/s^2]
p_sl = 101300;                    % Sea-level standard pressure [Pa]
radius_e = 6.371e6;               % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 4.14;  % Fuselage maximum diameter [m]
L      = 37.57;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 4.14;  % Fuselage width [m]
S_csw  = 22.0;  % Elevon/control-surface area [m^2]
F_w    = 4.14;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 17.5;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 2.80;  % length of main landing gear, [m in this deck]
L_n  = 2.10;  % nose gear length, [m in this deck]
N_mw = 4;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 1.0;                       % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                       % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;                      % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                      % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                      % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                    % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1.0;                       % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                       % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                      % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                       % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                       % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;                        % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;                     % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 1.73;  % engine diameter, [m in this deck]
L_d     = 2.5;  % duct length, [m in this deck]
L_tp    = 2.2;  % length of tailpipe, [m in this deck]
L_sh    = 1.2;  % length of engine shroud, [m in this deck]
L_a    = 18.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.88;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.12;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 4;  % number of fuel tanks
N_c = 2;                          % number of crew
% N_ci = 2.0;                       % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;                          % number of flight control systems
% N_u = 8;                          % number of hydraulic utility functions (typically 5-15)
Nf = 5;                           % number of functions performed by controls (typically 4-7)
Nm = 2;                           % number of mechanical functions (typically 0-2)
W_uav = 700*9.81;  % uninstalled avionics weight, [N in this deck]
% K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
% R_kva = 90;  % system electrical rating, kv·A (typically 40-60, or 160)

%% ---- nacelle 15.31 (pylon CFM56) ----
K_ng     = 1.017;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 5.2;  % nacelle length, [m in this deck]
N_w      = 2.10;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 2500/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 1800/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 2100;  % Takeoff field length [m]
Landing_field_length = 1500;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude used in carpet plot [m]
acc = 1.3;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 10000;  % Minimum search altitude [m]
max_alt = 14500;  % Maximum search altitude [m]
WdS_max = 7000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 10;  % Number of altitude points shown in design point plot [-]
i_max = 20;                       % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;                      % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;                       % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;                       % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;                        % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max = 15;                      % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max = 15;                      % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max = 30;                     % number of points used for the carpet plot that has smoothed out curves
xx_max = 20;                      % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max = 30;                     % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max = 30;                      % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max = 30;                      % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max = 30;  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low = 0;                % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high = 0.5;  % Upper bound for existing engine afterburner fuel fraction flow rate

elseif aircraft_case==6
    
% ==================== MISSION REQUIREMENTS (Citation CJ3) ====================
M = 0.70;  % Cruise Mach number [-]
Range_cruise = 3500000;  % Cruise range [m]
TOGW_i(1) = 6291*9.81;  % Initial takeoff gross weight [N]
W_e(1)    = 3874*9.81;  % Initial empty operating weight [N]
W_payload = 400*9.81;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 6;  % Passenger capacity [-]
type2     = 1;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% CJ-3 VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 27.32;  % Reference wing area [m^2]
b0      = 16.26;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.30));  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]
Alt0    = 13716;  % Reference cruise altitude [m]
R0      = 3780e3;  % Reference range [m]
L_D     = 13.0;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 1.898e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 0;

% ==================== PROPULSION (2 x Williams FJ44-3A) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.25;  % Inlet capture area per engine [m^2]
TIT_d_low   = 700;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1400;  % High afterburner total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 18;  % Minimum compressor pressure ratio [-]
CPR_max     = 18;  % Maximum compressor pressure ratio [-]
BR_min      = 3;  % Minimum bypass ratio [-]
BR_max      = 3;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.97;  % Inlet efficiency [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.35;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 12544;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 2500;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 12544;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 2500;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 700;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 18;  % Engine compressor pressure ratio [-]
BR_e           = 3;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 0.25;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1400;  % Existing engine turbine temperature [K]
W_eng_e        = 240*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.35;  % Airfoil maximum thickness location as a fraction of the chord length [-]
% cl_max       = 1.8;
% CL_max       = 1.7;          % landing flaps — sizes S
tdc          = 0.14;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 54;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 10;  % Nose half angle (useful for waverider configuration)
sweep        = 0*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.30;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
C_p_0        = -.9;    %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil


% ============================================================
% CESSNA CITATION CJ3 TAIL INPUTS
% ============================================================

% Horizontal tail
S_H       = 8.5;  % Horizontal-tail planform area [m^2]
b_H       = 5.5;  % Horizontal-tail span [m]
sweep_H   = 25*pi/180;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 1.55;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 5.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 3.0;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 1.75;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                        % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                   % Sea-level air density [kg/m^3]
T_sl = 288.15;                    % Sea-level standard temperature [K]
R = 287.05;                       % Specific gas constant for air [J/(kg*K)]
y = 1.4;                          % Ratio of specific heats [-]
g = 9.81;                         % Gravitational acceleration [m/s^2]
p_sl = 101300;                    % Sea-level standard pressure [Pa]
radius_e = 6.371e6;               % Earth radius [m]
mu_0 = 1.716e-5;  % Reference dynamic viscosity [Pa*s]
T_0  = 273.15;  % Reference temperature [K]
Su    = 110.4;  % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.80;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 1.50;  % Fuselage maximum diameter [m]
L      = 15.59;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 1.50;  % Fuselage width [m]
S_csw  = 4.0;  % Elevon/control-surface area [m^2]
F_w    = 1.50;  % Fuselage width [m]
Ht_Hv = 1;  % 0 for conventional tail, 1 for T tail
L_t    = 7.5;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]
L_m  = 1.40;  % length of main landing gear, [m in this deck]
L_n  = 1.10;  % nose gear length, [m in this deck]
N_mw = 2;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 1.0;                       % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                       % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;                      % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                      % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                      % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.0;                     % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;                         % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                       % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;                      % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;                       % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                       % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;                        % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;                     % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)
D_e     = 0.56;  % engine diameter, [m in this deck]
L_d     = 1.2;  % duct length, [m in this deck]
L_tp    = 1.0;  % length of tailpipe, [m in this deck]
L_sh    = 0.8;  % length of engine shroud, [m in this deck]
L_a    = 10.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 2;  % number of fuel tanks
N_c = 2;                          % number of crew
% N_ci = 2.0;                       % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;                          % number of flight control systems
% N_u = 6;                          % number of hydraulic utility functions (typically 5-15)
Nf = 4;                           % number of functions performed by controls (typically 4-7)
Nm = 2;                           % number of mechanical functions (typically 0-2)
W_uav = 200*9.81;                 % uninstalled avionics weight, [N in this deck]
% K_vsh = 1.0;                      % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc = 1.0;                       % 1.45 if mission completion required after failure; = 1.0 otherwise
% R_kva = 20;                       % system electrical rating, kv·A (typically 40-60, or 160)

%% ---- nacelle (aft-fuse pylons) ----
K_ng     = 1.017;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.0;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 2.2;  % nacelle length, [m in this deck]
N_w      = 0.70;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== AEROTHERMAL (unused at M=0.70; kept for code) ====================
TPS=0;
% R_wle = 0.02;  % Wing leading-edge radius [ft]
% g_s=g*3.2797;  % Gravity conversion parameter [ft/s^2]
% J=778;  % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity=0.8;  % Radiative emissivity [-]
% Cp_wall=1000;  % Wall/TPS specific heat [J/(kg*K)]
% k_wall=21;  % Aircraft wall thermal conductivity [W/(m*K)]
% k_ins=.3;  % Insulation thermal conductivity [W/(m*K)]
% t_ins=.051;  % Insulation thickness [m]
% t_wall=.001;  % Wall skin thickness [m]
% Cp_ins=500;  % Insulation specific heat [J/(kg*K)]
% rho_ins=112;  % Insulation density [kg/m^3]
% S_cooling=0;  % Active cooling surface area [m^2]
% rho_cooling=300;  % Active cooling surface areal density [kg/m^2]
% x = 4;  % Heating-analysis longitudinal location [m]
% Pr = 0.72;  % Prandtl number [-]

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
alt_0 = 0;  % Initial altitude used in carpet plot [m]
acc = 1.5;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 13000;  % Minimum search altitude [m]
max_alt = 16500;  % Maximum search altitude [m]
WdS_max = 4000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 20;  % Number of altitude points shown in design point plot [-]
i_max=20;                         % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;                        % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max=20;                         % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;                         % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max=50;                          % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max=15;                        % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max=15;                        % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max=30;                       % number of points used for the carpet plot that has smoothed out curves
xx_max=20; iii_max=30;            % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
ff_max=30;                        % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max=30;                        % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max=30;                   % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low=0;                  % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high=0.5;               % Upper bound for existing engine afterburner fuel fraction flow rate

    
elseif aircraft_case==7
        
% ==================== MISSION REQUIREMENTS (Dassault Falcon 2000EX) ====================
M = 0.80;  % Cruise Mach number [-]
Range_cruise = 6800000;  % Cruise range [m]
TOGW_i(1) = 19141*9.81;  % Initial takeoff gross weight [N]
W_e(1)    = 10487*9.81;  % Initial empty operating weight [N]
W_payload = 1000*4.44822;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 10;  % Passenger capacity [-]
type2     = 0;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% 2000 EX VERIFICATION AIRCRAFT DATA
% ============================================================
S0      = 49.00;  % Reference wing area [m^2]
b0      = 19.33;  % Reference wingspan [m]
c_r0    = 2*S0/(b0*(1+0.30));  % Reference root chord [m]
Lambda0 = 0;  % Reference wing sweep [rad]
Alt0    = 14326;  % Reference cruise altitude [m]
R0      = 7180000;  % Reference range [m]
L_D     = 13.0;  % Reference cruise lift-to-drag ratio [-]
TSFC0   = 1.84e-5;  % Reference cruise TSFC [kg/(N*s)]
sweep0  = 24.5*pi/180;

% ==================== PROPULSION (2 x Pratt & Whitney Canada PW308C) ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 0;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.56;  % Inlet capture area per engine [m^2]
TIT_d_low   = 900;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1400;  % High afterburner total temperature [K]
T_0_3_sl    = 500;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 25.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 25.0;  % Maximum compressor pressure ratio [-]
BR_min      = 3.5;  % Minimum bypass ratio [-]
BR_max      = 3.5;  % Maximum bypass ratio [-]
FPR         = 1.6;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0;  % High afterburner setting [-]
eta_inlet   = 0.97;  % Inlet efficiency [-]
eta_c       = 0.84;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.88;  % Turbine efficiency [-]
eta_b       = 0.99;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.99;  % Mechanical efficiency [-]
eta_n       = 0.97;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.37;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
Th_wet         = 31140;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 7030;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 31140;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 7030;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 900;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 25.0;  % Engine compressor pressure ratio [-]
BR_e           = 3.5;  % Engine bypass ratio [-]
FPR_e          = 1.6;  % Engine fan pressure ratio [-]
eta_t_e        = 0.88;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.84;  % Engine compressor efficiency [-]
eta_m_e        = 0.99;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 0.56;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1400;  % Existing engine turbine temperature [K]
W_eng_e        = 1336*4.44822;  % Engine weight per engine [N] 
    
% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 12;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.35;  % Airfoil maximum thickness location as a fraction of the chord length [-]
% cl_max       = 1.6;
% CL_max       = 1.7;
tdc          = 0.12;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 75;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 10;  % Nose half angle (useful for waverider configuration)
sweep        = 24.5*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.30;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
C_p_0        = -1.2;    %Minimum zero sweep pressure coefficient obtained from XFOIL of BAC XXX airfoil


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
sweep_H   = 28*pi/180;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 1.70;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0.10;  % Horizontal-tail thickness-to-chord ratio [-]

% ============================================================
% Vertical tail / fin
% ============================================================
S_V       = 8.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 4.5;  % Vertical-fin height/span [m]
sweep_V   = 35*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 2.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.10;  % Vertical-tail maximum thickness-to-chord ratio [-]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.80;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 2.34;  % Fuselage maximum diameter [m]
L      = 20.23;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 2.34;  % Fuselage width [m]
S_csw  = 5.0;  % Elevon/control-surface area [m^2]
F_w    = 2.34;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 8.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.30;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]

% ==================== LANDING GEAR ====================
L_m  = 1.40;  % length of main landing gear, [m in this deck]
L_n  = 1.10;  % nose gear length, [m in this deck]
N_mw = 4;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts
K_dw = 1.0;  % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;  % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;  % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;  % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;  % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.0;  % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;  % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;  % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;  % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;  % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;  % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;  % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;  % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)

% ==================== PROPULSION / NACELLE ====================
D_e     = 1.299;  % engine diameter, [m in this deck]
L_d     = 1.5;  % duct length, [m in this deck]
L_tp    = 1.2;  % length of tailpipe, [m in this deck]
L_sh    = 1.0;  % length of engine shroud, [m in this deck]
L_a    = 10.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 2;  % number of fuel tanks
N_c = 2;  % number of crew
% N_ci = 2.0;  % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
% N_s = 2;  % number of flight control systems
% N_u = 6;  % number of hydraulic utility functions (typically 5-15)
Nf = 4;  % number of functions performed by controls (typically 4-7)
Nm = 2;  % number of mechanical functions (typically 0-2)
W_uav = 300*9.81;  % uninstalled avionics weight, [N in this deck]
% K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
% R_kva = 15;  % system electrical rating, kv·A (typically 40-60, or 160)

% ==================== NACELLE (AFT-FUSE) ====================
K_ng     = 1.017;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p      = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr     = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt     = 2.2;  % nacelle length, [m in this deck]
N_w      = 1.00;  % nacelle width, [m in this deck]
S_n      = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en     = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft  = N_Lt/0.3048;  % nacelle length,
N_w_ft   = N_w/0.3048;  % nacelle width,
S_n_ft   = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb  = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 4375/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 2000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 1700;  % Takeoff field length [m]
Landing_field_length = 650;  % Landing field length [m]
g_turn = 1.3;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 0;  % Initial altitude used in carpet plot [m]
acc = 1.5;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== FUEL PROPERTIES ====================
E_sp=43.15E6;  % Fuel specific energy [J/kg]
ratio_fo=.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 13000;  % Minimum search altitude [m]
max_alt = 14000;  % Maximum search altitude [m]
WdS_max = 4000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 20;  % Number of altitude points shown in design point plot [-]
i_max=20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;  % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max=20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;  % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max=50;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
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
W_payload = 0*9.81;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 0;  % Passenger capacity [-]
type2     = 2;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

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

% ==================== PROPULSION ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 6;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 0;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 1;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 1.204;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1900;  % High afterburner total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 8.75;  % Minimum compressor pressure ratio [-]
CPR_max     = 8.75;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.5;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.85;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.7;  % Engine/nacelle drag coefficient [-] %This value is high since the inlets were intentionally designed to create a shockwave for compression lift
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
% General Electric YJ93 turbojet
Th_wet         = 128000;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 115000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 85000;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 80000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 8.75;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.85;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 1.204;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1900;  % Existing engine turbine temperature [K]
W_eng_e        = 2700*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
% Subsonic_airfoil = 5;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 1;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.50;  % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max       = 1.20;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 0.80;  % Aircraft maximum lift coefficient [-]
tdc          = 0.025;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 90;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 3;  % Nose half angle (useful for waverider configuration)
sweep        = 65*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.15;  % Wing taper ratio [-]
droop        = 1;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]
% S(q)      = 490; %Top view wing area with droop applied

% ==================== XB-70 TAIL INPUTS ====================

% ------------------------------------------------------------
% Horizontal tail
% ------------------------------------------------------------
% XB-70 has no conventional horizontal tail.
% Pitch control is provided by the canards/elevons.

S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0;  % Horizontal-tail thickness-to-chord ratio [-]

% ------------------------------------------------------------
% Vertical tails / fins
% ------------------------------------------------------------

% Two vertical fins are mounted on the outboard wing/nacelle region.
S_V       = 38.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 8.0;  % Vertical-fin height/span [m]
sweep_V   = 55*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 4.75;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.06;  % Vertical-tail maximum thickness-to-chord ratio [-]


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
S_fw   = 0;                     % Firewall surface area [m^2]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;  % Ultimate load factor / structural factor [-]
N_l = 3.0;  % Landing load factor / structural factor [-]
D_f    = 3.05;  % Fuselage maximum diameter [m]
L      = 56.64;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 3.05;  % Fuselage width [m]
skin=3; %1=aluminum, 2=titanium, 3=stainless steel
S_csw  = 50.0;  % Elevon/control-surface area [m^2]
F_w    = 3.05;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 20.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]

% ==================== FUSELAGE / LANDING GEAR ====================
L_m  = 3.50;  % length of main landing gear, [m in this deck]
L_n  = 2.80;  % nose gear length, [m in this deck]
N_mw = 8;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts

% ==================== RAYMER CONSTANTS ====================
K_dw = 0.768;  % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;  % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 0.774;  % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;  % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;  % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;  % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;  % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;  % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;  % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;  % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;  % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;  % 1.62 for variable geometry; = 1.0 otherwise
K_d = 2.75;  % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0;  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)

% ==================== ENGINE / NACELLE ====================
D_e     = 1.334;  % engine diameter, [m in this deck]
L_d     = 8.0;  % duct length, [m in this deck]
L_tp    = 2.0;  % length of tailpipe, [m in this deck]
L_sh    = 1.5;  % length of engine shroud, [m in this deck]
L_a    = 25.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]

% ==================== FUEL / VOLUME ====================
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 17;  % number of fuel tanks

% ==================== SYSTEMS ====================
N_c  = 3;  % number of crew
N_ci = 2.0;  % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
N_s  = 2;  % number of flight control systems
N_u  = 6;  % number of hydraulic utility functions (typically 5-15)
Nf = 5;  % number of functions performed by controls (typically 4-7)
Nm = 2;  % number of mechanical functions (typically 0-2)
W_uav = 800*9.81;  % uninstalled avionics weight, [N in this deck]
K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
R_kva = 80;  % system electrical rating, kv·A (typically 40-60, or 160)

% ==================== NACELLE / INTAKE ====================
K_ng = 1.0;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p  = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt = 8.0;  % nacelle length, [m in this deck]
N_w  = 1.238;  % nacelle width, [m in this deck]
S_n = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft = N_Lt/0.3048;  % nacelle length,
N_w_ft  = N_w/0.3048;  % nacelle width,
S_n_ft = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 27000/3.28/60;  % Rate of climb [m/s]
rate_of_descent  = 27000/3.28/60;  % Rate of descent [m/s]
Takeoff_field_length = 3200;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude used in carpet plot [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS = 1;  % Thermal protection system: 0=no, 1=yes
R_wle = 0.344/12;  % Wing leading-edge radius [ft]
g_s = g*3.2797;  % Gravity conversion parameter [ft/s^2]
J = 778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity = 0.8;  % Radiative emissivity [-]
Cp_wall = 1000;  % Wall/TPS specific heat [J/(kg*K)]
k_wall = 21;  % Aircraft wall thermal conductivity [W/(m*K)]
k_ins = 0.3;  % Insulation thermal conductivity [W/(m*K)]
t_ins = 0;  % Insulation thickness [m] (set to 0 to avoid double counting the weight)
t_wall = 0.0015;  % Wall skin thickness [m]
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
min_alt = 22000;  % Minimum search altitude [m]
max_alt = 24000;  % Maximum search altitude [m]
WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 200;  % Number of altitude points shown in design point plot [-]
i_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;  % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;  % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
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

elseif aircraft_case==9
    
% ============================================================
% SR-71 BLACKBIRD VERIFICATION AIRCRAFT
% ============================================================

% ==================== MISSION REQUIREMENTS ====================

M = 3.20;  % Cruise Mach number [-]
Range_cruise = 3200000;  % Cruise range [m]
TOGW_i(1) = 78000*9.81;  % Initial takeoff gross weight [N]
W_e(1)    = 30600*9.81;  % Initial empty operating weight [N]
W_payload = 1600*9.81;  % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np        = 0;  % Passenger capacity [-]
type2     = 2;  % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

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

% ==================== PROPULSION ====================
type        = 0;  % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en        = 2;  % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner = 1;  % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet    = 2;  % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider   = 0;  % Waverider configuration: 0 = disabled, 1 = enabled
A_inlet     = 0.90;  % Inlet capture area per engine [m^2]
TIT_d_low   = 1200;  % Low design turbine inlet temperature [K]
TIT_d_high  = 1400;  % High design turbine inlet temperature [K]
T_0_6_high  = 1900;  % High afterburner total temperature [K]
T_0_3_sl    = 550;  % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min     = 8.0;  % Minimum compressor pressure ratio [-]
CPR_max     = 8.0;  % Maximum compressor pressure ratio [-]
BR_min      = 0;  % Minimum bypass ratio [-]
BR_max      = 0;  % Maximum bypass ratio [-]
FPR         = 0;  % Fan pressure ratio [-]
n_ab_low    = 0;  % Low afterburner setting [-]
n_ab_high   = 0.5;  % High afterburner setting [-]
eta_inlet   = 0.92;  % Inlet efficiency [-]
eta_c       = 0.85;  % Compressor efficiency [-]
eta_f       = 0.88;  % Fan efficiency [-]
eta_t       = 0.90;  % Turbine efficiency [-]
eta_b       = 0.98;  % Burner/combustor efficiency [-]
eta_ab      = 0.90;  % Afterburner efficiency [-]
eta_m       = 0.98;  % Mechanical efficiency [-]
eta_n       = 0.94;  % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency
Cd_engine   = 0.1;  % Engine/nacelle drag coefficient [-]
rho_f       = 800;  % Fuel density [kg/m^3]

% ==================== EXISTING ENGINE DATA ====================
% Pratt & Whitney J58
Th_wet         = 151000;  % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise  = 140000;  % Maximum thrust with afterburner at cruise per engine [N]
Th_dry         = 107000;  % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise  = 100000;  % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low      = 1200;  % Engine low turbine inlet temperature [K]
TIT_e          = 1400;  % Engine high turbine inlet temperature [K]
CPR_e          = 8.0;  % Engine compressor pressure ratio [-]
BR_e           = 0;  % Engine bypass ratio [-]
FPR_e          = 0;  % Engine fan pressure ratio [-]
eta_t_e        = 0.90;  % Engine turbine efficiency [-]
eta_f_e        = 0.88;  % Engine fan efficiency [-]
eta_ab_e       = 0.90;  % Engine afterburner efficiency [-]
eta_c_e        = 0.85;  % Engine compressor efficiency [-]
eta_m_e        = 0.98;  % Engine mechanical efficiency [-]
eta_b_e=.98;                   % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e      = 0.90;  % Existing engine inlet area [m^2]
T_0_6_k_e      = 1900;  % Existing engine turbine temperature [K]
W_eng_e        = 2720*9.81;  % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
% Subsonic_airfoil = 5;  % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta        = 0;  % 0=no delta wing, 1=delta wing
sweep_solver = 0;  % 0=use input, 1=solve for (wing)
x_c          = 0.50;  % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max       = 1.20;  % 2-D airfoil maximum lift coefficient [-]
CL_max       = 0.80;  % Aircraft maximum lift coefficient [-]
tdc          = 0.03;  % Maximum thickness-to-chord ratio [-]
V_stall_sl   = 95;  % Sea-level fully loaded flaps up stall speed [m/s]
u_0          = 1.7894e-5;  % Reference dynamic viscosity [Pa*s]
nose_angle   = 3;  % Nose half angle (useful for waverider configuration)
sweep        = 60*pi/180;  % Wing leading edge sweep angle [rad]
lambda       = 0.01;  % Wing taper ratio [-]
droop        = 0;  % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]

% ==================== SR-71 TAIL INPUTS ====================

% ------------------------------------------------------------
% Horizontal tail
% ------------------------------------------------------------
S_H       = 0;  % Horizontal-tail planform area [m^2]
b_H       = 0;  % Horizontal-tail span [m]
sweep_H   = 0;  % Horizontal-tail leading edge sweep angle [rad]
c_H       = 0;  % Horizontal-tail mean aerodynamic chord [m]
x_t_H     = 0.30;  % Horizontal-tail maximum thickness location, x/c [-]
t_c_H     = 0;  % Horizontal-tail thickness-to-chord ratio [-]

% ------------------------------------------------------------
% Vertical tails
% ------------------------------------------------------------

% Two large canted vertical stabilizers
S_V       = 23.0;  % Vertical-tail/fin planform area [m^2]
b_V       = 5.0;  % Vertical-fin height/span [m]
sweep_V   = 15*pi/180;  % Vertical-fin leading-edge sweep angle [rad]
c_V       = 4.0;  % Vertical-tail mean aerodynamic chord [m]
x_t_V     = 0.30;  % Vertical-tail maximum thickness location, x/c [-]
t_c_V     = 0.06;  % Vertical-tail maximum thickness-to-chord ratio [-]

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
D_f    = 1.6;  % Fuselage maximum diameter [m]
L      = 32.74;  % Fuselage/aircraft length (0 = solve for) [m]
W_fuse = 1.7;  % Fuselage width [m]
skin=2; %1=aluminum, 2=titanium, 3=stainless steel
W_chine=2500/.225; %Chine weight [N] (obtained using AI)
S_fw   = 0;                     % Firewall surface area [m^2]

% ==================== MAIN WING ====================
S_csw  = 20.0;  % Elevon/control-surface area [m^2]
F_w    = 1.70;  % Fuselage width [m]
Ht_Hv = 0;  % 0 for conventional tail, 1 for T tail
L_t    = 10.0;  % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt = 0.35;  % Rudder/control surface to vertical-tail area ratio [-]
S_cs   = S_csw + Sr_Svt*S_V;  % Total control-surface area [m^2]

% ==================== FUSELAGE / LANDING GEAR ====================
L_m  = 3.50;  % length of main landing gear, [m in this deck]
L_n  = 2.80;  % nose gear length, [m in this deck]
N_mw = 6;  % number of main wheels
N_nw = 2;  % number of nose wheels
N_mss = 2;  % number of main gear shock struts

% ==================== RAYMER CONSTANTS ====================
K_dw = 0.768;  % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;  % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 0.774;  % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;  % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;  % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;  % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg = 1;  % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;  % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg = 1.0;  % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp = 1.0;  % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.0;  % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;  % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1;  % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld = 0.40;  % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)

% ==================== ENGINE / NACELLE ====================
D_e     = 1.3;  % engine diameter, [m in this deck]
L_d     = 4.57;  % duct length, [m in this deck]
L_tp    = 1.5;  % length of tailpipe, [m in this deck]
L_sh    = 1.0;  % length of engine shroud, [m in this deck]
L_a    = 15.0;  % electrical routing distance, generators to avionics to cockpit, [m in this deck]

% ==================== FUEL / VOLUME ====================
Vi_Vt = 0.90;  % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt = 0.10;  % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t   = 9;  % number of fuel tanks

% ==================== SYSTEMS ====================
N_c  = 3;  % number of crew
N_ci = 2.0;  % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
N_s  = 2;  % number of flight control systems
N_u  = 6;  % number of hydraulic utility functions (typically 5-15)
Nf = 5;  % number of functions performed by controls (typically 4-7)
Nm = 2;  % number of mechanical functions (typically 0-2)
W_uav = 800*9.81;  % uninstalled avionics weight, [N in this deck]
K_vsh = 1.0;  % 1.425 if variable sweep wing; = 1.0 otherwise
K_mc  = 1.0;  % 1.45 if mission completion required after failure; = 1.0 otherwise
R_kva = 80;  % system electrical rating, kv·A (typically 40-60, or 160)

% ==================== NACELLE / INTAKE ====================
K_ng = 1.0;  % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p  = 1.0;  % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr = 1.18;  % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt = 4.57;  % nacelle length, [m in this deck]
N_w  = 1.27;  % nacelle width, [m in this deck]
S_n = pi*N_w*N_Lt;  % nacelle wetted area, [m^2 in this deck]
W_en = W_eng_e;  % engine weight, each, [N in this deck]
N_Lt_ft = N_Lt/0.3048;  % nacelle length,
N_w_ft  = N_w/0.3048;  % nacelle width,
S_n_ft = S_n/0.3048^2;  % nacelle wetted area,
W_en_lb = W_eng_e/4.44822;  % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb    = 60.0;  % Rate of climb [m/s]
rate_of_descent  = 7.0;  % Rate of descent [m/s]
Takeoff_field_length = 3000;  % Takeoff field length [m]
Landing_field_length = 2200;  % Landing field length [m]
g_turn = 1.2;  % Turning/acceleration load factor [-]
mu_2 = 0.03;  % Ground friction coefficient [-]
alt_0 = 1000;  % Initial altitude used in carpet plot [m]
acc = 2.0;  % Acceleration [m/s^2]
V_lof = 1.1*V_stall_sl;  % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS = 1;  % Thermal protection system: 0=no, 1=yes
R_wle = 0.344/12;  % Wing leading-edge radius [ft]
g_s = g*3.2797;  % Gravity conversion parameter [ft/s^2]
J = 778;  % Mechanical equivalent of heat [ft*lbf/BTU]
emmissivity = 0.8;  % Radiative emissivity [-]
Cp_wall = 1000;  % Wall/TPS specific heat [J/(kg*K)]
k_wall = 21;  % Aircraft wall thermal conductivity [W/(m*K)]
k_ins = 0.3;  % Insulation thermal conductivity [W/(m*K)]
t_ins = 0;  % Insulation thickness [m]
t_wall = 0.0015;  % Wall skin thickness [m]
Cp_ins = 500;  % Insulation specific heat [J/(kg*K)]
rho_ins = 112;  % Insulation density [kg/m^3]
S_cooling = 0;  % Active cooling surface area [m^2]
rho_cooling = 300;  % Active cooling surface areal density [kg/m^2]
x = 4;  % Heating-analysis longitudinal location [m]
Pr = 0.72;  % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp = 43E6;  % Fuel specific energy [J/kg]
ratio_fo = 0.294;  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f = 2010;  % Fuel specific heat [J/(kg*K)]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 18000;  % Minimum search altitude [m]
max_alt = 25000;  % Maximum search altitude [m]
WdS_max = 6000;  % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 200;  % Number of altitude points shown in design point plot [-]
i_max = 20;  % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max = 25;  % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max = 20;  % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max = 20;  % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max = 50;  % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
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

elseif aircraft_case==10
       
% ============================================================
% L-2000 SST VERIFICATION AIRCRAFT
% ============================================================

% ==================== MISSION REQUIREMENTS ====================
M = 2.7;                         % Cruise Mach number [-]
Range_cruise = 5000000;          % Cruise range [m]
TOGW_i(1) = 267619*9.81;         % Initial takeoff gross weight [N]
W_e(1)    = 107955*9.81;         % Initial empty operating weight [N]
W_payload = 26750*9.81;        % Payload weight [N]
W_f(1)    = TOGW_i(1)-W_e(1)-W_payload;  % Initial fuel weight [N]
Np = 273;                        % Passenger capacity [-]
type2 = 2;                       % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% L-2000 VERIFICATION AIRCRAFT DATA
% ============================================================
S0=875.5;                        % Reference wing area [m^2]
b0=35.36;                        % Reference wingspan [m]
c_r0=27.66;                      % Reference root chord [m]
Lambda0=60*pi/180;               % Reference wing sweep [rad]
Alt0=23300;                      % Reference cruise altitude [m]
R0=6400E3;7400e3;                % Reference range [m]
L_D=8.0;                         % Reference cruise lift-to-drag ratio [-]
TSFC0=.0000442;                  % Reference cruise TSFC [kg/(N*s)]

% ==================== PROPULSION ====================
type=0;                          % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en=4;                          % Total number of engines [-]
propeller   = 0;               % 0=no, 1=yes
Afterburner=1;                   % Afterburner/reheat: 0=disabled, 1 = enabled
Scramjet=0;                      % Scramjet: 0 = disabled, 1 = scramjet, 2 = ramjet
Waverider=0;                     % Waverider configuration: 0 = disabled, 1 = enabled
Cd_engine=0.1;                    % Engine/nacelle drag coefficient [-]

% ==================== IDEAL ENGINE ====================
A_inlet=1.85;                    % Inlet capture area per engine [m^2]
TIT_d_low=1200;                  % Low design turbine inlet temperature [K]
TIT_d_high=1477;                 % High design turbine inlet temperature [K]
T_0_6_high=1800;                 % High afterburner total temperature [K]
T_0_3_sl=550;                    % Sea-level compressor total temperature which the sea level thrust depends on [K]
CPR_min=15.5;                    % Minimum compressor pressure ratio [-]
CPR_max=15.5;                    % Maximum compressor pressure ratio [-]
BR_min=0;                        % Minimum bypass ratio [-]
BR_max=0;                        % Maximum bypass ratio [-]
FPR=0;                           % Fan pressure ratio [-]
n_ab_low=0;                      % Low afterburner setting [-]
n_ab_high=.6;                    % High afterburner setting [-]
eta_inlet=.92;                   % Inlet efficiency [-]
eta_c=.85;                       % Compressor efficiency [-]
eta_f=.88;                       % Fan efficiency [-]
eta_t=.88;                       % Turbine efficiency [-]
eta_b=.98;                       % Burner/combustor efficiency [-]
eta_ab=.85;                      % Afterburner efficiency [-]
eta_m=.98;                       % Mechanical efficiency [-]
eta_n=.94;                       % Nozzle efficiency [-]
eta_propeller = .85;           % Propeller efficiency

% ==================== EXISTING ENGINE DATA ====================
% GE4 / JTF-17A class SST engine
Th_wet=290000;                   % Maximum thrust with afterburner per engine at sea level [N]
Th_wet_cruise=250000;            % Maximum thrust with afterburner at cruise per engine [N]
Th_dry=220000;                   % Maximum thrust without afterburner per engine at sea level [N]
Th_dry_cruise=200000;            % Maximum thrust without afterburner at cruise per engine [N]
TIT_e_low=800;                   % Engine low turbine inlet temperature [K]
TIT_e=1477;                      % Engine high turbine inlet temperature [K]
CPR_e=12.5;                      % Engine compressor pressure ratio [-]
BR_e=0;1.3;                      % Engine bypass ratio [-]
FPR_e=0;2.5;                     % Engine fan pressure ratio [-]
eta_t_e=.88;                     % Engine turbine efficiency [-]
eta_f_e=.88;                     % Engine fan efficiency [-]
eta_ab_e=.95;                    % Engine afterburner efficiency [-]
eta_c_e=.85;                     % Engine compressor efficiency [-]
eta_m_e=.98;                     % Engine mechanical efficiency [-]
eta_b_e=.98;                     % Engine burner efficiency [-]
eta_propeller_e = .85;           % Propeller efficiency
A_inlet_e=1.85;                  % Existing engine inlet area [m^2]
T_0_6_k_e=1704+273;              % Existing engine turbine temperature [K]
W_eng_e=11237.5*4.44822;         % Engine weight per engine [N] 

% ==================== AERODYNAMICS ====================
% Subsonic_airfoil=5;              % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]
delta=0;                         % 0=no delta wing, 1=delta wing
sweep_solver=0;                  % 0=use input, 1=solve for (wing)
x_c=.5;                          % Airfoil maximum thickness location as a fraction of the chord length [-]
cl_max=1.35;                     % 2-D airfoil maximum lift coefficient [-]
CL_max=.8;                       % Aircraft maximum lift coefficient [-]
tdc=.023;                        % Maximum thickness-to-chord ratio [-]
V_stall_sl=78;                   % Sea-level fully loaded flaps up stall speed [m/s]
u_0=1.7894e-5;                   % Reference dynamic viscosity [Pa*s]
nose_angle=5;                    % Nose half angle (useful for waverider configuration)
sweep=62*pi/180;                 % Wing leading edge sweep angle [rad]
lambda=.15;                      % Wing taper ratio [-]
droop=0;                         % Drooped wing: 0 = no droop, 1=droop applied
AR           = b0^2/S0;  % Wing aspect ratio [-]

% ==================== L-2000 TAIL INPUTS ====================
% Horizontal tail (tailless SST; area set near zero)
S_H=0;                           % Horizontal-tail planform area [m^2]
b_H=0;                           % Horizontal-tail span [m]
sweep_H=0;                       % Horizontal-tail leading edge sweep angle [rad]
c_H=5;                           % Horizontal-tail mean aerodynamic chord [m]
x_t_H=.30;                       % Horizontal-tail maximum thickness location, x/c [-]
t_c_H=.08;                       % Horizontal-tail thickness-to-chord ratio [-]

% Ventral / directional fin
S_V=18;                          % Vertical-tail/fin planform area [m^2]
b_V=5.5;                         % Vertical-fin height/span [m]
sweep_V=65*pi/180;               % Vertical-fin leading-edge sweep angle [rad]
t_c_V=.04;                       % Vertical-tail maximum thickness-to-chord ratio [-]
c_V=5.5;                         % Vertical-tail mean aerodynamic chord [m]
x_t_V=.30;                       % Vertical-tail maximum thickness location, x/c [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp=1009;                         % Specific heat of air [J/(kg*K)]
rho_sl=1.226;                    % Sea-level air density [kg/m^3]
T_sl=288.15;                     % Sea-level standard temperature [K]
R=287.05;                        % Specific gas constant for air [J/(kg*K)]
y=1.4;                           % Ratio of specific heats [-]
g=9.81;                          % Gravitational acceleration [m/s^2]
p_sl=101300;                     % Sea-level standard pressure [Pa]
radius_e=6.371e6;                % Earth radius [m]
mu_0=1.716e-5;                   % Reference dynamic viscosity [Pa*s]
T_0=273.15;                      % Reference temperature [K]
Su=110.4;                        % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z=3.75;                        % Ultimate load factor / structural factor [-]
N_l=3.0;                         % Landing load factor / structural factor [-]
D_f=3.35;                        % Fuselage maximum diameter [m]
L=83.26;                         % Fuselage/aircraft length (0 = solve for) [m]
W_fuse=2.88;                     % Fuselage width [m]
skin=2;                          % 1=aluminum, 2=titanium, 3=stainless steel
S_csw=30;                        % Elevon/control-surface area [m^2]
F_w=2.88;                        % Fuselage width [m]
Ht_Hv=0;                        % 0 for conventional tail, 1 for T tail
L_t=25;                          % Tail length (wing quarter MAC to tail quarter MAC) [m]
Sr_Svt=.35;                      % Rudder/control surface to vertical-tail area ratio [-]
S_cs=S_csw+Sr_Svt*S_V;             % Total control-surface area [m^2]
S_fw   = 0;                     % Firewall surface area [m^2]

% ==================== FUSELAGE / LANDING GEAR ====================
L_m=3.50;                        % length of main landing gear, [m in this deck]
L_n=2.80;                        % nose gear length, [m in this deck]
N_mw=12;                          % number of main wheels
N_nw=2;                          % number of nose wheels
N_mss=2;                         % number of main gear shock struts

% ==================== RAYMER CONSTANTS ====================
K_dw=.768;                       % 0.768 for delta wing; = 1.0 otherwise
K_vs=1;                          % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf=.774;                      % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht=1;                         % 1.047 for rolling tail; = 1.0 otherwise
K_uht=1;                         % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door=1.06;                     % 1.0 if no cargo door; = 1.06 if one side cargo door; = 1.12 if two side cargo doors; = 1.12 if aft clamshell door; = 1.25 if two side cargo doors and aft clamshell door
K_Lg=1;                       % 1.12 if fuselage-mounted main landing gear; = 1.0 otherwise
K_cb=1;                          % 2.25 for cross-beam (F-111) gear; = 1.0 otherwise
K_tpg=1;                         % 0.826 for tripod (A-7) gear; = 1.0 otherwise
K_mp=1;                          % 1.126 for kneeling gear; = 1.0 otherwise
K_np=1;                          % 1.15 for kneeling gear; = 1.0 otherwise
K_vg=1;                          % 1.62 for variable geometry; = 1.0 otherwise
K_d=2.75;                        % inlet duct cross section: circular=1, semi circle=1.31, bottom surface U shape = 2.2, rectangular=2.75, elliptical (a=1.5, b=1) =1.68, top and bottom surface U shaped: 3.43, elliptical (a=2, b=1) = 2.6
Ls_Ld=1.3;                       % Conical centerbody inlet spike length divided by inlet duct length (supersonic fighter type aircrafts, 0 if not applicable)

% ==================== ENGINE / NACELLE LENGTHS ====================
D_e=2.27;                        % engine diameter, [m in this deck]
L_d=8;                           % duct length, [m in this deck]
L_tp=2;                          % length of tailpipe, [m in this deck]
L_sh=1.5;                        % length of engine shroud, [m in this deck]
L_a=55;                         % electrical routing distance, generators to avionics to cockpit, [m in this deck]

% ==================== FUEL / VOLUME ====================
Vi_Vt=.90;                       % integral tanks volume / total fuel volume, V_i/V_t
Vp_Vt=.10;                       % self-sealing "protected" tanks volume / total fuel volume, V_p/V_t
N_t=17;                          % number of fuel tanks

% ==================== SYSTEMS ====================
N_c=3;                           % number of crew
N_ci=2;                          % 1.0 if single pilot; = 1.2 if pilot plus backseater; = 2.0 pilot and copassenger
N_s=2;                           % number of flight control systems
N_u=6;                           % number of hydraulic utility functions (typically 5-15)
Nf=5;                            % number of functions performed by controls (typically 4-7)
Nm=2;                            % number of mechanical functions (typically 0-2)
W_uav=800*9.81;                  % uninstalled avionics weight, [N in this deck]
K_vsh=1;                         % 1.425 if variable sweep wing; = 1.0 otherwise
K_mc=1;                          % 1.45 if mission completion required after failure; = 1.0 otherwise
R_kva=80;                        % system electrical rating, kv·A (typically 40-60, or 160)

% ==================== NACELLE / INTAKE ====================
K_ng=1.017;                      % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p=1;                           % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr=1.18;                       % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt=8;                          % nacelle length, [m in this deck]
N_w=1.40;                        % nacelle width, [m in this deck]
S_n=pi*N_w*N_Lt;                 % nacelle wetted area, [m^2 in this deck]
W_en=W_eng_e;                    % engine weight, each, [N in this deck]
N_Lt_ft=N_Lt/.3048;              % nacelle length,
N_w_ft=N_w/.3048;                % nacelle width,
S_n_ft=S_n/.3048^2;              % nacelle wetted area,
W_en_lb=W_eng_e/4.44822;         % engine weight, each,

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb=5000/3.28/60;      % Rate of climb [m/s]
rate_of_descent=3000/3.28/60;    % Rate of descent [m/s]
Takeoff_field_length=3300;       % Takeoff field length [m]
Landing_field_length=2200;       % Landing field length [m]
g_turn=1.2;                      % Turning/acceleration load factor [-]
mu_2=.03;                        % Ground friction coefficient [-]
alt_0=1000;                      % Initial altitude used in carpet plot [m]
acc=2;                           % Acceleration [m/s^2]
V_lof=1.1*V_stall_sl;            % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS=0;                           % Thermal protection system: 0=no, 1=yes
% R_wle=.344/12;                   % Wing leading-edge radius [ft]
% g_s=g*3.2797;                    % Gravity conversion parameter [ft/s^2]
% J=778;                           % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity=.8;                  % Radiative emissivity [-]
% Cp_wall=1000;                    % Wall/TPS specific heat [J/(kg*K)]
% k_wall=21;                   % Aircraft wall thermal conductivity [W/(m*K)]
% k_ins=.3;                 % Insulation thermal conductivity [W/(m*K)]
% t_ins=.051;                      % Insulation thickness [m]
% t_wall=.001;                      % Wall skin thickness [m]
% Cp_ins=500;                      % Insulation specific heat [J/(kg*K)]
% rho_ins=112;                     % Insulation density [kg/m^3]
% 270;                             % Not assigned
% S_cooling=39;                    % Active cooling surface area [m^2]
% 20;                              % Not assigned
% 39;                              % Not assigned
% rho_cooling=300;                 % Active cooling surface areal density [kg/m^2]
% x=4;                             % Heating-analysis longitudinal location [m]
% Pr=.72;                          % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp=44E6;                       % Fuel specific energy [J/kg]
ratio_fo=.294;                   % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f=2010;                       % Fuel specific heat [J/(kg*K)]
rho_f=800;                       % Fuel density [kg/m^3]

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt=18000;                   % Minimum search altitude [m]
max_alt=22000;                   % Maximum search altitude [m]
WdS_max=6000;                    % Maximum wing loading shown in carpet plot [N/m^2]
alt_points=200;                  % Number of altitude points shown in design point plot [-]
i_max=20;                        % number of iterations for finding the ideal engine compressor pressure ratio during cruise [-]
ii_max=25;                       % number of iterations for finding the existing Engine high turbine inlet temperature during cruise [-]
j_max=20;                        % number of iterations for finding the ideal engine bypass ratio during cruise [-]
k_max=20;                        % number of iterations for finding the ideal Engine high turbine inlet temperature during cruise [-]
q_max=50;                         % number of iterations for the takeoff gross weight using the Breuget fuel weight for each iteration
dd_max=15;                       % number of iterations from altitude=0 to altitude=design for the descent phase
cc_max=15;                       % number of iterations from altitude=0 to altitude=design for the climb phase
zzz_max=30;                      % number of points used for the carpet plot that has smoothed out curves
xx_max=20;                       % number of iterations for finding the shockwave angle generated at the nose of a supersonic aircraft
iii_max=30;                      % number of iterations for the stagnation temperature during aerodynamic heating of a high speed supersonic aircraft
ff_max=30;                       % number of iterations for finding the existing engine afterburner fuel fraction flowrate
kk_max=30;                       % number of iterations for finding the ideal engine afterburner fuel fraction flowrate
kk_Dmax_max=30;                  % number of iterations for finding the ideal engine afterburner fuel fraction flowrate at the max drag condition
f_6_k_e_i_low=0;                 % Lower bound for existing engine afterburner fuel fraction flow rate
f_6_k_e_i_high=.5;               % Upper bound for existing engine afterburner fuel fraction flow rate
   
elseif aircraft_case==11
    
% ============================================================
% BOEING 707-120 VERIFICATION AIRCRAFT
% ORIGINAL PRODUCTION SERIES
% ============================================================

% ==================== MISSION REQUIREMENTS ====================
M = 0.80;                         % Cruise Mach number [-]
Range_cruise = 5300000;           % Cruise range [m]
TOGW_i(1) = 112490*9.81;          % Initial takeoff gross weight [N]
W_e(1) = 57600*9.81;              % Initial empty weight [N], approximate
W_payload = 5000*9.81;           % Payload weight [N], assumed
W_f(1) = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N]
Np = 179;                         % Maximum passenger capacity [-]
type2 = 0;                        % Aircraft family: 0=transport, 1=GA, 2=fighter/delta family

% ============================================================
% BOEING 707-120 VERIFICATION AIRCRAFT DATA
% ============================================================
S0 = 226.0;                       % Reference wing area [m^2]
b0 = 39.88;                       % Reference wingspan [m]
c_r0 = 8.50;                      % Reference root chord [m], approximate
Lambda0 = 35*pi/180;              % Reference quarter-chord sweep [rad], approximate
Alt0 = 10668;                     % Reference cruise altitude [m]
R0 = 5600E3;                      % Reference range [m]
L_D = 13.31;                         % Reference cruise lift-to-drag ratio [-], estimate
TSFC0 = 0.0000257;                 % Reference cruise TSFC [kg/(N*s)], initial estimate
sweep0 = 35*pi/180;               % Wing leading edge sweep angle

% ==================== PROPULSION ====================
type = 0;                         % Propulsion type: 0 = existing engine, 1 = ideal engine
N_en = 4;                         % Total number of engines [-]
propeller = 0;                    % 0=no, 1=yes
Afterburner = 0;                  % Afterburner/reheat: 0=disabled, 1=enabled
Scramjet = 0;                     % Scramjet: 0=disabled, 1=scramjet, 2=ramjet
Waverider = 0;                    % Waverider configuration: 0=disabled, 1=enabled
Cd_engine = 0.2;                  % Engine/nacelle drag coefficient [-], initial estimate

% ==================== IDEAL ENGINE ====================
A_inlet = .79;                   % Inlet capture area per engine [m^2], approximate
TIT_d_low = 800;                 % Low design turbine inlet temperature [K], model assumption
TIT_d_high = 1250;                % High design turbine inlet temperature [K], model assumption
T_0_6_high = 1400;                % High afterburner total temperature [K], model assumption
T_0_3_sl = 500;                   % Sea-level compressor total temperature [K], model assumption
CPR_min = 12.5;                    % Minimum compressor pressure ratio [-], approximate
CPR_max = 12.5;                    % Maximum compressor pressure ratio [-], approximate
BR_min = 0;                       % Minimum bypass ratio [-], turbojet
BR_max = 0;                       % Maximum bypass ratio [-], turbojet
FPR = 0;                          % Fan pressure ratio [-], no fan
n_ab_low = 0;                     % Low afterburner setting [-]
n_ab_high = 0;                    % High afterburner setting [-]
eta_inlet = 0.98;                 % Inlet efficiency [-], initial estimate
eta_c = 0.85;                     % Compressor efficiency [-], model assumption
eta_f = 0.88;                     % Fan efficiency [-], unused for turbojet
eta_t = 0.88;                     % Turbine efficiency [-], model assumption
eta_b = 0.98;                     % Burner/combustor efficiency [-], model assumption
eta_ab = 0.85;                    % Afterburner efficiency [-], unused
eta_m = 0.98;                     % Mechanical efficiency [-], model assumption
eta_n = 0.94;                     % Nozzle efficiency [-], model assumption
eta_propeller = 0.85;             % Propeller efficiency [-], unused

% ==================== EXISTING ENGINE DATA ====================
% Pratt & Whitney JT3C-6 turbojet
Th_wet = 60050;                   % Maximum thrust per engine [N], approximate
Th_wet_cruise = 30000;            % Cruise thrust per engine [N], initial estimate
Th_dry = 60050;                   % Maximum thrust without afterburner per engine at sea level [N], approximate
Th_dry_cruise = 30000;            % Cruise thrust per engine [N], initial estimate
TIT_e_low = 400;                  % Engine low turbine inlet temperature [K], model assumption
TIT_e = 1100;                     % Engine high turbine inlet temperature [K], model assumption
CPR_e = 12.5;                      % Engine compressor pressure ratio [-], approximate
BR_e = 0;                         % Engine bypass ratio [-], turbojet
FPR_e = 0;                        % Engine fan pressure ratio [-], no fan
eta_t_e = 0.88;                   % Engine turbine efficiency [-], model assumption
eta_f_e = 0.88;                   % Engine fan efficiency [-], unused
eta_ab_e = 0.85;                  % Engine afterburner efficiency [-], unused
eta_c_e = 0.85;                   % Engine compressor efficiency [-], model assumption
eta_m_e = 0.98;                   % Engine mechanical efficiency [-], model assumption
eta_b_e = 0.98;                   % Engine burner efficiency [-], model assumption
eta_propeller_e = 0.85;            % Propeller efficiency [-], unused
A_inlet_e = .79;                 % Existing engine inlet area [m^2], approximate
T_0_6_k_e = 1100;                 % Existing engine turbine temperature [K], model assumption
W_eng_e = 1800*9.81;              % Engine weight per engine [N], approximate

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 13;              % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]; verify selected airfoil
delta = 0;                         % 0=no delta wing, 1=delta wing
sweep_solver = 0;                  % 0=use input, 1=solve for (wing)
x_c = 0.30;                        % Airfoil maximum thickness location, x/c [-], assumed
% cl_max = 1.60;                     % 2-D airfoil maximum lift coefficient [-], assumed
% CL_max = 1.50;                     % Aircraft maximum lift coefficient [-], assumed
tdc = 0.10;                        % Maximum thickness-to-chord ratio [-], approximate
V_stall_sl = 95;                   % Sea-level fully loaded flaps-up stall speed [m/s], estimate
u_0 = 1.7894e-5;                   % Reference dynamic viscosity [Pa*s]
nose_angle = 0;                    % Nose half angle [deg]
sweep = 35*pi/180;                 % Wing leading-edge sweep [rad], approximate
lambda = 0.25;                     % Wing taper ratio [-], approximate
droop = 0;                         % Drooped wing: 0=no, 1=yes
AR = b0^2/S0;                      % Wing aspect ratio [-]
washout=0;                         % Wing total washout [rad];
C_p_0=-1.4;                       % Minimum pressure coefficient obtained using XFOIL

% ==================== BOEING 707-120 TAIL INPUTS ====================
% Horizontal tail
S_H = 45;                          % Horizontal-tail planform area [m^2], approximate
b_H = 12;                          % Horizontal-tail span [m], approximate
sweep_H = 35*pi/180;               % Horizontal-tail leading edge sweep angle [rad], approximate
c_H = 4.0;                         % Horizontal-tail mean aerodynamic chord [m], approximate
x_t_H = 0.30;                      % Horizontal-tail maximum thickness location, x/c [-]
t_c_H = 0.08;                      % Horizontal-tail thickness-to-chord ratio [-]

% Vertical tail
S_V = 25;                          % Vertical-tail planform area [m^2], approximate
b_V = 6.0;                         % Vertical-fin height/span [m], approximate
sweep_V = 40*pi/180;               % Vertical-fin leading-edge sweep angle [rad], approximate
t_c_V = 0.08;                      % Vertical-tail thickness-to-chord ratio [-]
c_V = 4.5;                         % Vertical-tail mean aerodynamic chord [m], approximate
x_t_V = 0.30;                      % Vertical-tail maximum thickness location, x/c [-]

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                         % Specific heat of air [J/(kg*K)]
rho_sl = 1.226;                    % Sea-level air density [kg/m^3]
T_sl = 288.15;                     % Sea-level standard temperature [K]
R = 287.05;                        % Specific gas constant for air [J/(kg*K)]
y = 1.4;                           % Ratio of specific heats [-]
g = 9.81;                          % Gravitational acceleration [m/s^2]
p_sl = 101300;                     % Sea-level standard pressure [Pa]
radius_e = 6.371e6;                % Earth radius [m]
mu_0 = 1.716e-5;                   % Reference dynamic viscosity [Pa*s]
T_0 = 273.15;                      % Reference temperature [K]
Su = 110.4;                        % Sutherland constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;                        % Ultimate load factor / structural factor [-]
N_l = 3.0;                         % Landing load factor / structural factor [-]
D_f = 3.76;                        % Fuselage maximum diameter [m]
L = 44.22;                         % Fuselage/aircraft length [m]
W_fuse = 3.76;                     % Fuselage width [m]
skin = 1;                          % 1=aluminum, 2=titanium, 3=stainless steel
S_csw = 25;                        % Control-surface area [m^2], approximate
F_w = 3.76;                        % Fuselage width [m]
Ht_Hv = 0;                         % 0 for conventional tail, 1 for T tail
L_t = 14;                          % Tail length (wing quarter MAC to tail quarter MAC) [m], approximate
Sr_Svt = 0.30;                     % Rudder/control surface to vertical-tail area ratio [-]
S_cs = S_csw + Sr_Svt*S_V;         % Total control-surface area [m^2]
S_fw = 0;                          % Firewall surface area [m^2]

% ==================== FUSELAGE / LANDING GEAR ====================
L_m = 1.80;                        % Length of main landing gear [m], approximate
L_n = 1.60;                        % Nose gear length [m], approximate
N_mw = 8;                          % Number of main wheels [-], representative
N_nw = 2;                           % Number of nose wheels [-]
N_mss = 2;                         % Number of main gear shock struts [-], assumed

% ==================== RAYMER CONSTANTS ====================
K_dw = 1.0;                        % 0.768 for delta wing; = 1.0 otherwise
K_vs = 1.0;                        % 1.19 for variable sweep wing; = 1.0 otherwise
K_dwf = 1.0;                       % 0.774 for delta wing aircraft; = 1.0 otherwise
K_rht = 1.0;                       % 1.047 for rolling tail; = 1.0 otherwise
K_uht = 1.0;                       % 1.143 for unit (all-moving) horizontal tail; = 1.0 otherwise
K_door = 1.06;                     % 1.0 no cargo door; 1.06 one side cargo door
K_Lg = 1.0;                        % 1.12 for fuselage-mounted main landing gear; = 1.0 otherwise
K_cb = 1.0;                        % 2.25 for cross-beam gear; = 1.0 otherwise
K_tpg = 1.0;                       % 0.826 for tripod gear; = 1.0 otherwise
K_mp = 1.126;                      % 1.126 for kneeling gear; = 1.0 otherwise
K_np = 1.15;                       % 1.15 for kneeling gear; = 1.0 otherwise
K_vg = 1.0;                        % 1.62 for variable geometry; = 1.0 otherwise
K_d = 1.0;                         % Inlet duct cross-section factor [-], assumed circular
Ls_Ld = 0;                         % Spike length / inlet duct length; 0 if not applicable

% ==================== ENGINE / NACELLE LENGTHS ====================
D_e = 1.20;                        % Engine diameter [m], approximate JT3C
L_d = 3.0;                         % Duct length [m], approximate
L_tp = 2.0;                        % Length of tailpipe [m], approximate
L_sh = 1.0;                        % Length of engine shroud [m], approximate
L_a = 20;                          % Electrical routing distance [m], assumed

% ==================== FUEL / VOLUME ====================
Vi_Vt = 0.90;                      % Integral tanks volume / total fuel volume [-]
Vp_Vt = 0.10;                      % Protected tanks volume / total fuel volume [-]
N_t = 12;                          % Number of fuel tanks, assumed

% ==================== SYSTEMS ====================
N_c = 3;                           % Number of crew [-]
% N_ci = 1.2;                        % Pilot plus backseater factor [-]
% N_s = 2;                           % Number of flight control systems [-]
% N_u = 6;                           % Number of hydraulic utility functions [-]
Nf = 5;                            % Number of functions performed by controls [-]
Nm = 2;                            % Number of mechanical functions [-]
W_uav = 800*9.81;                  % Uninstalled avionics weight [N]
% K_vsh = 1.0;                       % 1.425 if variable sweep wing; = 1.0 otherwise
% K_mc = 1.0;                        % 1.45 if mission completion required after failure
% R_kva = 80;                         % System electrical rating [kVA]

% ==================== NACELLE / INTAKE ====================
K_ng = 1.017;                      % 1.017 for pylon-mounted nacelle; = 1.0 otherwise
K_p = 1.0;                         % 1.133 if reciprocating engine; = 1.0 otherwise
K_tr = 1.18;                        % 1.18 for jet with thrust reverser; = 1.0 otherwise
N_Lt = 4.0;                        % Nacelle length [m], approximate
N_w = 1.20;                        % Nacelle width/diameter [m], approximate
S_n = pi*N_w*N_Lt;                 % Nacelle wetted area [m^2], approximate
W_en = W_eng_e;                    % Engine weight, each [N]
N_Lt_ft = N_Lt/0.3048;             % Nacelle length [ft]
N_w_ft = N_w/0.3048;               % Nacelle width [ft]
S_n_ft = S_n/0.3048^2;             % Nacelle wetted area [ft^2]
W_en_lb = W_eng_e/4.44822;         % Engine weight, each [lbf]

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb = 2500/3.28/60;      % Rate of climb [m/s], approximate
rate_of_descent = 2000/3.28/60;    % Rate of descent [m/s], approximate
Takeoff_field_length = 3200;       % Takeoff field length [m], approximate
Landing_field_length = 2000;       % Landing field length [m], approximate
g_turn = 1.2;                      % Turning/acceleration load factor [-]
mu_2 = 0.03;                       % Ground friction coefficient [-]
alt_0 = 1000;                      % Initial altitude used in carpet plot [m]
acc = 2;                           % Acceleration [m/s^2], assumed
V_lof = 1.1*V_stall_sl;            % Lift-off speed [m/s]

% ==================== AEROTHERMAL PARAMETERS ====================
TPS = 0;                           % Thermal protection system: 0=no, 1=yes
% R_wle = 0.344/12;                  % Wing leading-edge radius [ft], placeholder
% g_s = g*3.2797;                    % Gravity conversion parameter [ft/s^2]
% J = 778;                           % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity = 0.8;                 % Radiative emissivity [-]
% Cp_wall = 1000;                    % Wall/TPS specific heat [J/(kg*K)]
% k_wall = 21;                       % Aircraft wall thermal conductivity [W/(m*K)]
% k_ins = 0.3;                % Insulation thermal conductivity [W/(m*K)]
% t_ins = 0.051;                     % Insulation thickness [m]
% t_wall = 0.001;                    % Wall skin thickness [m]
% Cp_ins = 500;                      % Insulation specific heat [J/(kg*K)]
% rho_ins = 112;                     % Insulation density [kg/m^3]
% S_cooling = 0;                     % Active cooling surface area [m^2]
% rho_cooling = 300;                 % Active cooling surface areal density [kg/m^2]
% x = 4;                             % Heating-analysis longitudinal location [m]
% Pr = 0.72;                         % Prandtl number [-]

% ==================== FUEL PROPERTIES ====================
E_sp = 43E6;                       % Fuel specific energy [J/kg], approximate jet fuel
ratio_fo = 0.294;                  % Stoichiometric fuel-to-oxygen mass ratio [-]
Cp_f = 2010;                       % Fuel specific heat [J/(kg*K)]
rho_f = 800;                       % Fuel density [kg/m^3], approximate

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 9000;                   % Minimum search altitude [m]
max_alt = 13000;                   % Maximum search altitude [m]
WdS_max = 6000;                    % Maximum wing loading shown in carpet plot [N/m^2]
alt_points = 200;                  % Number of altitude points shown in design point plot [-]
i_max = 20;                        % Ideal engine compressor pressure ratio iterations [-]
ii_max = 25;                       % Existing Engine high turbine inlet temperature iterations [-]
j_max = 20;                        % Ideal engine bypass ratio iterations [-]
k_max = 20;                        % Ideal Engine high turbine inlet temperature iterations [-]
q_max = 50;                         % Takeoff gross weight iterations using Breguet fuel weight [-]
dd_max = 15;                       % Descent-phase iterations [-]
cc_max = 15;                       % Climb-phase iterations [-]
zzz_max = 30;                      % Number of smoothed carpet-plot points [-]
xx_max = 20;                       % Shockwave-angle iterations [-]
iii_max = 30;                      % Stagnation-temperature iterations [-]
ff_max = 30;                       % Existing engine afterburner fuel-flow iterations [-]
kk_max = 30;                       % Ideal engine afterburner fuel-flow iterations [-]
kk_Dmax_max = 30;                  % Ideal engine afterburner fuel-flow iterations at max drag [-]
f_6_k_e_i_low = 0;                 % Lower bound for existing-engine afterburner fuel fraction [-]
f_6_k_e_i_high = 0;                % Upper bound for existing-engine afterburner fuel fraction [-]

elseif aircraft_case==12
    
% ============================================================
% LOCKHEED C-5A GALAXY VERIFICATION AIRCRAFT
% ORIGINAL C-5A CONFIGURATION WITH GE TF39 TURBOFAN ENGINES
% Approximate and assumed values are identified in comments.
% ============================================================

% ==================== MISSION REQUIREMENTS ====================
M = 0.77;                         % Cruise Mach number [-], assumed
Range_cruise = 5.68e6;            % Cruise mission range [m], approximate
TOGW_i(1) = 348815*9.81;          % Initial takeoff gross weight [N], approximate
W_e(1) = 169644*9.81;             % Initial empty weight [N], approximate
W_payload = 55000*9.81;70000*9.81;118385*9.81;          % Payload weight [N], approximate
W_f(1) = TOGW_i(1)-W_e(1)-W_payload; % Initial fuel weight [N], calculated residual
Np = 81;                          % Passenger/crew capacity proxy [-], assumed
type2 = 0;                        % Aircraft family [-]: 0=transport, 1=GA, 2=fighter/delta

% ==================== REFERENCE AIRCRAFT DATA ====================
S0 = 576.0;                       % Reference wing area [m^2], approximate
b0 = 67.88;                       % Reference wingspan [m], approximate
c_r0 = 13.5;                      % Reference wing root chord [m], approximate
Lambda0 = 25*pi/180;              % Quarter-chord sweep angle [rad], approximate
Alt0 = 9144;                      % Reference cruise altitude [m], assumed
R0 = Range_cruise;                % Reference mission range [m]
L_D = 15.0;                       % Reference cruise lift-to-drag ratio [-], assumed
TSFC0 = 0.000017;                 % Reference cruise TSFC [kg/(N*s)], assumed
sweep0 = 28.3*pi/180;               % Leading-edge sweep angle [rad], approximate

% ==================== PROPULSION CONFIGURATION ====================
type = 0;                         % Propulsion model [-]: 0=existing engine, 1=ideal engine
N_en = 4;                         % Number of engines [-]
propeller = 0;                    % Propeller flag [-]: 0=no, 1=yes
Afterburner = 0;                  % Afterburner flag [-]: 0=disabled, 1=enabled
Scramjet = 0;                     % Scramjet/ramjet flag [-]: 0=disabled, 1=scramjet, 2=ramjet
Waverider = 0;                    % Waverider configuration flag [-]: 0=disabled, 1=enabled
Cd_engine = 0.2;                 % Engine/nacelle drag coefficient [-], assumed

% ==================== IDEAL ENGINE PARAMETERS ====================
% Retained for compatibility with the ideal-engine calculation branch.
A_inlet = 3;                    % Ideal-engine inlet capture area per engine [m^2], approximate
TIT_d_low = 800;                  % Low design turbine inlet temperature [K], assumed
TIT_d_high = 1250;                % High design turbine inlet temperature [K], assumed
T_0_6_high = 1400;                % High design station total temperature [K], assumed
T_0_3_sl = 500;                   % Sea-level compressor station total temperature [K], assumed
CPR_min = 25;                    % Minimum compressor pressure ratio [-], assumed
CPR_max = 25;                    % Maximum compressor pressure ratio [-], assumed
BR_min = 8;                       % Minimum bypass ratio [-], turbojet ideal-engine setup
BR_max = 8;                       % Maximum bypass ratio [-], turbojet ideal-engine setup
FPR = 1.6;                          % Ideal-engine fan pressure ratio [-], no fan in this setup
n_ab_low = 0;                     % Low afterburner setting [-], disabled
n_ab_high = 0;                    % High afterburner setting [-], disabled
eta_inlet = 0.98;                 % Ideal-engine inlet efficiency [-], assumed
eta_c = 0.85;                     % Ideal-engine compressor efficiency [-], assumed
eta_f = 0.88;                     % Ideal-engine fan efficiency [-], unused for turbojet setup
eta_t = 0.88;                     % Ideal-engine turbine efficiency [-], assumed
eta_b = 0.98;                     % Ideal-engine burner efficiency [-], assumed
eta_ab = 0.85;                    % Ideal-engine afterburner efficiency [-], unused
eta_m = 0.98;                     % Ideal-engine mechanical efficiency [-], assumed
eta_n = 0.94;                     % Ideal-engine nozzle efficiency [-], assumed
eta_propeller = 0.85;             % Ideal-engine propeller efficiency [-], unused

% ==================== EXISTING ENGINE DATA ====================
% GE TF39 turbofan; cycle parameters below are approximate model inputs.
Th_wet = 43000*4.44822;            % Maximum rated thrust per engine [N], approximate
Th_wet_cruise = 15000*4.44822;     % Cruise thrust per engine [N], assumed
Th_dry = 43000*4.44822;            % Rated thrust per engine [N], approximate
Th_dry_cruise = 15000*4.44822;     % Cruise thrust per engine [N], assumed
TIT_e_low = 600;                   % Low Engine high turbine inlet temperature [K], assumed
TIT_e = 1800;1643;                      % Engine high turbine inlet temperature [K], assumed
CPR_e = 25.0;                      % Existing-engine overall pressure ratio [-], approximate
BR_e = 8;                        % Existing-engine bypass ratio [-], approximate
FPR_e = 1.6;                       % Existing-engine fan pressure ratio [-], approximate
eta_t_e = 0.88;                    % Existing-engine turbine efficiency [-], assumed
eta_f_e = 0.88;                    % Existing-engine fan efficiency [-], assumed
eta_ab_e = 0.85;                   % Existing-engine afterburner efficiency [-], unused
eta_c_e = 0.85;                    % Existing-engine compressor efficiency [-], assumed
eta_m_e = 0.98;                    % Existing-engine mechanical efficiency [-], assumed
eta_b_e = 0.98;                    % Existing-engine burner efficiency [-], assumed
eta_propeller_e = 0.85;            % Existing-engine propeller efficiency [-], unused
A_inlet_e = 3.0;                   % Existing-engine inlet capture area [m^2], approximate
T_0_6_k_e = 1100;                  % Existing-engine station total temperature [K], assumed
W_eng_e = 3600*9.81;               % Existing-engine weight per engine [N], approximate

% ==================== AERODYNAMICS ====================
Subsonic_airfoil = 14;             % 1=SC(2)-0012, 2=SC(2)-0406, 3=SC(2)-0412, 4=SC(2)-0606, 5=SC(2)-0612, 6=SC(2)-0706, 7=SC(2)-0712, 8=NACA 0012, 9=NACA 2412, 10=NACA 4412, 11=NACA 6412, 12=BAC-XXX, 13=B707, 14=C5 [-]: verify index in your code
delta = 0;                         % 0=no delta wing, 1=delta wing
sweep_solver = 0;                  % 0=use input, 1=solve for (wing)
x_c = 0.30;                        % Maximum airfoil thickness chordwise location, x/c [-], assumed
% cl_max = 1.60;                     % Maximum 2-D airfoil lift coefficient [-], assumed
% CL_max = 1.50;                     % Maximum aircraft lift coefficient [-], assumed
tdc = 0.11;                        % Wing maximum thickness-to-chord ratio [-], approximate
V_stall_sl = 91;                   % Sea-level stall speed [m/s], assumed estimate
u_0 = 1.7894e-5;                   % Reference dynamic viscosity [Pa*s]
nose_angle = 0;                    % Nose half angle [deg]
sweep = 28.3*pi/180;                 % Wing leading-edge sweep angle [rad], approximate
lambda = 0.25;                     % Wing taper ratio [-], assumed; verify code definition
droop = 0;                         % Drooped wing [-]: 0=no, 1=yes
AR = b0^2/S0;                      % Wing aspect ratio [-], calculated from span and area
washout = 3;                       % Total wing washout angle [deg], assumed
C_p_0 = -1.2;                      % Minimum pressure coefficient [-], placeholder; verify for selected airfoil

% ==================== HORIZONTAL TAIL INPUTS ====================
S_H = 100;                         % Horizontal-tail planform area [m^2], approximate
b_H = 20.8;                        % Horizontal-tail span [m], approximate
sweep_H = 30*pi/180;               % Horizontal-tail leading edge sweep angle angle [rad], approximate
c_H = 5.0;                         % Horizontal-tail mean aerodynamic chord [m], approximate
x_t_H = 0.30;                      % Horizontal-tail maximum thickness location, x/c [-], assumed
t_c_H = 0.08;                      % Horizontal-tail thickness-to-chord ratio [-], assumed

% ==================== VERTICAL TAIL INPUTS ====================
S_V = 55;                          % Vertical-tail planform area [m^2], approximate
b_V = 12.0;                        % Vertical-tail height/span [m], approximate
sweep_V = 35*pi/180;               % Vertical-tail leading-edge sweep angle [rad], approximate
t_c_V = 0.08;                      % Vertical-tail thickness-to-chord ratio [-], assumed
c_V = 6.0;                         % Vertical-tail mean aerodynamic chord [m], approximate
x_t_V = 0.30;                      % Vertical-tail maximum thickness location, x/c [-], assumed

% ==================== ATMOSPHERE / EARTH ====================
Cp = 1009;                         % Specific heat of air at constant pressure [J/(kg*K)]
rho_sl = 1.226;                    % Sea-level air density [kg/m^3]
T_sl = 288.15;                     % Sea-level standard temperature [K]
R = 287.05;                        % Specific gas constant for air [J/(kg*K)]
y = 1.4;                           % Ratio of specific heats [-]
g = 9.81;                          % Gravitational acceleration [m/s^2]
p_sl = 101300;                     % Sea-level standard pressure [Pa]
radius_e = 6.371e6;                % Mean Earth radius [m]
mu_0 = 1.716e-5;                   % Reference dynamic viscosity [Pa*s]
T_0 = 273.15;                      % Reference temperature [K]
Su = 110.4;                        % Sutherland temperature constant [K]

% ==================== STRUCTURAL / RAYMER GEOMETRY ====================
N_z = 3.75;                        % Ultimate load factor / structural factor [-]
N_l = 3.0;                         % Landing load factor / structural factor [-]
D_f = 5.8;                         % Maximum fuselage diameter [m], approximate
L = 75.5;                          % Overall aircraft length [m], approximate
W_fuse = 5.8;                      % Fuselage width [m], approximate
skin = 1;                          % Skin material selector [-]: 1=aluminum, 2=titanium, 3=stainless steel
S_csw = 45;                        % Control-surface area [m^2], approximate
F_w = 5.8;                         % Fuselage width parameter [m], approximate
Ht_Hv = 0;                         % Tail configuration flag [-]: 0=conventional tail, 1=T-tail
L_t = 25;                          % Wing quarter-MAC to tail quarter-MAC distance [m], approximate
Sr_Svt = 0.30;                     % Rudder/control-surface to vertical-tail area ratio [-], assumed
S_cs = S_csw + Sr_Svt*S_V;         % Total control-surface area [m^2], calculated
S_fw = 0;                          % Firewall surface area [m^2], assumed none

% ==================== FUSELAGE / LANDING GEAR ====================
L_m = 2.0;                         % Main landing-gear length [m], approximate
L_n = 1.8;                         % Nose landing-gear length [m], approximate
N_mw = 24;                         % Number of main-gear wheels [-], approximate; verify model definition
N_nw = 4;                          % Number of nose-gear wheels [-], approximate
N_mss = 4;                         % Number of main-gear shock struts [-], approximate

% ==================== RAYMER CONSTANTS ====================
K_dw = 1.0;                        % Wing-design factor [-]: 0.768 for delta wing, 1.0 otherwise
K_vs = 1.0;                        % Variable-sweep factor [-]: 1.19 for variable sweep, 1.0 otherwise
K_dwf = 1.0;                       % Delta-wing/fuselage factor [-]: 0.774 for delta wing, 1.0 otherwise
K_rht = 1.0;                       % Rolling-horizontal-tail factor [-]: 1.047 for rolling tail, 1.0 otherwise
K_uht = 1.0;                       % All-moving horizontal-tail factor [-]: 1.143 for all-moving tail, 1.0 otherwise
K_door = 1.06;                     % Cargo-door structural factor [-]: 1.06 with cargo door
K_Lg = 1.12;                        % Landing-gear location factor [-]: 1.12 for fuselage-mounted main gear, 1.0 otherwise
K_cb = 1.0;                        % Cross-beam landing-gear factor [-]: 2.25 for cross-beam gear
K_tpg = 1.0;                       % Tripod landing-gear factor [-]: 0.826 for tripod gear
K_mp = 1.0;                        % Kneeling main-gear factor [-]: 1.126 for kneeling gear
K_np = 1.0;                        % Kneeling nose-gear factor [-]: 1.15 for kneeling gear
K_vg = 1.0;                        % Variable-geometry factor [-]: 1.62 for variable geometry
K_d = 1.0;                          % Inlet duct cross-section factor [-]: 1.0=circular duct, assumed
Ls_Ld = 0;                         % Spike length / inlet duct length ratio [-]: not applicable

% ==================== ENGINE / NACELLE LENGTHS ====================
D_e = 2.6;                         % Engine diameter [m], approximate
L_d = 3.0;                         % Engine duct length [m], approximate
L_tp = 2.0;                        % Engine tailpipe length [m], approximate
L_sh = 1.0;                        % Engine shroud length [m], approximate
L_a = 20;                          % Electrical routing distance [m], assumed

% ==================== FUEL / VOLUME ====================
Vi_Vt = 0.90;                      % Integral fuel-tank volume / total fuel volume [-], assumed
Vp_Vt = 0.10;                      % Protected fuel-tank volume / total fuel volume [-], assumed
N_t = 12;                          % Number of fuel tanks [-], assumed

% ==================== SYSTEMS ====================
N_c = 4;                           % Flight-deck crew count [-], assumed baseline
% N_ci = 1.2;                        % Pilot/backseater systems factor [-], assumed
% N_s = 2;                           % Number of flight-control systems [-], assumed
% N_u = 6;                           % Number of hydraulic utility functions [-], assumed
Nf = 5;                            % Number of functions performed by controls [-], assumed
Nm = 2;                            % Number of mechanical functions [-], assumed
W_uav = 800*9.81;                  % Uninstalled avionics weight [N], assumed
% K_vsh = 1.0;                       % Variable-sweep hardware factor [-]: 1.425 for variable sweep
% K_mc = 1.0;                        % Mission-completion-after-failure factor [-]: 1.45 if applicable
% R_kva = 80;                        % Electrical system rating [kVA], assumed

% ==================== NACELLE / INTAKE ====================
K_ng = 1.017;                        % Nacelle mounting factor [-]: 1.017 for pylon-mounted nacelle
K_p = 1.0;                         % Propulsion-system factor [-]: 1.133 for reciprocating engine
K_tr = 1.18;                        % Thrust-reverser factor [-]: 1.18 for jet with thrust reverser
N_Lt = 5.0;                        % Nacelle length [m], approximate
N_w = 2.4;                         % Nacelle width/diameter [m], approximate
S_n = pi*N_w*N_Lt;                 % Nacelle wetted-area estimate [m^2], calculated
W_en = W_eng_e;                    % Engine weight per engine [N], copied from engine data
N_Lt_ft = N_Lt/0.3048;              % Nacelle length [ft], converted from metres
N_w_ft = N_w/0.3048;                % Nacelle width/diameter [ft], converted from metres
S_n_ft = S_n/0.3048^2;              % Nacelle wetted area [ft^2], converted from m^2
W_en_lb = W_eng_e/4.44822;          % Engine weight per engine [lbf], converted from newtons

% ==================== PERFORMANCE / TRAJECTORY ====================
rate_of_climb = 1800/3.28/60;      % Rate of climb [m/s], approximate
rate_of_descent = 2000/3.28/60;    % Rate of descent [m/s], approximate
Takeoff_field_length = 3700;       % Takeoff field length [m], approximate
Landing_field_length = 1500;       % Landing field length [m], approximate
g_turn = 1.2;                      % Turning/acceleration load factor [-], assumed
mu_2 = 0.03;                       % Ground friction coefficient [-], assumed
alt_0 = 1000;                      % Initial altitude for carpet plot [m], assumed
acc = 2;                            % Longitudinal acceleration [m/s^2], assumed
V_lof = 1.1*V_stall_sl;            % Lift-off speed [m/s], calculated from stall speed

% ==================== AEROTHERMAL PARAMETERS ====================
TPS = 0;                           % Thermal protection system flag [-]: 0=disabled, 1=enabled
% R_wle = 0.344/12;                  % Wing leading-edge radius [ft], placeholder
% g_s = g*3.2797;                     % Gravitational acceleration [ft/s^2], converted from SI
% J = 778;                           % Mechanical equivalent of heat [ft*lbf/BTU]
% emmissivity = 0.8;                 % Surface emissivity [-], assumed
% Cp_wall = 1000;                    % Wall/TPS specific heat [J/(kg*K)], assumed
% k_wall = 21;                       % Wall thermal conductivity [W/(m*K)], assumed
% k_ins = 0.3;                % Insulation thermal conductivity [W/(m*K)], assumed
% t_ins = 0.051;                     % Insulation thickness [m], assumed
% t_wall = 0.001;                    % Wall thickness [m], assumed
% Cp_ins = 500;                      % Insulation specific heat [J/(kg*K)], assumed
% rho_ins = 112;                     % Insulation density [kg/m^3], assumed
% S_cooling = 0;                     % Active cooling surface area [m^2], disabled
% rho_cooling = 300;                 % Active cooling areal density [kg/m^2], assumed
% x = 4;                             % Longitudinal location for heating analysis [m], assumed
% Pr = 0.72;                         % Prandtl number [-], approximate for air

% ==================== FUEL PROPERTIES ====================
E_sp = 43E6;                       % Fuel specific energy [J/kg], approximate jet fuel
ratio_fo = 0.294;                  % Stoichiometric fuel/oxygen mass ratio [-], assumed
Cp_f = 2010;                       % Fuel specific heat [J/(kg*K)], assumed
rho_f = 800;                       % Fuel density [kg/m^3], approximate

% ==================== ITERATION / SEARCH PARAMETERS ====================
min_alt = 9000;                    % Minimum altitude in search [m]
max_alt = 10000;                    % Maximum altitude in search [m]
WdS_max = 6000;                    % Maximum wing loading in carpet plot [N/m^2]
alt_points = 200;                  % Number of altitude points in design plot [-]
i_max = 20;                        % Ideal-engine compressor pressure-ratio iteration limit [-]
ii_max = 25;                       % Existing-engine turbine-temperature iteration limit [-]
j_max = 20;                        % Ideal-engine bypass-ratio iteration limit [-]
k_max = 20;                        % Ideal-engine turbine-temperature iteration limit [-]
q_max = 50;                         % Takeoff gross-weight iteration limit using Breguet fuel weight [-]
dd_max = 15;                       % Descent-phase iteration limit [-]
cc_max = 15;                       % Climb-phase iteration limit [-]
zzz_max = 30;                      % Number of smoothed carpet-plot points [-]
xx_max = 20;                       % Shockwave-angle iteration limit [-]
iii_max = 30;                      % Stagnation-temperature iteration limit [-]
ff_max = 30;                       % Existing-engine afterburner fuel-flow iteration limit [-]
kk_max = 30;                       % Ideal-engine afterburner fuel-flow iteration limit [-]
kk_Dmax_max = 30;                  % Ideal-engine afterburner fuel-flow iteration limit at maximum drag [-]
f_6_k_e_i_low = 0;                 % Lower bound for existing-engine afterburner fuel fraction [-]
f_6_k_e_i_high = 0;                % Upper bound for existing-engine afterburner fuel fraction [-]

end

%Induced drag data using Raymer plot
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

elseif Subsonic_airfoil==12
    
Alpha=	[-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75];	%14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17];
Cl=[	-0.5187	-0.5473	-0.5826	-0.6085	-0.604	-0.5974	-0.5885	-0.5668	-0.3175	-0.2861	-0.2581	-0.2391	-0.2133	-0.1847	-0.1623	-0.1357	-0.1098	-0.0848	-0.0578	-0.0307	-0.005	0.0104	0.0331	0.06	0.0863	0.1125	0.1663	0.1933	0.2199	0.2458	0.2697	0.2914	0.3119	0.3327	0.3556	0.3796	0.4042	0.4286	0.4531	0.4778	0.5024	0.5275	0.5546	0.585	0.6171	0.6498	0.6858	0.7206	0.756	0.7915	0.8164	0.8375	0.8591	0.8806	0.902	0.9242	0.9465	0.9689	0.9922	1.0148	1.0363	1.0585	1.0804	1.104	1.126	1.1488	1.1708	1.1926	1.215	1.2355	1.2533	1.2719	1.2892	1.3058	1.323	1.3379	1.3501	1.3534	1.3674	1.3795	1.3901	1.399	1.407	1.4116];	%1.4107	1.4017	1.386	1.3859	1.3841	1.3794	1.3705	1.3594	1.3447	1.3256	1.3019	1.2745	1.2436];
Cd=[	0.08084	0.07319	0.06929	0.06586	0.06065	0.05622	0.05206	0.04741	0.01467	0.01374	0.01202	0.01118	0.01053	0.00994	0.00951	0.00912	0.00879	0.00847	0.00819	0.00798	0.00771	0.00593	0.00548	0.00531	0.0052	0.00507	0.00497	0.00495	0.00494	0.00498	0.00509	0.00532	0.00564	0.006	0.00626	0.00647	0.00663	0.00678	0.00692	0.00701	0.00711	0.00723	0.00738	0.00756	0.00776	0.00798	0.0082	0.00843	0.00867	0.00891	0.00913	0.00933	0.00953	0.00975	0.01001	0.01024	0.01049	0.01078	0.01102	0.01135	0.0118	0.01221	0.01266	0.01294	0.01339	0.01374	0.01417	0.01461	0.01498	0.01551	0.01628	0.01696	0.01772	0.0185	0.01912	0.01983	0.0207	0.02218	0.02288	0.0237	0.02462	0.02567	0.0268	0.02819];	%0.03005	0.03267	0.03609	0.0384	0.04107	0.04433	0.04845	0.05341	0.05955	0.06688	0.07521	0.08401	0.09332];
Cdp=[	0.07925	0.07159	0.06766	0.06416	0.05881	0.05424	0.04991	0.04498	0.00951	0.0085	0.00669	0.00581	0.00511	0.00448	0.00401	0.00359	0.00321	0.00285	0.00252	0.00227	0.00201	0.0015	0.00143	0.00135	0.00129	0.00123	0.00113	0.0011	0.00106	0.00103	0.00101	0.00103	0.00111	0.00121	0.00132	0.00141	0.0015	0.0016	0.00171	0.00182	0.00193	0.00206	0.00219	0.00233	0.00248	0.00263	0.00278	0.00295	0.00312	0.00331	0.00348	0.00365	0.00383	0.00402	0.00422	0.00445	0.00469	0.00494	0.0052	0.00548	0.00583	0.0062	0.00662	0.00694	0.00736	0.00775	0.00817	0.00863	0.00902	0.00954	0.01023	0.01095	0.01175	0.01262	0.01332	0.0141	0.01505	0.01668	0.01746	0.01838	0.01939	0.02053	0.02174	0.02323];	%0.02522	0.028	0.03162	0.03405	0.03684	0.04023	0.0445	0.04961	0.05591	0.06341	0.0719	0.08086	0.09031];
Cm=[	-0.0402	-0.0451	-0.0442	-0.0432	-0.0473	-0.0497	-0.051	-0.0516	-0.0533	-0.054	-0.0542	-0.0524	-0.052	-0.0522	-0.0509	-0.0505	-0.05	-0.0494	-0.0491	-0.0488	-0.0483	-0.0467	-0.0458	-0.0455	-0.0451	-0.0447	-0.0441	-0.0438	-0.0435	-0.043	-0.042	-0.0407	-0.0391	-0.0378	-0.0368	-0.0361	-0.0356	-0.0349	-0.0343	-0.0336	-0.0329	-0.0323	-0.0321	-0.0328	-0.0339	-0.0352	-0.0373	-0.039	-0.041	-0.043	-0.0427	-0.0414	-0.0403	-0.0391	-0.038	-0.037	-0.036	-0.0352	-0.0344	-0.0336	-0.0327	-0.0319	-0.031	-0.0304	-0.0296	-0.0289	-0.0282	-0.0274	-0.0267	-0.0257	-0.0244	-0.0231	-0.0216	-0.02	-0.0185	-0.0167	-0.0145	-0.0111	-0.0093	-0.0074	-0.0054	-0.0034	-0.0013	0.0009];	%0.0034	0.0061	0.0084	0.0093	0.0098	0.0098	0.0089	0.0071	0.004	0.0001	-0.0043	-0.0086	-0.013];
% tdc=.1537;
% tdc=.1289;

x_u=[0	0.0005	0.001	0.0025	0.005	0.015	0.02	0.025	0.0375	0.05	0.075	0.1	0.125	0.15	0.2	0.25	0.3	0.35	0.4	0.45	0.5	0.55	0.6	0.65	0.7	0.75	0.8	0.85	0.9	0.95	1];
y_u=[0	0.0037	0.005	0.0078	0.0112	0.0207	0.0242	0.0271	0.0334	0.0384	0.0465	0.0521	0.0566	0.0599	0.0643	0.0665	0.0674	0.0673	0.0667	0.0654	0.0636	0.0613	0.0585	0.0548	0.0503	0.045	0.0384	0.0307	0.0218	0.0116	0.0004];
																														
x_l=[0	0.0005	0.001	0.0025	0.005	0.015	0.02	0.025	0.0375	0.05	0.075	0.1	0.125	0.15	0.2	0.25	0.3	0.35	0.4	0.45	0.5	0.55	0.6	0.65	0.7	0.75	0.8	0.85	0.9	0.95	1];
y_l=[0	-0.0018	-0.0027	-0.0043	-0.0058	-0.0098	-0.0112	-0.0125	-0.0152	-0.0175	-0.0216	-0.0254	-0.0288	-0.032	-0.0375	-0.0417	-0.0445	-0.0458	-0.0457	-0.0443	-0.0417	-0.0383	-0.0344	-0.0303	-0.026	-0.0218	-0.0174	-0.0132	-0.009	-0.0047	-0.0004];


elseif Subsonic_airfoil==13

Alpha1=[	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-16.25	-16	-15.75	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13]; %	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16
Cl1=[	-0.599	-1.0221	-1.0481	-1.0348	-1.063	-1.0836	-1.1067	-1.1237	-1.1398	-1.1546	-1.1732	-1.1794	-1.1852	-1.1913	-1.1948	-1.1981	-1.1956	-1.1966	-1.1937	-1.1892	-1.1827	-1.1757	-1.1681	-1.1599	-1.1515	-1.1428	-1.1343	-1.1261	-1.0458	-1.0383	-1.0308	-1.0216	-1.0076	-0.9927	-0.9756	-0.9588	-0.9403	-0.922	-0.9029	-0.8834	-0.8627	-0.8419	-0.8204	-0.7982	-0.7755	-0.7522	-0.7284	-0.7045	-0.68	-0.655	-0.6298	-0.6043	-0.5789	-0.5532	-0.5275	-0.5023	-0.4753	-0.4485	-0.4213	-0.3948	-0.368	-0.3415	-0.3148	-0.2877	-0.2604	-0.2334	-0.2064	-0.1847	-0.1641	-0.136	-0.1081	-0.0797	-0.0523	-0.0237	0.005	0.0341	0.0608	0.0875	0.1146	0.1418	0.1689	0.1961	0.2234	0.2501	0.2769	0.3291	0.3561	0.3832	0.4099	0.4369	0.4637	0.4906	0.5172	0.5417	0.5674	0.5925	0.616	0.6387	0.6602	0.6892	0.7493	0.7861	0.82	0.8564	0.8777	0.8989	0.9199	0.9407	0.961	0.9814	1.0012	1.0207	1.0394	1.0576	1.0754	1.0926	1.1088	1.1243	1.1392	1.1521	1.1642	1.1741	1.1816	1.1825];	%1.182	1.1808	1.1779	1.1717	1.1627	1.1536	1.1431	1.1265	1.1068	1.0887	1.0729	1.0574
Cd1=[	0.18839	0.11815	0.10827	0.10793	0.09844	0.09087	0.08352	0.07732	0.07161	0.06632	0.0607	0.05688	0.0532	0.04953	0.04628	0.04311	0.04079	0.03806	0.03589	0.03397	0.03227	0.0307	0.02929	0.02807	0.0269	0.02586	0.0249	0.02408	0.02312	0.02243	0.02196	0.02124	0.02054	0.01985	0.01928	0.01866	0.01811	0.01753	0.01698	0.01643	0.01595	0.01544	0.01498	0.01456	0.01416	0.0138	0.01347	0.01312	0.01283	0.01258	0.01234	0.01212	0.01189	0.01168	0.01146	0.01119	0.01112	0.01103	0.011	0.01088	0.01083	0.01078	0.01066	0.01056	0.01045	0.01032	0.01023	0.00946	0.00873	0.00865	0.00857	0.00858	0.0086	0.0085	0.00844	0.00835	0.00856	0.00865	0.00873	0.00881	0.00894	0.00907	0.00921	0.00931	0.00944	0.00995	0.01009	0.01023	0.01034	0.01043	0.01058	0.01067	0.01079	0.01117	0.01134	0.01147	0.01148	0.0115	0.0115	0.01146	0.01194	0.01238	0.01282	0.0133	0.01372	0.01416	0.01462	0.01512	0.01566	0.01619	0.01677	0.01738	0.01804	0.01873	0.01944	0.02018	0.02098	0.02181	0.02265	0.0236	0.02455	0.02559	0.02667	0.02779];	%0.02909	0.03064	0.03262	0.03549	0.04047	0.04702	0.05388	0.06059	0.067	0.07301	0.07874	0.08452
Cdp1=[	0.1868	0.11567	0.10563	0.10536	0.09572	0.088	0.0805	0.07415	0.06829	0.06286	0.05704	0.05311	0.0493	0.04548	0.04209	0.03877	0.03636	0.03345	0.03115	0.02911	0.02732	0.02564	0.02416	0.02284	0.02161	0.0205	0.01948	0.01861	0.01648	0.0156	0.01483	0.01404	0.01329	0.01255	0.01193	0.01125	0.01066	0.01003	0.00944	0.00885	0.00833	0.00779	0.00729	0.00683	0.00639	0.00601	0.00564	0.00527	0.00495	0.00467	0.00441	0.00415	0.0039	0.00366	0.00344	0.00328	0.00323	0.00317	0.00312	0.00302	0.00296	0.00289	0.00279	0.00269	0.00259	0.00247	0.00238	0.00208	0.00188	0.00187	0.00185	0.00184	0.00185	0.0018	0.00173	0.00155	0.00159	0.00164	0.00169	0.00175	0.00181	0.00188	0.00196	0.00204	0.00213	0.00246	0.0026	0.00273	0.00287	0.00299	0.00313	0.00326	0.0034	0.00371	0.00397	0.00424	0.00453	0.00482	0.00512	0.00552	0.00607	0.00653	0.00701	0.00753	0.00797	0.00846	0.00896	0.00949	0.01008	0.01066	0.01129	0.01195	0.01266	0.01341	0.01418	0.01498	0.01584	0.01674	0.01764	0.01866	0.01968	0.02079	0.02195	0.02314];	%0.02451	0.02616	0.02823	0.03122	0.03637	0.04308	0.05008	0.0569	0.06341	0.06949	0.07531	0.08116
Cm1=[	0.0404	-0.0004	-0.0067	-0.0049	-0.0104	-0.0147	-0.0183	-0.0215	-0.0243	-0.0265	-0.0286	-0.0301	-0.0314	-0.0324	-0.0331	-0.0335	-0.0337	-0.0336	-0.0332	-0.0326	-0.0317	-0.0306	-0.0294	-0.0279	-0.0261	-0.0241	-0.0218	-0.0192	-0.0316	-0.0287	-0.0256	-0.0228	-0.0208	-0.0189	-0.0172	-0.0155	-0.014	-0.0125	-0.0111	-0.0097	-0.0085	-0.0073	-0.0061	-0.0051	-0.0041	-0.0033	-0.0024	-0.0016	-0.0009	-0.0002	0.0004	0.001	0.0016	0.0022	0.0028	0.0034	0.0037	0.0041	0.0044	0.0048	0.0052	0.0056	0.0061	0.0064	0.0067	0.0071	0.0075	0.0085	0.0097	0.01	0.0102	0.0104	0.0107	0.0108	0.011	0.011	0.0114	0.0118	0.0122	0.0125	0.0129	0.0132	0.0136	0.014	0.0144	0.0153	0.0157	0.016	0.0164	0.0168	0.0172	0.0176	0.018	0.0187	0.0193	0.0199	0.0209	0.0221	0.0236	0.0237	0.0167	0.0148	0.0135	0.0116	0.0129	0.0143	0.0156	0.017	0.0184	0.0198	0.0212	0.0226	0.0241	0.0257	0.0272	0.0288	0.0305	0.0323	0.034	0.036	0.038	0.0401	0.0425	0.0459];	%0.049	0.0513	0.0527	0.0525	0.0482	0.042	0.0365	0.0321	0.0282	0.0248	0.0216	0.0185
sp1=.08;

Alpha2=[	-18.75	-18.5	-18.25	-18	-17.75	-17.5	-17.25	-17	-16.75	-16.5	-15.5	-15.25	-15	-14.75	-14.5	-14.25	-14	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75];%	15	15.25	15.5	15.75	16	16.25	16.5	16.75	17
Cl2=[	-1.2134	-1.2439	-1.2616	-1.2765	-1.2886	-1.2962	-1.3015	-1.2958	-1.2878	-1.2828	-1.2177	-1.2134	-1.2304	-1.2355	-1.2355	-1.2033	-1.1988	-1.1494	-1.1389	-1.1283	-1.1159	-1.107	-1.0956	-1.087	-1.082	-1.0727	-1.0571	-1.0388	-1.0219	-1.0042	-0.9863	-0.9514	-0.93	-0.9088	-0.8907	-0.8691	-0.8458	-0.8214	-0.7973	-0.7729	-0.7485	-0.7241	-0.6998	-0.676	-0.6549	-0.6325	-0.6094	-0.585	-0.5602	-0.5351	-0.5099	-0.486	-0.4633	-0.4394	-0.4149	-0.3896	-0.364	-0.3381	-0.3119	-0.2853	-0.2587	-0.2318	-0.2048	-0.1818	-0.1561	-0.1054	-0.0786	-0.0523	-0.0369	-0.013	0.0066	0.0284	0.0517	0.0771	0.1022	0.1273	0.1512	0.1759	0.201	0.2263	0.2515	0.276	0.3024	0.3293	0.3568	0.385	0.4126	0.4437	0.4729	0.5056	0.5343	0.571	0.6068	0.6411	0.6733	0.7022	0.7367	0.7693	0.8027	0.8353	0.866	0.898	0.9316	0.9644	0.9968	1.0217	1.0353	1.0472	1.0568	1.0631	1.077	1.0896	1.1015	1.1116	1.1156	1.1189	1.1236	1.1282	1.1317	1.1322	1.1242	1.122	1.1341	1.1385	1.1447	1.1456	1.1452];	%1.1412	1.1363	1.1285	1.1199	1.1066	1.0935	1.0783	1.0594	1.0407
Cd2=[	0.09052	0.08131	0.07459	0.06863	0.06348	0.05918	0.05541	0.05316	0.05126	0.04908	0.04613	0.04418	0.03983	0.03695	0.03468	0.03257	0.03239	0.03076	0.03025	0.02981	0.02977	0.02901	0.02933	0.029	0.02976	0.02771	0.02636	0.02645	0.02584	0.02574	0.02569	0.0231	0.02248	0.02216	0.02453	0.02238	0.02146	0.0199	0.01917	0.01852	0.01779	0.01715	0.01657	0.01585	0.01482	0.01425	0.01379	0.01348	0.01323	0.01295	0.01266	0.01226	0.01177	0.01139	0.01108	0.01076	0.01048	0.01025	0.01004	0.00983	0.00965	0.00949	0.00934	0.00975	0.00987	0.00994	0.00992	0.00984	0.00872	0.00846	0.00793	0.00816	0.00836	0.00837	0.00835	0.00834	0.00831	0.00834	0.00841	0.00853	0.00867	0.00887	0.00892	0.009	0.00912	0.00926	0.00941	0.00959	0.00982	0.0102	0.01022	0.0103	0.0104	0.01057	0.01076	0.01097	0.01133	0.01195	0.01234	0.01269	0.01306	0.01348	0.0139	0.01439	0.01494	0.01553	0.0161	0.01678	0.01759	0.01859	0.01907	0.01962	0.02021	0.0209	0.02159	0.02239	0.02336	0.02457	0.02604	0.02792	0.03066	0.03308	0.03427	0.0362	0.03804	0.04041	0.04299];	%0.04609	0.04941	0.05337	0.05794	0.06432	0.07099	0.07781	0.08483	0.09183
Cdp2=[	0.08786	0.07846	0.07159	0.0655	0.06019	0.05578	0.05192	0.04964	0.04773	0.04551	0.0427	0.04071	0.03605	0.03297	0.03053	0.02816	0.02736	0.02555	0.02499	0.02449	0.02442	0.02357	0.02383	0.02341	0.02409	0.02184	0.0205	0.0207	0.02003	0.01987	0.01973	0.01687	0.01628	0.01595	0.01825	0.01594	0.01494	0.01329	0.01251	0.01181	0.01104	0.01036	0.00975	0.009	0.00796	0.00738	0.00689	0.00652	0.00623	0.00596	0.00566	0.00526	0.00474	0.00433	0.00397	0.00365	0.00335	0.00309	0.00286	0.00265	0.00245	0.00227	0.0021	0.00202	0.002	0.00192	0.0019	0.00187	0.00161	0.00156	0.00147	0.00157	0.00169	0.00177	0.00185	0.00196	0.00207	0.00221	0.00235	0.00253	0.00273	0.00301	0.00311	0.00323	0.00338	0.00354	0.0037	0.00388	0.0041	0.0045	0.00451	0.00457	0.00464	0.00473	0.00491	0.00508	0.00532	0.00589	0.0063	0.00668	0.0071	0.00756	0.00802	0.00856	0.00917	0.00981	0.01044	0.01118	0.01206	0.01314	0.01367	0.01428	0.01492	0.01567	0.01642	0.01728	0.01832	0.0196	0.02114	0.02309	0.02592	0.02843	0.02971	0.03173	0.03365	0.03612	0.03879];	%0.04198	0.04541	0.04948	0.05417	0.06069	0.06748	0.07442	0.08154	0.08864
Cm2=[	-0.0094	-0.0146	-0.0183	-0.0213	-0.0236	-0.0253	-0.0266	-0.0272	-0.0277	-0.0281	-0.0285	-0.0285	-0.0278	-0.0268	-0.0257	-0.0304	-0.0276	-0.0226	-0.021	-0.0192	-0.0173	-0.0151	-0.0127	-0.0099	-0.0059	-0.0039	-0.0028	-0.0016	-0.0001	0.0014	0.003	0.0054	0.0061	0.007	0.0091	0.0097	0.0104	0.0107	0.0112	0.0118	0.0123	0.0129	0.0135	0.0141	0.0152	0.0161	0.017	0.0177	0.0184	0.0189	0.0195	0.0203	0.0213	0.0221	0.0228	0.0234	0.0239	0.0244	0.0248	0.0252	0.0255	0.0258	0.0261	0.0268	0.0272	0.028	0.0282	0.0285	0.0305	0.0312	0.0326	0.0336	0.0344	0.0349	0.0355	0.0361	0.037	0.0377	0.0383	0.0389	0.0395	0.0402	0.0405	0.0407	0.0408	0.0407	0.0407	0.0399	0.0395	0.0383	0.0381	0.0361	0.0343	0.0327	0.0316	0.0312	0.0294	0.0278	0.0263	0.025	0.024	0.0227	0.0211	0.0195	0.018	0.0179	0.0201	0.0226	0.0253	0.0285	0.0306	0.0328	0.035	0.0374	0.0408	0.044	0.0463	0.0481	0.0495	0.0505	0.0514	0.052	0.0521	0.0522	0.0523	0.0522	0.0519];	%0.0513	0.0504	0.049	0.0469	0.0431	0.0391	0.0352	0.0316	0.028
sp2=.19;

Alpha3=[	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5];%	15.5	15.75
Cl3=[	-0.4801	-0.4816	-0.4861	-0.5003	-0.5162	-0.5245	-0.5316	-0.5124	-0.4884	-0.4668	-0.4511	-0.4323	-0.4073	-0.381	-0.3513	-0.3217	-0.2883	-0.2653	-0.252	-0.2356	-0.2182	-0.1971	-0.1743	-0.1262	-0.1021	-0.0774	-0.0516	-0.0302	-0.013	0.0082	0.0323	0.0566	0.0814	0.1041	0.1301	0.1563	0.1827	0.209	0.2348	0.261	0.2875	0.3136	0.3347	0.3596	0.3816	0.4037	0.4288	0.4534	0.4772	0.5003	0.5255	0.5621	0.6034	0.6424	0.6777	0.7106	0.7467	0.7712	0.7929	0.8147	0.8366	0.8585	0.8803	0.9019	0.9233	0.9437	0.9644	0.9848	1.0051	1.0246	1.0438	1.0624	1.0802	1.0972	1.1136	1.1288	1.1425	1.1572	1.1694	1.1833	1.1964	1.2069	1.2173];%	0.7876	0.7929
Cd3=[	0.08746	0.08351	0.07926	0.07461	0.0712	0.06783	0.06478	0.05943	0.05381	0.04983	0.04414	0.04151	0.03877	0.03628	0.03368	0.03111	0.02893	0.02703	0.02368	0.02256	0.02168	0.02067	0.01956	0.01689	0.01586	0.01509	0.01174	0.01148	0.01173	0.01125	0.01092	0.01064	0.01047	0.01071	0.01061	0.01053	0.01052	0.01054	0.01062	0.01065	0.01067	0.01069	0.01	0.00997	0.00959	0.00923	0.00921	0.00918	0.00915	0.00908	0.00903	0.00919	0.00945	0.00972	0.00998	0.01023	0.0105	0.01074	0.01098	0.01122	0.01148	0.01176	0.01205	0.01237	0.0127	0.01313	0.01354	0.01398	0.01442	0.01494	0.01548	0.01606	0.0167	0.01742	0.01818	0.01906	0.02011	0.02106	0.02234	0.02309	0.02374	0.02484	0.02595];%	0.17401	0.17759
Cdp3=[	0.08577	0.08184	0.07761	0.07298	0.06956	0.06614	0.06305	0.05756	0.05165	0.04751	0.04163	0.03891	0.03606	0.03344	0.03068	0.02791	0.02539	0.02319	0.01945	0.01801	0.01678	0.01553	0.01422	0.0109	0.00979	0.0089	0.0051	0.00446	0.00403	0.00335	0.00297	0.00261	0.0023	0.00219	0.00208	0.002	0.00201	0.00206	0.00214	0.00217	0.00217	0.00219	0.00219	0.00229	0.00232	0.00236	0.00242	0.00249	0.00256	0.00265	0.00276	0.00288	0.00315	0.00343	0.00368	0.00392	0.00419	0.00444	0.00468	0.00494	0.00521	0.00551	0.00583	0.00618	0.00654	0.007	0.00745	0.00793	0.00842	0.00898	0.00958	0.01022	0.01092	0.01171	0.01256	0.01352	0.01467	0.0157	0.01712	0.01794	0.01865	0.01987	0.02112];%	0.17222	0.17579
Cm3=[	-0.0256	-0.0277	-0.0305	-0.0346	-0.0342	-0.0338	-0.0324	-0.0372	-0.0406	-0.0426	-0.045	-0.0457	-0.0473	-0.0488	-0.0506	-0.0518	-0.0513	-0.0503	-0.0482	-0.0464	-0.0446	-0.0435	-0.0425	-0.0397	-0.0392	-0.0387	-0.0366	-0.0353	-0.0335	-0.0323	-0.0316	-0.0308	-0.0302	-0.0293	-0.0289	-0.0285	-0.0281	-0.0278	-0.0274	-0.0271	-0.0269	-0.0266	-0.0256	-0.0252	-0.0243	-0.0232	-0.0227	-0.0221	-0.0212	-0.0202	-0.0195	-0.0215	-0.0247	-0.0274	-0.0292	-0.0306	-0.0327	-0.0321	-0.031	-0.0299	-0.0288	-0.0277	-0.0266	-0.0255	-0.0244	-0.0231	-0.0219	-0.0206	-0.0194	-0.018	-0.0166	-0.0152	-0.0136	-0.012	-0.0102	-0.0084	-0.0064	-0.0045	-0.0024	-0.0004	0.0017	0.004	0.0063];%	-0.037	-0.0388
sp3=.4;

Alpha4=[	-8.75	-8.5	-8.25	-8	-7.75	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	0	0.25	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25];%	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5
Cl4=[	-0.4569	-0.4636	-0.4725	-0.4753	-0.4766	-0.4146	-0.3966	-0.3777	-0.3623	-0.3494	-0.3278	-0.3092	-0.2925	-0.2751	-0.2565	0.0038	0.0279	0.0524	0.0781	0.1039	0.1299	0.156	0.1808	0.2332	0.2496	0.2889	0.3111	0.3396	0.3754	0.4093	0.4408	0.4741	0.5062	0.5401	0.5739	0.6024	0.6248	0.6469	0.6684	0.6909	0.7137	0.7365	0.7591	0.7816	0.8037	0.8252	0.8467	0.8696	0.8891	0.9125	0.9351	0.9573	0.976	0.9955	1.0148	1.0319	1.048	1.0624	1.0759	1.0889	1.1059	1.1211	1.1315	1.1439];%	1.128	1.1204	1.0997	1.0834	1.0709	1.0488	1.0322	1.0041	0.9873
Cd4=[	0.08119	0.07639	0.07276	0.06894	0.06554	0.03789	0.034	0.03032	0.02714	0.02426	0.02229	0.02031	0.01791	0.01551	0.01341	0.01045	0.00919	0.00863	0.00828	0.00804	0.00783	0.00767	0.00738	0.00719	0.00567	0.00491	0.0048	0.00488	0.00517	0.00616	0.00685	0.00728	0.00767	0.00803	0.00843	0.00859	0.00872	0.00891	0.00917	0.00937	0.00957	0.00978	0.01003	0.0103	0.01062	0.01101	0.01139	0.01165	0.01225	0.01249	0.01279	0.01315	0.01387	0.01449	0.01513	0.01598	0.01693	0.01809	0.01947	0.02119	0.02201	0.02308	0.02536	0.02658];%	0.03428	0.03691	0.04094	0.04409	0.04693	0.05105	0.055	0.06077	0.06575
Cdp4=[	0.07962	0.07484	0.0712	0.06734	0.0639	0.03611	0.03212	0.02832	0.025	0.02194	0.01973	0.01754	0.01491	0.01229	0.00996	0.00484	0.00348	0.00285	0.00248	0.00219	0.00193	0.00172	0.00131	0.00098	0.0008	0.00084	0.00089	0.00095	0.00102	0.00129	0.00151	0.00166	0.00181	0.00197	0.00219	0.00232	0.00246	0.00262	0.00281	0.00298	0.00319	0.00342	0.00369	0.00402	0.00437	0.0048	0.00524	0.0055	0.00615	0.00641	0.00673	0.00707	0.00789	0.00858	0.0093	0.01024	0.01128	0.01256	0.01407	0.01596	0.01687	0.01805	0.02055	0.0219];%	0.03036	0.03324	0.03758	0.04094	0.04397	0.04826	0.05241	0.05831	0.06345
Cm4=[	-0.0329	-0.0379	-0.0389	-0.0398	-0.0397	-0.05	-0.0525	-0.0545	-0.0549	-0.0541	-0.0527	-0.0514	-0.0503	-0.0492	-0.0481	-0.0442	-0.0432	-0.0425	-0.042	-0.0416	-0.0412	-0.0408	-0.0402	-0.0394	-0.0379	-0.0344	-0.0329	-0.033	-0.0348	-0.0368	-0.0382	-0.0399	-0.0412	-0.043	-0.0447	-0.0451	-0.0441	-0.0431	-0.042	-0.041	-0.0401	-0.0392	-0.0382	-0.0373	-0.0363	-0.0352	-0.0341	-0.0334	-0.032	-0.0313	-0.0305	-0.0296	-0.0281	-0.0268	-0.0254	-0.0237	-0.0219	-0.0199	-0.0177	-0.0155	-0.0142	-0.0126	-0.0103	-0.0084];%	-0.0028	0.0015	0.0067	0.0103	0.0127	0.0144	0.015	0.0141	0.0123
sp4=.54;

Alpha5=[	-9.5	-9.25	-9	-8.75	-8.5	-7.25	-7	-6.75	-6.5	-6.25	-5.75	-5.5	-5.25	-5	-4.75	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5];%	10.75	11	11.25	11.5	11.75
Cl5=[	-0.3718	-0.3743	-0.3778	-0.382	-0.3881	-0.4941	-0.4756	-0.4535	-0.4276	-0.4045	-0.3342	-0.3011	-0.2745	-0.2557	-0.2371	-0.2058	-0.1827	-0.1588	-0.1344	-0.1091	-0.0833	-0.0561	-0.0291	-0.0055	0.0206	0.0475	0.0725	0.0982	0.1235	0.1488	0.172	0.1967	0.2215	0.2466	0.2717	0.2966	0.3203	0.3326	0.3403	0.3534	0.3704	0.4359	0.4849	0.5188	0.5547	0.5902	0.6171	0.6386	0.6607	0.6829	0.705	0.7272	0.7481	0.7738	0.7985	0.8192	0.8412	0.8631	0.8841	0.9044	0.9242	0.9437	0.9622	0.98	0.9951	1.0103	1.0261	1.0381	1.053	1.0671	1.0823	1.0947	1.1104	1.1204	1.1284];%	1.1283	1.1238	1.1152	1.0974	1.0813
Cd5=[	0.08509	0.08065	0.07611	0.07165	0.06704	0.05427	0.05046	0.04775	0.0445	0.04145	0.03648	0.03348	0.03062	0.02829	0.0262	0.01905	0.01798	0.017	0.01616	0.01525	0.01448	0.01453	0.0146	0.01176	0.01124	0.01111	0.01019	0.00917	0.0089	0.00869	0.0081	0.00779	0.00761	0.00751	0.00748	0.00751	0.00768	0.00892	0.00916	0.00848	0.00811	0.00821	0.00841	0.00857	0.00872	0.00889	0.00909	0.00929	0.00945	0.00962	0.00981	0.01002	0.01034	0.01026	0.01028	0.01066	0.01095	0.01126	0.01166	0.01213	0.01265	0.01319	0.01383	0.01452	0.01546	0.01642	0.01735	0.01882	0.02009	0.02164	0.02278	0.02451	0.02517	0.02686	0.02826];%	0.03038	0.0328	0.03552	0.03921	0.04269
Cdp5=[	0.08356	0.07913	0.0746	0.07015	0.06556	0.05238	0.04848	0.0457	0.04231	0.03913	0.03376	0.03052	0.02734	0.02466	0.02226	0.0144	0.01319	0.01209	0.01112	0.0101	0.00919	0.00915	0.00913	0.00611	0.0056	0.00542	0.00448	0.00334	0.00303	0.00279	0.00214	0.00175	0.00146	0.00124	0.00109	0.00098	0.00091	0.00115	0.00143	0.00157	0.0017	0.002	0.00219	0.00233	0.00248	0.00266	0.00286	0.00308	0.00324	0.00339	0.00359	0.00379	0.00412	0.00399	0.00395	0.00419	0.00451	0.00488	0.00534	0.00591	0.00649	0.00711	0.00782	0.00858	0.00959	0.01065	0.01167	0.01326	0.01465	0.01633	0.01767	0.01958	0.02039	0.02227	0.02386];%	0.02622	0.02889	0.03186	0.03584	0.03956
Cm5=[	-0.0306	-0.032	-0.0334	-0.035	-0.0371	-0.0417	-0.0449	-0.0472	-0.0501	-0.0515	-0.0557	-0.0579	-0.0585	-0.0571	-0.0555	-0.0521	-0.0515	-0.051	-0.0504	-0.0499	-0.0494	-0.0491	-0.0487	-0.048	-0.0478	-0.0475	-0.0471	-0.0458	-0.0452	-0.0446	-0.0436	-0.0429	-0.0422	-0.0416	-0.041	-0.0403	-0.0395	-0.0369	-0.0338	-0.0311	-0.0286	-0.0369	-0.0417	-0.0432	-0.0452	-0.0471	-0.0471	-0.0459	-0.0449	-0.0439	-0.0428	-0.0418	-0.0406	-0.0403	-0.0398	-0.0385	-0.0374	-0.0362	-0.035	-0.0336	-0.0321	-0.0306	-0.029	-0.0273	-0.0252	-0.0231	-0.0212	-0.0187	-0.0168	-0.0148	-0.013	-0.0109	-0.0092	-0.0068	-0.0041];%	-0.0004	0.0036	0.0075	0.0114	0.014

elseif Subsonic_airfoil==14
    
Alpha1=[	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5];%	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19	19.25
Cl1=[	-0.7849	-0.8185	-0.989	-1.0096	-1.0282	-1.0415	-1.0528	-1.0645	-1.0762	-1.089	-1.1036	-1.1019	-1.0834	-1.0633	-1.0417	-1.0183	-0.9933	-0.9722	-0.949	-0.924	-0.8981	-0.8712	-0.8436	-0.8153	-0.7859	-0.7658	-0.7444	-0.7183	-0.6893	-0.6588	-0.6273	-0.5949	-0.5748	-0.5447	-0.5147	-0.4817	-0.4481	-0.4134	-0.3786	-0.348	-0.329	-0.2759	-0.2482	-0.2201	-0.1964	-0.1672	-0.1365	-0.1054	-0.0789	-0.0505	-0.0195	0.0084	0.0375	0.0661	0.095	0.1229	0.1512	0.1791	0.2059	0.2319	0.2573	0.2823	0.3061	0.3275	0.3496	0.3719	0.3942	0.4145	0.4353	0.4578	0.4812	0.5059	0.5317	0.5557	0.5802	0.6056	0.631	0.6563	0.6807	0.7064	0.7313	0.756	0.7806	0.804	0.8263	0.8467	0.8713	0.8947	0.9179	0.9402	0.9816	0.9962	1.0231	1.052	1.0774	1.1003	1.1207	1.1436	1.1659	1.1864	1.2045	1.2247	1.2442	1.2625	1.2763	1.2943	1.3114	1.3277	1.3421	1.3505	1.3656	1.3788	1.391	1.4021	1.4105	1.4122	1.4164	1.4245	1.4306	1.4347	1.4379	1.4398];%	1.4389	1.435	1.4277	1.4147	1.3984	1.3758	1.3517	1.3269	1.3003	1.2711	1.2361
Cd1=[	0.08115	0.07244	0.04827	0.04471	0.04166	0.0394	0.03752	0.0358	0.03433	0.03317	0.03228	0.0309	0.02926	0.02775	0.02637	0.02509	0.02394	0.02288	0.022	0.02117	0.02028	0.01941	0.01865	0.01797	0.01736	0.01683	0.01563	0.0148	0.01419	0.0137	0.01327	0.01287	0.01257	0.01224	0.01164	0.0113	0.01093	0.01058	0.01019	0.00936	0.00905	0.00746	0.00668	0.00625	0.00605	0.00584	0.00571	0.00559	0.00554	0.00544	0.00535	0.00527	0.00521	0.00514	0.00511	0.00505	0.00503	0.00503	0.00507	0.00517	0.00526	0.00539	0.00557	0.00586	0.00618	0.00649	0.00687	0.00744	0.00798	0.00847	0.00884	0.00915	0.00936	0.0097	0.01004	0.01026	0.01048	0.01075	0.01102	0.01122	0.01144	0.01168	0.01195	0.01224	0.01268	0.01319	0.01338	0.01359	0.01384	0.01407	0.01466	0.0153	0.01541	0.01557	0.01571	0.01598	0.01642	0.01666	0.01693	0.01733	0.0179	0.0183	0.01874	0.01926	0.02009	0.02061	0.02119	0.02181	0.02256	0.02374	0.02444	0.02527	0.02618	0.02718	0.02839	0.03013	0.03172	0.03307	0.03462	0.03638	0.0383	0.04044];%	0.04293	0.0459	0.04946	0.05398	0.05933	0.06606	0.07347	0.08156	0.0904	0.10003	0.11101
Cdp1=[	0.079	0.07014	0.04495	0.0412	0.03796	0.03555	0.03355	0.03169	0.03009	0.02883	0.02784	0.02634	0.02455	0.02291	0.0214	0.02001	0.01876	0.01761	0.01666	0.01575	0.01478	0.01384	0.01302	0.01228	0.01162	0.01105	0.0098	0.00892	0.00829	0.00778	0.00733	0.00691	0.00659	0.00624	0.00561	0.00527	0.00489	0.00455	0.0042	0.00373	0.00352	0.00277	0.00246	0.00228	0.00217	0.00206	0.00197	0.00189	0.00185	0.00178	0.00166	0.0016	0.00152	0.00148	0.00142	0.00137	0.00133	0.00129	0.00126	0.00125	0.00127	0.0013	0.00135	0.00144	0.00154	0.00166	0.0018	0.002	0.00222	0.00243	0.00265	0.00283	0.003	0.00321	0.00343	0.00366	0.00388	0.00415	0.00448	0.00472	0.00499	0.00527	0.00556	0.0059	0.00638	0.00698	0.00721	0.00748	0.00779	0.00807	0.00873	0.00946	0.00962	0.00981	0.00993	0.01017	0.01064	0.0109	0.01117	0.01156	0.01216	0.01262	0.01308	0.01361	0.01448	0.01507	0.01571	0.01637	0.01716	0.0184	0.01918	0.02009	0.02107	0.02214	0.02342	0.02525	0.02695	0.0284	0.03006	0.03192	0.03394	0.03617];%	0.03877	0.04185	0.04553	0.05019	0.05569	0.06259	0.07018	0.07844	0.08745	0.09725	0.10841
Cm1=[	-0.0559	-0.0606	-0.0647	-0.0626	-0.0596	-0.0564	-0.0526	-0.0484	-0.0436	-0.038	-0.0316	-0.0283	-0.0283	-0.0284	-0.0286	-0.029	-0.0296	-0.0292	-0.0291	-0.0294	-0.0298	-0.0304	-0.0311	-0.0319	-0.0329	-0.0317	-0.0313	-0.0317	-0.0326	-0.0337	-0.0349	-0.0364	-0.035	-0.0357	-0.0367	-0.0381	-0.0396	-0.0415	-0.0433	-0.0445	-0.0428	-0.0436	-0.0442	-0.0445	-0.0436	-0.044	-0.0447	-0.0454	-0.0451	-0.0452	-0.0458	-0.0458	-0.046	-0.0461	-0.0463	-0.0462	-0.0463	-0.0462	-0.0458	-0.0454	-0.0448	-0.0441	-0.0432	-0.0418	-0.0407	-0.0396	-0.0386	-0.0373	-0.0361	-0.0352	-0.0345	-0.034	-0.0337	-0.0331	-0.0326	-0.0322	-0.0318	-0.0314	-0.0307	-0.0304	-0.0299	-0.0294	-0.0289	-0.0281	-0.0272	-0.0258	-0.0253	-0.0245	-0.0238	-0.0228	-0.0202	-0.0178	-0.0178	-0.0184	-0.0183	-0.0177	-0.0166	-0.016	-0.0153	-0.0143	-0.0129	-0.0119	-0.0107	-0.0095	-0.0076	-0.0063	-0.005	-0.0036	-0.002	0.0004	0.0018	0.0034	0.005	0.0066	0.0084	0.0107	0.0126	0.0139	0.0153	0.0166	0.0177	0.0186];%	0.0193	0.0197	0.0196	0.0189	0.0173	0.0147	0.0112	0.007	0.0023	-0.0029	-0.0088
sp1=.365;

Alpha2=[	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5];%	16.75	17	17.5	17.75	18	18.25	18.5	18.75	19	19.25
Cl2=[	-0.7241	-0.7674	-0.8021	-0.8355	-0.8682	-0.8985	-0.916	-0.9348	-0.9264	-0.9081	-0.8858	-0.8613	-0.8355	-0.8123	-0.7919	-0.7665	-0.7395	-0.7118	-0.6832	-0.6537	-0.6232	-0.5922	-0.5622	-0.549	-0.5204	-0.49	-0.4593	-0.4281	-0.3975	-0.367	-0.3399	-0.3176	-0.289	-0.2598	-0.2291	-0.1993	-0.1777	-0.1518	-0.1245	-0.0971	-0.0707	-0.049	-0.0253	0.0013	0.0291	0.0554	0.0818	0.111	0.1375	0.1932	0.2211	0.248	0.2747	0.3015	0.3279	0.3538	0.3786	0.4012	0.4216	0.442	0.4628	0.4806	0.4999	0.5198	0.5405	0.5616	0.5836	0.6061	0.6308	0.6578	0.6884	0.7217	0.7555	0.7903	0.8193	0.837	0.8559	0.8771	0.9002	0.9234	0.9464	0.9694	0.9912	1.0122	1.0369	1.0611	1.0842	1.1053	1.128	1.1496	1.1699	1.1886	1.2091	1.2294	1.2489	1.2658	1.2842	1.3027	1.3206	1.3376	1.3516	1.367	1.3834	1.3984	1.4103	1.4202	1.4341	1.4462	1.4567	1.4629	1.4687	1.4783	1.4861	1.4929	1.4984	1.5023	1.5039];%	1.5022	1.4966	1.4787	1.4704	1.4597	1.4447	1.4259	1.4006	1.3653	1.314
Cd2=[	0.07079	0.06025	0.05426	0.05015	0.04728	0.04419	0.03883	0.03122	0.02785	0.02569	0.02398	0.02256	0.02131	0.02026	0.01935	0.01844	0.0176	0.01681	0.01609	0.01542	0.01481	0.01424	0.01346	0.01286	0.0123	0.0119	0.01155	0.01123	0.01084	0.01044	0.01026	0.00994	0.00973	0.00941	0.00915	0.0088	0.00838	0.00814	0.00782	0.00744	0.00693	0.00641	0.00595	0.00564	0.00541	0.00527	0.00515	0.00504	0.00498	0.00487	0.00481	0.00478	0.00475	0.00476	0.00479	0.00482	0.00492	0.00511	0.00539	0.00572	0.00606	0.00661	0.00715	0.00767	0.00811	0.00852	0.00883	0.0091	0.00936	0.00976	0.01007	0.0104	0.01065	0.01092	0.01121	0.01147	0.01181	0.01211	0.01232	0.01256	0.01282	0.01311	0.01349	0.01394	0.01408	0.01426	0.01452	0.01494	0.01519	0.01549	0.01587	0.01636	0.01671	0.01707	0.0175	0.01812	0.01861	0.01908	0.01959	0.02016	0.02096	0.02163	0.02223	0.02291	0.02385	0.02491	0.02569	0.0266	0.02763	0.02901	0.03043	0.03157	0.03289	0.03431	0.03589	0.03766	0.03971];%	0.04216	0.04514	0.05254	0.05657	0.06123	0.06682	0.07337	0.08155	0.09205	0.10617
Cdp2=[	0.06879	0.05809	0.05197	0.04774	0.04475	0.0415	0.03575	0.02745	0.02371	0.02131	0.01939	0.0178	0.01641	0.01525	0.01423	0.01322	0.0123	0.01143	0.01064	0.0099	0.00924	0.00863	0.00778	0.00713	0.00654	0.00611	0.00576	0.00542	0.005	0.00461	0.00442	0.00411	0.0039	0.00359	0.00333	0.00305	0.00287	0.00271	0.00254	0.00235	0.00217	0.00203	0.00192	0.00182	0.00173	0.00172	0.00167	0.00157	0.00152	0.00143	0.00135	0.0013	0.00127	0.00125	0.00126	0.00128	0.0013	0.00134	0.00144	0.00156	0.0017	0.0019	0.00211	0.00234	0.00255	0.00277	0.00297	0.00316	0.00336	0.0036	0.00384	0.00414	0.00439	0.00467	0.00496	0.00521	0.00557	0.0059	0.00613	0.00639	0.00667	0.00696	0.00734	0.00782	0.00798	0.00817	0.00839	0.00883	0.0091	0.00941	0.00978	0.0103	0.01069	0.01107	0.01151	0.01215	0.01269	0.01322	0.01377	0.01438	0.01521	0.01596	0.01663	0.01736	0.01834	0.01949	0.02036	0.02134	0.02242	0.02387	0.02539	0.02662	0.02803	0.02954	0.03121	0.03306	0.03521];%	0.03776	0.04086	0.04854	0.05271	0.05751	0.06326	0.06997	0.07833	0.08904	0.10338
Cm2=[	-0.0604	-0.0675	-0.0691	-0.0675	-0.0635	-0.0597	-0.0591	-0.0575	-0.0565	-0.0565	-0.0568	-0.0573	-0.0579	-0.0578	-0.0569	-0.057	-0.0575	-0.058	-0.0588	-0.0596	-0.0607	-0.0618	-0.0628	-0.06	-0.0605	-0.0614	-0.0623	-0.0632	-0.064	-0.0648	-0.0648	-0.0638	-0.064	-0.0644	-0.0651	-0.0657	-0.0646	-0.0643	-0.0643	-0.0645	-0.0645	-0.0635	-0.0628	-0.0626	-0.0626	-0.0622	-0.0618	-0.0619	-0.0615	-0.0612	-0.061	-0.0607	-0.0603	-0.0599	-0.0595	-0.0589	-0.0581	-0.0568	-0.0551	-0.0535	-0.052	-0.05	-0.0484	-0.047	-0.0456	-0.0443	-0.0431	-0.042	-0.0414	-0.0414	-0.0422	-0.0436	-0.0451	-0.0469	-0.0474	-0.0454	-0.0437	-0.0425	-0.0416	-0.0408	-0.04	-0.0392	-0.0382	-0.0371	-0.0366	-0.0361	-0.0354	-0.0343	-0.0335	-0.0325	-0.0313	-0.0298	-0.0287	-0.0276	-0.0263	-0.0247	-0.0233	-0.022	-0.0206	-0.0191	-0.0172	-0.0156	-0.0142	-0.0126	-0.0107	-0.0085	-0.007	-0.0053	-0.0035	-0.0014	0.0006	0.0022	0.0038	0.0053	0.0067	0.0081	0.0093];%	0.0105	0.0114	0.0117	0.0111	0.0099	0.0079	0.0051	0.001	-0.0047	-0.0124
sp2=.431;

Alpha3=[	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16	16.25	16.5];%	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19	19.25
Cl3=[	-0.7646	-0.8038	-0.8409	-0.8785	-0.9114	-0.9178	-0.9088	-0.8921	-0.8712	-0.8479	-0.8226	-0.8036	-0.7807	-0.7548	-0.7277	-0.6995	-0.6701	-0.6399	-0.6101	-0.5833	-0.5635	-0.5388	-0.5108	-0.4821	-0.4534	-0.4241	-0.3964	-0.3717	-0.3473	-0.3212	-0.2933	-0.2646	-0.2366	-0.2106	-0.1855	-0.1595	-0.1338	-0.1063	-0.0786	-0.0536	-0.0289	-0.0042	0.0205	0.0445	0.0696	0.0961	0.1241	0.1505	0.1787	0.2051	0.2324	0.2597	0.287	0.314	0.3407	0.3663	0.3898	0.4111	0.4314	0.4518	0.4706	0.4883	0.5074	0.5294	0.5548	0.583	0.6157	0.65	0.6842	0.7194	0.754	0.7744	0.7932	0.8124	0.8318	0.8517	0.8741	0.8967	0.9193	0.9419	0.9643	0.9853	1.0048	1.0293	1.0531	1.0766	1.0991	1.1189	1.1421	1.1642	1.1852	1.2025	1.2233	1.2439	1.2639	1.2821	1.2978	1.3164	1.3344	1.3522	1.3691	1.3837	1.397	1.4132	1.4288	1.4431	1.4558	1.4621	1.4765	1.4888	1.4996	1.5066	1.5096	1.5184	1.5249	1.5295	1.5317];%	1.5292	1.5214	1.5203	1.5169	1.5106	1.5028	1.4918	1.4772	1.4584	1.4325	1.397
Cd3=[	0.05962	0.05319	0.04892	0.04414	0.03713	0.03223	0.02918	0.02683	0.02495	0.02333	0.02196	0.02084	0.0198	0.01881	0.01791	0.01708	0.01633	0.01563	0.01496	0.01385	0.01325	0.01278	0.01241	0.01207	0.01171	0.01142	0.01089	0.0107	0.01049	0.01014	0.00991	0.00971	0.00942	0.00923	0.00906	0.0088	0.00837	0.00809	0.00784	0.00759	0.00722	0.0067	0.00609	0.00567	0.00538	0.00518	0.00504	0.00496	0.00484	0.00477	0.00471	0.00468	0.00467	0.00464	0.00463	0.00466	0.00476	0.00497	0.00526	0.00557	0.00597	0.00647	0.00702	0.00759	0.00815	0.00863	0.00908	0.00937	0.00965	0.00996	0.01031	0.01053	0.01072	0.01095	0.01124	0.01156	0.01177	0.01201	0.01228	0.01257	0.01289	0.01334	0.0139	0.01406	0.01427	0.0145	0.0148	0.01531	0.0155	0.01573	0.01604	0.01665	0.01697	0.0173	0.01768	0.01819	0.01889	0.01936	0.01986	0.02037	0.02093	0.02168	0.0225	0.0231	0.02375	0.02448	0.02534	0.02668	0.02741	0.02831	0.02933	0.03068	0.03236	0.03362	0.0351	0.03676	0.03871];%	0.04119	0.04431	0.04694	0.04997	0.05352	0.05752	0.06222	0.06774	0.0743	0.08255	0.09303
Cdp3=[	0.05756	0.05099	0.0466	0.0416	0.03409	0.02874	0.02538	0.02276	0.02065	0.01884	0.0173	0.01604	0.01487	0.01377	0.01276	0.01184	0.01101	0.01024	0.0095	0.00829	0.00764	0.00715	0.00677	0.0064	0.00601	0.00569	0.00514	0.00495	0.00472	0.00437	0.00413	0.00392	0.00362	0.00344	0.00326	0.00305	0.00283	0.00265	0.00249	0.00238	0.00225	0.00208	0.00187	0.00176	0.0017	0.00166	0.0016	0.00156	0.00143	0.00138	0.00133	0.00128	0.00126	0.00127	0.00127	0.00126	0.00127	0.00134	0.00145	0.00158	0.00175	0.00195	0.00218	0.00241	0.00266	0.0029	0.00313	0.00334	0.00357	0.00378	0.00402	0.00422	0.00441	0.00463	0.0049	0.00525	0.00548	0.00574	0.00603	0.00634	0.00667	0.00713	0.00774	0.00792	0.00815	0.00838	0.00867	0.00921	0.00941	0.00966	0.00996	0.01058	0.01094	0.01131	0.0117	0.01222	0.01296	0.01349	0.01404	0.01461	0.0152	0.01597	0.01686	0.01753	0.01825	0.01904	0.01994	0.02134	0.02216	0.02314	0.02423	0.02563	0.02741	0.02878	0.03034	0.03209	0.03412];%	0.0367	0.03995	0.0427	0.04586	0.04954	0.05368	0.05853	0.06419	0.07091	0.07935	0.09003
Cm3=[	-0.0678	-0.0697	-0.0681	-0.0667	-0.0665	-0.0655	-0.065	-0.0651	-0.0654	-0.0659	-0.0666	-0.0657	-0.0654	-0.0657	-0.0663	-0.067	-0.0679	-0.0689	-0.0698	-0.0703	-0.0689	-0.0685	-0.0689	-0.0693	-0.0697	-0.0702	-0.0704	-0.0699	-0.0692	-0.069	-0.0691	-0.0693	-0.0694	-0.0691	-0.0685	-0.0682	-0.0679	-0.0679	-0.068	-0.0675	-0.067	-0.0666	-0.0661	-0.0654	-0.0648	-0.0645	-0.0644	-0.0639	-0.0637	-0.0632	-0.063	-0.0627	-0.0624	-0.0621	-0.0617	-0.0611	-0.0599	-0.0583	-0.0566	-0.0549	-0.0529	-0.0508	-0.049	-0.048	-0.0478	-0.0481	-0.0496	-0.0512	-0.0528	-0.0547	-0.0565	-0.055	-0.0532	-0.0515	-0.0499	-0.0483	-0.0473	-0.0463	-0.0453	-0.0444	-0.0435	-0.0423	-0.0409	-0.0403	-0.0397	-0.0391	-0.0383	-0.0369	-0.0362	-0.0353	-0.0342	-0.0325	-0.0314	-0.0303	-0.0292	-0.0278	-0.026	-0.0246	-0.0233	-0.0219	-0.0205	-0.0188	-0.0169	-0.0155	-0.014	-0.0125	-0.0108	-0.0084	-0.007	-0.0055	-0.0039	-0.002	0.0001	0.0015	0.003	0.0045	0.0059];%	0.0073	0.0085	0.0091	0.0094	0.0093	0.0087	0.0075	0.0056	0.0028	-0.0013	-0.0069
sp3=.568;

Alpha4=[	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.75	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16];%	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19
Cl4=[	-0.7664	-0.8909	-0.92	-0.9392	-0.926	-0.9101	-0.8911	-0.886	-0.8718	-0.8515	-0.8271	-0.8015	-0.7751	-0.7509	-0.728	-0.7015	-0.6734	-0.6441	-0.6137	-0.5826	-0.5511	-0.5199	-0.4978	-0.4711	-0.4412	-0.4103	-0.3792	-0.3492	-0.3258	-0.2991	-0.2711	-0.242	-0.2133	-0.1886	-0.162	-0.1345	-0.1075	-0.0807	-0.0546	-0.0276	-0.0007	0.0237	0.0454	0.0712	0.098	0.1246	0.1524	0.179	0.2056	0.2327	0.2594	0.2859	0.3124	0.3386	0.3639	0.3881	0.4112	0.4336	0.4556	0.4778	0.4961	0.5152	0.5348	0.5557	0.5814	0.6111	0.6425	0.6751	0.7089	0.7434	0.7682	0.7881	0.8097	0.832	0.8548	0.8776	0.9	0.9203	0.944	0.968	0.992	1.0157	1.0388	1.0611	1.0778	1.1032	1.1277	1.1514	1.1748	1.1959	1.216	1.2383	1.2601	1.2787	1.2994	1.3189	1.3351	1.3532	1.3702	1.3852	1.3976	1.4128	1.427	1.4403	1.4512	1.4563	1.466	1.4763	1.4853	1.4929	1.4994	1.5049	1.5089	1.5107];%	1.5098	1.5049	1.4956	1.4816	1.4693	1.4632	1.454	1.4407	1.4239	1.402	1.3745	1.3396
Cd4=[	0.03495	0.04296	0.04161	0.0403	0.03775	0.03571	0.03386	0.02965	0.0267	0.02453	0.02301	0.02154	0.02	0.01876	0.01771	0.01675	0.01587	0.01509	0.01437	0.01373	0.01316	0.01265	0.01205	0.01152	0.0111	0.01074	0.01042	0.01003	0.00979	0.00959	0.00933	0.00913	0.00889	0.0087	0.00848	0.00822	0.00788	0.00767	0.00748	0.00722	0.00687	0.00637	0.00559	0.00526	0.00507	0.00496	0.00487	0.0048	0.00473	0.0047	0.00469	0.0047	0.00472	0.00476	0.00486	0.00499	0.00518	0.00543	0.00572	0.00603	0.00658	0.0071	0.00759	0.00807	0.00846	0.00885	0.00923	0.00958	0.00994	0.01023	0.0105	0.01071	0.01092	0.01116	0.01142	0.01172	0.01208	0.01263	0.0129	0.01315	0.0134	0.01367	0.01398	0.01436	0.01521	0.0153	0.01546	0.01567	0.0159	0.01629	0.01668	0.01689	0.01713	0.01761	0.01792	0.01832	0.01897	0.01947	0.02004	0.02076	0.02165	0.02233	0.02307	0.02388	0.02487	0.02629	0.02738	0.02844	0.02962	0.03092	0.03234	0.03389	0.03561	0.03758];%	0.03989	0.0427	0.04615	0.05043	0.05485	0.05877	0.06339	0.06884	0.07519	0.08278	0.09176	0.10247
Cdp4=[	0.03252	0.0402	0.03875	0.03732	0.03458	0.03236	0.03035	0.02581	0.02256	0.02015	0.01845	0.0168	0.01509	0.01371	0.01255	0.01149	0.01052	0.00965	0.00887	0.00817	0.00754	0.00699	0.00633	0.00575	0.00532	0.00495	0.00461	0.00423	0.00399	0.00382	0.00357	0.00338	0.00314	0.00297	0.00277	0.00257	0.00239	0.00228	0.00215	0.00201	0.00186	0.00173	0.00158	0.00151	0.00147	0.00144	0.00137	0.00132	0.00129	0.00127	0.00124	0.00122	0.00122	0.00123	0.00124	0.00129	0.00137	0.00146	0.00157	0.0017	0.0019	0.00212	0.00234	0.00257	0.00278	0.00299	0.00322	0.00346	0.0037	0.00394	0.00419	0.00441	0.00463	0.00488	0.00516	0.00546	0.00582	0.00642	0.00671	0.00698	0.00725	0.00753	0.00785	0.00823	0.00914	0.00924	0.00942	0.00966	0.00989	0.01026	0.01069	0.01092	0.01116	0.01162	0.01197	0.01238	0.01303	0.01359	0.0142	0.01494	0.01589	0.01665	0.01747	0.01833	0.01937	0.02087	0.02206	0.02321	0.02449	0.02589	0.0274	0.02904	0.03085	0.03292];%	0.03533	0.03826	0.04185	0.04628	0.05085	0.05491	0.05966	0.06527	0.07177	0.07952	0.08868	0.09957
Cm4=[	-0.0729	-0.0681	-0.0615	-0.0558	-0.0568	-0.0574	-0.0582	-0.0577	-0.0576	-0.0579	-0.0584	-0.0591	-0.0599	-0.0599	-0.0595	-0.0598	-0.0604	-0.0612	-0.0622	-0.0634	-0.0645	-0.0656	-0.0646	-0.0646	-0.0653	-0.0661	-0.0669	-0.0675	-0.0666	-0.0664	-0.0665	-0.0668	-0.067	-0.0664	-0.0661	-0.0661	-0.066	-0.0659	-0.0655	-0.0654	-0.0653	-0.0649	-0.064	-0.0636	-0.0633	-0.0629	-0.0627	-0.0622	-0.0618	-0.0614	-0.061	-0.0605	-0.06	-0.0595	-0.0589	-0.0579	-0.0568	-0.0555	-0.0542	-0.053	-0.0511	-0.0494	-0.0477	-0.0463	-0.046	-0.0467	-0.0477	-0.049	-0.0505	-0.0522	-0.0517	-0.0501	-0.0489	-0.0478	-0.0469	-0.046	-0.045	-0.0436	-0.0429	-0.0423	-0.0417	-0.041	-0.0403	-0.0394	-0.0376	-0.0373	-0.0368	-0.0362	-0.0355	-0.0345	-0.0333	-0.0325	-0.0316	-0.0301	-0.0291	-0.0279	-0.0262	-0.0248	-0.0233	-0.0215	-0.0194	-0.0178	-0.0161	-0.0143	-0.0124	-0.0098	-0.0079	-0.0061	-0.0044	-0.0026	-0.0009	0.0007	0.0023	0.0038];%	0.0051	0.0063	0.0071	0.0073	0.0068	0.006	0.0046	0.0026	-0.0002	-0.0039	-0.0086	-0.0143
sp4=.940;

Alpha5=[	-15.25	-15	-14.75	-14.5	-14.25	-14	-13.75	-13.5	-13.25	-13	-12.75	-12.5	-12.25	-12	-11.75	-11.5	-11.25	-11	-10.75	-10.5	-10.25	-10	-9.75	-9.5	-9.25	-9	-8.75	-8.5	-8.25	-8	-7.75	-7.5	-7.25	-7	-6.75	-6.5	-6.25	-6	-5.75	-5.5	-5.25	-5	-4.5	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5	10.75	11	11.25	11.5	11.75	12	12.25	12.5	12.75	13	13.25	13.5	13.75	14	14.25	14.5	14.75	15	15.25	15.5	15.75	16];%	16.25	16.5	16.75	17	17.25	17.5	17.75	18	18.25	18.5	18.75	19	19.25
Cl5=[	-1.2436	-1.2535	-1.2572	-1.2578	-1.2542	-1.2442	-1.231	-1.2151	-1.2062	-1.1887	-1.1682	-1.1465	-1.1246	-1.1027	-1.0807	-1.0583	-1.0353	-1.0169	-0.9955	-0.9726	-0.9497	-0.927	-0.9046	-0.8823	-0.86	-0.8378	-0.8155	-0.7964	-0.7735	-0.7382	-0.7032	-0.6681	-0.6341	-0.6004	-0.5653	-0.5305	-0.5001	-0.4697	-0.4395	-0.4113	-0.3843	-0.3578	-0.305	-0.2784	-0.2509	-0.2233	-0.1954	-0.1678	-0.1399	-0.1118	-0.0838	-0.0559	-0.0279	0.0006	0.0287	0.0573	0.0856	0.1141	0.1425	0.1708	0.1988	0.2247	0.2527	0.2806	0.3086	0.3365	0.3639	0.3913	0.4186	0.4448	0.4692	0.4917	0.5168	0.5429	0.5696	0.5959	0.6221	0.6486	0.6749	0.7009	0.7268	0.7521	0.777	0.8009	0.8249	0.855	0.8814	0.9085	0.9352	0.9615	0.9874	1.0131	1.038	1.0601	1.0852	1.1111	1.1365	1.1618	1.1869	1.2094	1.2332	1.258	1.2822	1.306	1.3273	1.348	1.3697	1.3907	1.4112	1.4308	1.4479	1.459	1.4735	1.4878	1.5009	1.5133	1.5254	1.5375	1.549	1.5591	1.568	1.5757	1.5813	1.5848	1.5856];%	1.5833	1.5804	1.5828	1.5832	1.5818	1.5791	1.575	1.5697	1.5617	1.5527	1.5406	1.5253	1.5084
Cd5=[	0.04112	0.03764	0.03508	0.033	0.03133	0.0298	0.02849	0.02739	0.02541	0.0244	0.02373	0.02318	0.02264	0.02206	0.02148	0.02094	0.02049	0.01932	0.01871	0.01837	0.01805	0.0177	0.01732	0.01691	0.01653	0.01619	0.0159	0.01493	0.01441	0.01402	0.01367	0.01331	0.01293	0.01256	0.01219	0.01187	0.01139	0.01079	0.01048	0.01025	0.01009	0.00998	0.0096	0.00929	0.00913	0.009	0.00885	0.00863	0.00842	0.0083	0.00818	0.00802	0.00794	0.00778	0.00765	0.00752	0.00745	0.00735	0.00727	0.00716	0.00686	0.00574	0.00568	0.00569	0.0057	0.00573	0.00584	0.00598	0.00614	0.00645	0.007	0.00782	0.0083	0.00858	0.00877	0.00902	0.00927	0.00948	0.00971	0.00992	0.01016	0.01032	0.0105	0.0107	0.01091	0.01125	0.01163	0.01188	0.01216	0.01247	0.01279	0.01313	0.01353	0.01423	0.01458	0.01481	0.01509	0.01537	0.01564	0.01617	0.01655	0.0168	0.0171	0.01742	0.01796	0.01852	0.01896	0.01943	0.01991	0.02044	0.02105	0.02196	0.02266	0.02337	0.02419	0.02508	0.02602	0.02698	0.02804	0.02924	0.0306	0.03211	0.03387	0.0359	0.03824];%	0.04101	0.04391	0.04635	0.04909	0.05209	0.05536	0.05896	0.06284	0.06725	0.07199	0.07737	0.08342	0.09
Cdp5=[	0.03813	0.03447	0.03174	0.02952	0.02769	0.02603	0.02458	0.02335	0.02119	0.02012	0.01941	0.01882	0.01823	0.01759	0.01693	0.01631	0.01579	0.01449	0.01385	0.01349	0.01316	0.01277	0.01234	0.01188	0.01145	0.01105	0.01071	0.00965	0.00911	0.00871	0.00835	0.00797	0.00755	0.00715	0.00676	0.00641	0.00589	0.00528	0.00496	0.00471	0.0045	0.00431	0.00376	0.00342	0.00323	0.00305	0.00288	0.0026	0.00239	0.00224	0.00209	0.00193	0.00181	0.0017	0.00161	0.00156	0.00151	0.00145	0.0014	0.00136	0.00128	0.00116	0.00117	0.00118	0.00121	0.00125	0.0013	0.00138	0.00146	0.00158	0.00179	0.00212	0.00235	0.00254	0.00271	0.00287	0.00306	0.00326	0.00343	0.00362	0.00385	0.00406	0.00428	0.0045	0.00476	0.0051	0.00549	0.00576	0.00605	0.00637	0.0067	0.00704	0.00744	0.00816	0.00854	0.0088	0.0091	0.00939	0.00965	0.01017	0.01058	0.01087	0.01118	0.0115	0.01203	0.01264	0.01312	0.01363	0.01415	0.0147	0.01533	0.01628	0.01704	0.01782	0.01872	0.01968	0.02069	0.02171	0.02283	0.0241	0.02553	0.02711	0.02894	0.03105	0.03349];%	0.03637	0.03939	0.04194	0.04479	0.0479	0.05129	0.055	0.059	0.06353	0.0684	0.0739	0.08009	0.08681
Cm5=[	-0.0548	-0.055	-0.0543	-0.053	-0.0514	-0.0503	-0.0494	-0.0485	-0.0473	-0.0465	-0.0459	-0.0454	-0.0448	-0.0443	-0.0437	-0.0432	-0.0426	-0.0416	-0.0409	-0.0402	-0.0395	-0.0387	-0.0379	-0.037	-0.0361	-0.0351	-0.0341	-0.0329	-0.0322	-0.034	-0.0357	-0.0375	-0.0389	-0.0403	-0.042	-0.0436	-0.0442	-0.045	-0.0455	-0.0455	-0.0452	-0.0447	-0.044	-0.0437	-0.0436	-0.0435	-0.0434	-0.0433	-0.0433	-0.0433	-0.0432	-0.0432	-0.0431	-0.0432	-0.0432	-0.0433	-0.0433	-0.0434	-0.0434	-0.0435	-0.0436	-0.044	-0.0439	-0.0438	-0.0437	-0.0436	-0.0434	-0.0432	-0.043	-0.0427	-0.0421	-0.0414	-0.041	-0.0407	-0.0403	-0.04	-0.0397	-0.0393	-0.039	-0.0386	-0.0382	-0.0376	-0.0369	-0.036	-0.0351	-0.0358	-0.0357	-0.0356	-0.0355	-0.0354	-0.0352	-0.0349	-0.0345	-0.0338	-0.0334	-0.0332	-0.0329	-0.0326	-0.0323	-0.0317	-0.0312	-0.0308	-0.0304	-0.0299	-0.0291	-0.0282	-0.0274	-0.0265	-0.0256	-0.0246	-0.0232	-0.021	-0.0193	-0.0178	-0.0161	-0.0146	-0.0131	-0.0118	-0.0106	-0.0094	-0.0083	-0.0073	-0.0063	-0.0055	-0.0047];%	-0.0042	-0.0038	-0.0037	-0.0038	-0.0041	-0.0046	-0.0054	-0.0064	-0.0078	-0.0096	-0.0118	-0.0145	-0.0176

    

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
p_inf=P;
W_ep(q)=W_e(q-1)+W_payload;
W_l=1.1*W_ep(q);

% --- Logarithmic mean aircraft weight ---
if aircraft_case==8 && q==2
W_mean=(W_c0-W_cf)/log(W_c0/W_cf);
else
W_mean=(TOGW_i(q-1)-W_l)/log(TOGW_i(q-1)/W_l);
end

% W_mean=40000*9.81;


% --- Mach-angle parameter ---
if M>1
beta = sqrt((M)^2 - 1);
else
beta = sqrt(1-M^2);
end

if sweep_solver==1
sweep_low=0;
sweep_high=pi/2;
sweep_i(1)=(sweep_low+sweep_high)/2;
i_sw_max=12;
for i_sw=2:i_sw_max

% --- Wing reference area from stall-speed constraint ---

if aircraft_case==11 || aircraft_case==12
    
if M<1
CL_max=.9*max(Cl1)*cos(sweep_i(i_sw-1));
end 

else
    
if M<1
CL_max=.9*max(Cl)*cos(sweep_i(i_sw-1));
end

end

% S(q)=2*W_cc_q(q-1)/(V_stall_sl^2*rho_sl*CL_max);
% 
M_n = M*cos(sweep_i(i_sw-1)); %Normal wing component Mach number
% 
% C_L=W_mean/(.5*rho*V_inf^2*S(q))*(g-(V_inf^2/(radius_e+alt)))/g;
% 
% C_l=C_L*1.17/cos(sweep_i(i_sw-1))^2;

% C_p_0=-(2.5*tdc+.8*C_L);
% C_p_0=-(.01*tdc*(2.33+.01*tdc)+C_l*(.5+.005*tdc+.07*C_l));
% C_p_0=-.5;


M_cr_low=0;
M_cr_high=1;
M_cr=(M_cr_low+M_cr_high)/2;
for i=2:30
% --- Critical Mach-number / compressibility relation ---
if M<1
aa=C_p_0/(1-M^2)^.5;
aa=C_p_0*(cos(sweep_i(i_sw-1)))^2;
else
aa=C_p_0/(M^2-1)^.5;
aa=C_p_0*(cos(sweep_i(i_sw-1)))^2;
end

% aa=-.8;
% bb=2/(y*M_cr^2)*((((1+(y-1)/2*M_cr^2)/(1+(y-1)/2))^(y/(y-1)))-1);
bb=2/(y*M_cr^2)*(((2+(y-1)*M_cr^2)/(y+1))^(y/(y-1))-1);

if aa>bb
    
   M_cr=M_cr+(M_cr_high-M_cr_low)/2^i;
   
else
    
   M_cr=M_cr-(M_cr_high-M_cr_low)/2^i;
   
end

end

if M_cr<M_n
   sweep_i(i_sw)=sweep_i(i_sw-1)+(sweep_high-sweep_low)/2^i_sw;
else
   sweep_i(i_sw)=sweep_i(i_sw-1)-(sweep_high-sweep_low)/2^i_sw;
end

sweep=sweep_i(i_sw);

end

end

M_n = M*cos(sweep); %Normal wing component Mach number


% --- Wing reference area from stall-speed constraint ---
if aircraft_case==11 || aircraft_case==12
    
if M<1
CL_max=.9*max(Cl1)*cos(sweep);
end 

else
    
if M<1
CL_max=.9*max(Cl)*cos(sweep);
end

end

S(q)=2*W_cc_q(q-1)/(V_stall_sl^2*rho_sl*CL_max);



sweep_d=sweep*180/(pi);


if delta==1
% --- Delta-wing aspect ratio ---
AR=4/tan(sweep); %Delta wing
end

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
if sweep>30*pi/180
e=(4.61*(1-.045*AR^.68)*(cos(sweep))^.15-3.1); %Raymer
else
e=1.78*(1-.045*AR^.68)-.64; %Raymer
end

if delta==1
% --- Delta-wing root chord length ---
c_r=S(q)/(b/2);
else
c_r=2*S(q)/(b*(1+lambda));
end


c_t=lambda*c_r;

Re_w=rho*V_inf*(c_r+c_t)/2/mu;


% --- lift-curve slope ---
if M*cos(sweep)>1
C_l_alpha=4/(M_n^2-1)^.5*cos(sweep); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.17 page 331
C_L_alpha=C_l_alpha*(1-1/(2*AR*(M_n^2-1)^.5)); %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.18 page 331
else
C_l_alpha=2*pi/(1-M_n^2)^.5; %https://repo.poltekbangsby.ac.id/id/eprint/12/1/Flight%20Dynamics%20Principles,%20Second%20Edition%20A%20Linear%20Systems%20Approach%20to%20Aircraft%20Stability%20and%20Control%20by%20Michael%20V.%20Cook%20(z-lib.org).pdf eq. 12.14 page 330
eta_s = C_l_alpha/(2*pi/beta);
F = 1.07*(1 + D_f/b)^2;
C_L_alpha = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep_xc)^2/beta^2)))*F;
C_L_alpha_B = (2*pi*AR) / (2 + sqrt(4 + (AR^2*beta^2/eta_s^2) * (1 + tan(sweep_xc)^2/beta^2)));
end

% --- Engine inlet wave/form drag: D = q*S_inlet*CD ---
if type==0
    D_engine=.5*rho*V_inf^2*(pi*N_w^2*.25)*Cd_engine*N_en;
else
    D_engine=.5*rho*V_inf^2*(pi*N_w^2*.25)*Cd_engine*N_en; 
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

if R_cutoff>Re
   Re0=Re;
else
   Re0=R_cutoff;
end

c_f_to=.455/(((log10(Re_to))^2.58)*(1+.144*(V_lof/a_sl)^2)^.65);

% --- Supersonic turbulent skin-friction coefficient ---
if R_cutoff>Re
c_f=.455/(((log10(Re))^2.58)*(1+.144*M^2)^.65);
else
c_f=.455/(((log10(R_cutoff))^2.58)*(1+.144*M^2)^.65);
end

if aircraft_case==8
cf_low=0;
cf_high=.005;
i_cf_max=10;
c_f_i(1)=(cf_low+cf_high)/2;
for i_cf=2:i_cf_max
   c_f_a=.242/c_f_i(i_cf-1)^.5;
   c_f_b=log10(Re0*c_f_i(i_cf-1));
   
   if c_f_a>c_f_b
      c_f_i(i_cf)=c_f_i(i_cf-1)+(cf_high-cf_low)/2^(i_cf);
   else
      c_f_i(i_cf)=c_f_i(i_cf-1)-(cf_high-cf_low)/2^(i_cf);
   end
    
end

c_f=c_f_i(i_cf);

end
% --- Skin-friction drag coefficient ---
C_d_skin = c_f * FF;
% --- Skin-friction drag coefficient ---
C_d_skin_to = c_f_to * FF;

% --- Zero-lift wing wave-drag coefficient ---
if M >= 1
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

% --- Zero-lift drag coefficient ---
% if M>=1
C_d_0 = C_d_skin + C_d_wave_wing_0 + C_d_wave_body_0;
C_d_0_to=C_d_skin_to;
C_d_0_cc= C_d_skin + C_d_wave_body_0;
% else
    
% if aircraft_case==11 || aircraft_case==12
% C_d_0 = min(Cd1);
% C_d_0_to = min(Cd1);
% C_d_0_cc=C_d_skin  + C_d_wave_body_0;    
% else
% C_d_0 = min(Cd);
% C_d_0_to = min(Cd);
C_d_0_cc=C_d_skin  + C_d_wave_body_0;
% end
% end

if aircraft_case==8
C_L=(W_mean-L_comp)/(.5*rho*V_inf^2*S(q))*(g-(V_inf^2/(radius_e+alt)))/g; %Do W-mean-compression lift for waverider
% --- Max Lift coefficient including waverider compression lift ---
C_L_max_W=(TOGW_i(q-1)-L_comp_max)/(.5*rho*V_inf^2*S(q))*(g-(V_inf^2/(radius_e+alt)))/g;
else
% --- Lift coefficient including waverider compression lift ---
C_L=(W_mean-L_comp)/(.5*rho*V_inf^2*S(q)); %Do W-mean-compression lift for waverider
% --- Max Lift coefficient including waverider compression lift ---
C_L_max_W=(TOGW_i(q-1)-L_comp_max)/(.5*rho*V_inf^2*S(q));
end

if M>=1
alpha=C_L/C_L_alpha*180/pi;
alpha_inc=alpha*(cos(sweep))^2;
end

if (aircraft_case==11 || aircraft_case==12) && M<1
        
alpha_r_low=-5;
alpha_r_high=10;
alpha_r=(alpha_r_low+alpha_r_high)/2;
alpha_r_Dmax=(alpha_r_low+alpha_r_high)/2;

www_max=10;
for www=2:www_max
ww_max=10;
Lw1(1)=0;
dw1(1)=0;
S1(1)=0;
Lw2(1)=0;
dw2(1)=0;
S2(1)=0;
Lw3(1)=0;
dw3(1)=0;
S3(1)=0;
Lw4(1)=0;
dw4(1)=0;
S4(1)=0;
Lw5(1)=0;
dw5(1)=0;
S5(1)=0;

Lw1_Dmax(1)=0;
dw1_Dmax(1)=0;
Lw2_Dmax(1)=0;
dw2_Dmax(1)=0;
Lw3_Dmax(1)=0;
dw3_Dmax(1)=0;
Lw4_Dmax(1)=0;
dw4_Dmax(1)=0;
Lw5_Dmax(1)=0;
dw5_Dmax(1)=0;
for ww=2:ww_max+1
%     alpha_r=3;
    db1(ww)=sp1*b/2/ww_max;
%     if ww==2
%         fprintf('sp1=%f\nb=%f\nww_max=%f\n',sp1,b,ww_max)
%     end
    alphaw1(ww)=-washout*db1(ww)*(ww-1)/(b/2)+washout+alpha_r;
    alphaw1_Dmax=-washout*db1(ww)*(ww-1)/(b/2)+washout+alpha_r_Dmax;
    c1=(c_t-c_r)*2/b*(db1(ww)*(ww-1))+c_r;
    dS1=c1*db1(ww);
    S1(ww)=dS1+S1(ww-1);
    
    lowerIdxw1 = find(alphaw1(ww) >= Alpha1, 1, 'last');
    if lowerIdxw1 == length(Alpha1)
    upperIdxw1 = lowerIdxw1;
    else
    upperIdxw1 = lowerIdxw1 + 1;
    end
    
    lowerIdxw1_Dmax = find(alphaw1_Dmax >= Alpha1, 1, 'last');
    if lowerIdxw1_Dmax == length(Alpha1)
    upperIdxw1_Dmax = lowerIdxw1_Dmax;
    else
    upperIdxw1_Dmax = lowerIdxw1_Dmax + 1;
    end
    
    lowerIdxw1_alpha=Alpha1(lowerIdxw1);
    upperIdxw1_alpha=Alpha1(upperIdxw1);
    
    lowerIdxw1_alpha_Dmax=Alpha1(lowerIdxw1_Dmax);
    upperIdxw1_alpha_Dmax=Alpha1(upperIdxw1_Dmax);
    
    lowerIdxw1_Cl=Cl1(lowerIdxw1);
    upperIdxw1_Cl=Cl1(upperIdxw1);
    lowerIdxw1_Cd=Cd1(lowerIdxw1);
    upperIdxw1_Cd=Cd1(upperIdxw1);
    sl_Clw1=(upperIdxw1_Cl-lowerIdxw1_Cl)/(upperIdxw1_alpha-lowerIdxw1_alpha);
    sl_Cdw1=(upperIdxw1_Cd-lowerIdxw1_Cd)/(upperIdxw1_alpha-lowerIdxw1_alpha);
    co_Clw1=lowerIdxw1_Cl-sl_Clw1*lowerIdxw1_alpha;
    co_Cdw1=lowerIdxw1_Cd-sl_Cdw1*lowerIdxw1_alpha;
    C_lw1(ww)=sl_Clw1*alphaw1(ww)+co_Clw1;
    C_dw1(ww)=sl_Cdw1*alphaw1(ww)+co_Cdw1;
    dLw1=C_lw1(ww)*dS1;
    Lw1(ww)=dLw1+Lw1(ww-1);
    C_l1=Lw1(ww)/(.5*S(q));
    ddw1=C_dw1(ww)*dS1;
    dw1(ww)=ddw1+dw1(ww-1);
    C_d1=dw1(ww)/(.5*S(q));
    
    
    
    lowerIdxw1_Cl_Dmax=Cl1(lowerIdxw1_Dmax);
    upperIdxw1_Cl_Dmax=Cl1(upperIdxw1_Dmax);
    lowerIdxw1_Cd_Dmax=Cd1(lowerIdxw1_Dmax);
    upperIdxw1_Cd_Dmax=Cd1(upperIdxw1_Dmax);
    sl_Clw1_Dmax=(upperIdxw1_Cl_Dmax-lowerIdxw1_Cl_Dmax)/(upperIdxw1_alpha_Dmax-lowerIdxw1_alpha_Dmax);
    sl_Cdw1_Dmax=(upperIdxw1_Cd_Dmax-lowerIdxw1_Cd_Dmax)/(upperIdxw1_alpha_Dmax-lowerIdxw1_alpha_Dmax);
    co_Clw1_Dmax=lowerIdxw1_Cl_Dmax-sl_Clw1_Dmax*lowerIdxw1_alpha_Dmax;
    co_Cdw1_Dmax=lowerIdxw1_Cd_Dmax-sl_Cdw1_Dmax*lowerIdxw1_alpha_Dmax;
    C_lw1_Dmax(ww)=sl_Clw1_Dmax*alphaw1_Dmax+co_Clw1_Dmax;
    C_dw1_Dmax(ww)=sl_Cdw1_Dmax*alphaw1_Dmax+co_Cdw1_Dmax;
    dLw1_Dmax=C_lw1_Dmax(ww)*dS1;
    Lw1_Dmax(ww)=dLw1_Dmax+Lw1_Dmax(ww-1);
    C_l1_Dmax=Lw1_Dmax(ww)/(.5*S(q));
    ddw1_Dmax=C_dw1_Dmax(ww)*dS1;
    dw1_Dmax(ww)=ddw1_Dmax+dw1_Dmax(ww-1);
    C_d1_Dmax=dw1_Dmax(ww)/(.5*S(q));
    
    
    
    
    
    
    
    db2=(sp2-sp1)*b/2/ww_max;
    alphaw2=-washout*db2*(ww-1)/(b/2)+alphaw1(ww);
    alphaw2_Dmax=-washout*db2*(ww-1)/(b/2)+alphaw1_Dmax;
    b2=db2*(ww-1)+sp1*b/2;
    c2=(c_t-c_r)*2/b*b2+c_r;
    dS2=c2*db2;
    S2(ww)=dS2+S2(ww-1);
    
    lowerIdxw2 = find(alphaw2 >= Alpha2, 1, 'last');
    if lowerIdxw2 == length(Alpha2)
    upperIdxw2 = lowerIdxw2;
    else
    upperIdxw2 = lowerIdxw2 + 1;
    end
    
    lowerIdxw2_Dmax = find(alphaw2_Dmax >= Alpha2, 1, 'last');
    if lowerIdxw2_Dmax == length(Alpha2)
    upperIdxw2_Dmax = lowerIdxw2_Dmax;
    else
    upperIdxw2_Dmax = lowerIdxw2_Dmax + 1;
    end
    
    lowerIdxw2_alpha=Alpha2(lowerIdxw2);
    upperIdxw2_alpha=Alpha2(upperIdxw2);
    
    lowerIdxw2_alpha_Dmax=Alpha2(lowerIdxw2_Dmax);
    upperIdxw2_alpha_Dmax=Alpha2(upperIdxw2_Dmax);
    
    lowerIdxw2_Cl=Cl2(lowerIdxw2);
    upperIdxw2_Cl=Cl2(upperIdxw2);
    lowerIdxw2_Cd=Cd2(lowerIdxw2);
    upperIdxw2_Cd=Cd2(upperIdxw2);
    sl_Clw2=(upperIdxw2_Cl-lowerIdxw2_Cl)/(upperIdxw2_alpha-lowerIdxw2_alpha);
    sl_Cdw2=(upperIdxw2_Cd-lowerIdxw2_Cd)/(upperIdxw2_alpha-lowerIdxw2_alpha);
    co_Clw2=lowerIdxw2_Cl-sl_Clw2*lowerIdxw2_alpha;
    co_Cdw2=lowerIdxw2_Cd-sl_Cdw2*lowerIdxw2_alpha;
    C_lw2(ww)=sl_Clw2*alphaw2+co_Clw2;
    C_dw2(ww)=sl_Cdw2*alphaw2+co_Cdw2;
    dLw2=C_lw2(ww)*dS2;
    Lw2(ww)=dLw2+Lw2(ww-1);
    C_l2=Lw2(ww)/(.5*S(q));
    ddw2=C_dw2(ww)*dS2;
    dw2(ww)=ddw2+dw2(ww-1);
    C_d2=dw2(ww)/(.5*S(q));
    
    
    
    lowerIdxw2_Cl_Dmax=Cl2(lowerIdxw2_Dmax);
    upperIdxw2_Cl_Dmax=Cl2(upperIdxw2_Dmax);
    lowerIdxw2_Cd_Dmax=Cd2(lowerIdxw2_Dmax);
    upperIdxw2_Cd_Dmax=Cd2(upperIdxw2_Dmax);
    sl_Clw2_Dmax=(upperIdxw2_Cl_Dmax-lowerIdxw2_Cl_Dmax)/(upperIdxw2_alpha_Dmax-lowerIdxw2_alpha_Dmax);
    sl_Cdw2_Dmax=(upperIdxw2_Cd_Dmax-lowerIdxw2_Cd_Dmax)/(upperIdxw2_alpha_Dmax-lowerIdxw2_alpha_Dmax);
    co_Clw2_Dmax=lowerIdxw2_Cl_Dmax-sl_Clw2_Dmax*lowerIdxw2_alpha_Dmax;
    co_Cdw2_Dmax=lowerIdxw2_Cd_Dmax-sl_Cdw2_Dmax*lowerIdxw2_alpha_Dmax;
    C_lw2_Dmax(ww)=sl_Clw2_Dmax*alphaw2_Dmax+co_Clw2_Dmax;
    C_dw2_Dmax(ww)=sl_Cdw2_Dmax*alphaw2_Dmax+co_Cdw2_Dmax;
    dLw2_Dmax=C_lw2_Dmax(ww)*dS2;
    Lw2_Dmax(ww)=dLw2_Dmax+Lw2_Dmax(ww-1);
    C_l2_Dmax=Lw2_Dmax(ww)/(.5*S(q));
    ddw2_Dmax=C_dw2_Dmax(ww)*dS2;
    dw2_Dmax(ww)=ddw2_Dmax+dw2_Dmax(ww-1);
    C_d2_Dmax=dw2_Dmax(ww)/(.5*S(q));
    
    
    
    
    
    db3=(sp3-sp2)*b/2/ww_max;
    alphaw3=-washout*db3*(ww-1)/(b/2)+alphaw2;
    alphaw3_Dmax=-washout*db3*(ww-1)/(b/2)+alphaw2_Dmax;
    b3=db3*(ww-1)+sp2*b/2;
    c3=(c_t-c_r)*2/b*b3+c_r;
    dS3=c3*db3;
    S3(ww)=dS3+S3(ww-1);
    
    lowerIdxw3 = find(alphaw3 >= Alpha3, 1, 'last');
    if lowerIdxw3 == length(Alpha3)
    upperIdxw3 = lowerIdxw3;
    else
    upperIdxw3 = lowerIdxw3 + 1;
    end
    
    lowerIdxw3_Dmax = find(alphaw3_Dmax >= Alpha3, 1, 'last');
    if lowerIdxw3_Dmax == length(Alpha3)
    upperIdxw3_Dmax = lowerIdxw3_Dmax;
    else
    upperIdxw3_Dmax = lowerIdxw3_Dmax + 1;
    end
    
    lowerIdxw3_alpha=Alpha3(lowerIdxw3);
    upperIdxw3_alpha=Alpha3(upperIdxw3);
    
    lowerIdxw3_alpha_Dmax=Alpha3(lowerIdxw3_Dmax);
    upperIdxw3_alpha_Dmax=Alpha3(upperIdxw3_Dmax);
    
    lowerIdxw3_Cl=Cl3(lowerIdxw3);
    upperIdxw3_Cl=Cl3(upperIdxw3);
    lowerIdxw3_Cd=Cd3(lowerIdxw3);
    upperIdxw3_Cd=Cd3(upperIdxw3);
    sl_Clw3=(upperIdxw3_Cl-lowerIdxw3_Cl)/(upperIdxw3_alpha-lowerIdxw3_alpha);
    sl_Cdw3=(upperIdxw3_Cd-lowerIdxw3_Cd)/(upperIdxw3_alpha-lowerIdxw3_alpha);
    co_Clw3=lowerIdxw3_Cl-sl_Clw3*lowerIdxw3_alpha;
    co_Cdw3=lowerIdxw3_Cd-sl_Cdw3*lowerIdxw3_alpha;
    C_lw3(ww)=sl_Clw3*alphaw3+co_Clw3;
    C_dw3(ww)=sl_Cdw3*alphaw3+co_Cdw3;
    dLw3=C_lw3(ww)*dS3;
    Lw3(ww)=dLw3+Lw3(ww-1);
    C_l3=Lw3(ww)/(.5*S(q));
    ddw3(ww)=C_dw3(ww)*dS3;
    dw3(ww)=ddw3(ww)+dw3(ww-1);
    C_d3=dw3(ww)/(.5*S(q));
    
    
    
    lowerIdxw3_Cl_Dmax=Cl3(lowerIdxw3_Dmax);
    upperIdxw3_Cl_Dmax=Cl3(upperIdxw3_Dmax);
    lowerIdxw3_Cd_Dmax=Cd3(lowerIdxw3_Dmax);
    upperIdxw3_Cd_Dmax=Cd3(upperIdxw3_Dmax);
    sl_Clw3_Dmax=(upperIdxw3_Cl_Dmax-lowerIdxw3_Cl_Dmax)/(upperIdxw3_alpha_Dmax-lowerIdxw3_alpha_Dmax);
    sl_Cdw3_Dmax=(upperIdxw3_Cd_Dmax-lowerIdxw3_Cd_Dmax)/(upperIdxw3_alpha_Dmax-lowerIdxw3_alpha_Dmax);
    co_Clw3_Dmax=lowerIdxw3_Cl_Dmax-sl_Clw3_Dmax*lowerIdxw3_alpha_Dmax;
    co_Cdw3_Dmax=lowerIdxw3_Cd_Dmax-sl_Cdw3_Dmax*lowerIdxw3_alpha_Dmax;
    C_lw3_Dmax(ww)=sl_Clw3_Dmax*alphaw3_Dmax+co_Clw3_Dmax;
    C_dw3_Dmax(ww)=sl_Cdw3_Dmax*alphaw3_Dmax+co_Cdw3_Dmax;
    dLw3_Dmax=C_lw3_Dmax(ww)*dS3;
    Lw3_Dmax(ww)=dLw3_Dmax+Lw3_Dmax(ww-1);
    C_l3_Dmax=Lw3_Dmax(ww)/(.5*S(q));
    ddw3_Dmax=C_dw3_Dmax(ww)*dS3;
    dw3_Dmax(ww)=ddw3_Dmax+dw3_Dmax(ww-1);
    C_d3_Dmax=dw3_Dmax(ww)/(.5*S(q));
    
    
    
    
    
    
    
    
    db4=(sp4-sp3)*b/2/ww_max;
    alphaw4=-washout*db4*(ww-1)/(b/2)+alphaw3;
    alphaw4_Dmax=-washout*db4*(ww-1)/(b/2)+alphaw3_Dmax;
    b4=db4*(ww-1)+sp3*b/2;
    c4=(c_t-c_r)*2/b*b4+c_r;
    dS4=c4*db4;
    S4(ww)=dS4+S4(ww-1);
    
    lowerIdxw4 = find(alphaw4 >= Alpha4, 1, 'last');
    if lowerIdxw4 == length(Alpha4)
    upperIdxw4 = lowerIdxw4;
    else
    upperIdxw4 = lowerIdxw4 + 1;
    end
    
    lowerIdxw4_Dmax = find(alphaw4_Dmax >= Alpha4, 1, 'last');
    if lowerIdxw4_Dmax == length(Alpha4)
    upperIdxw4_Dmax = lowerIdxw4_Dmax;
    else
    upperIdxw4_Dmax = lowerIdxw4_Dmax + 1;
    end
    
    lowerIdxw4_alpha=Alpha4(lowerIdxw4);
    upperIdxw4_alpha=Alpha4(upperIdxw4);
    
    lowerIdxw4_alpha_Dmax=Alpha4(lowerIdxw4_Dmax);
    upperIdxw4_alpha_Dmax=Alpha4(upperIdxw4_Dmax);
    
    lowerIdxw4_Cl=Cl4(lowerIdxw4);
    upperIdxw4_Cl=Cl4(upperIdxw4);
    lowerIdxw4_Cd=Cd4(lowerIdxw4);
    upperIdxw4_Cd=Cd4(upperIdxw4);
    sl_Clw4=(upperIdxw4_Cl-lowerIdxw4_Cl)/(upperIdxw4_alpha-lowerIdxw4_alpha);
    sl_Cdw4=(upperIdxw4_Cd-lowerIdxw4_Cd)/(upperIdxw4_alpha-lowerIdxw4_alpha);
    co_Clw4=lowerIdxw4_Cl-sl_Clw4*lowerIdxw4_alpha;
    co_Cdw4=lowerIdxw4_Cd-sl_Cdw4*lowerIdxw4_alpha;
    C_lw4(ww)=sl_Clw4*alphaw4+co_Clw4;
    C_dw4(ww)=sl_Cdw4*alphaw4+co_Cdw4;
    dLw4=C_lw4(ww)*dS4;
    Lw4(ww)=dLw4+Lw4(ww-1);
    C_l4=Lw4(ww)/(.5*S(q));
    ddw4=C_dw4(ww)*dS4;
    dw4(ww)=ddw4+dw4(ww-1);
    C_d4=dw4(ww)/(.5*S(q));
    
    lowerIdxw4_Cl_Dmax=Cl4(lowerIdxw4_Dmax);
    upperIdxw4_Cl_Dmax=Cl4(upperIdxw4_Dmax);
    lowerIdxw4_Cd_Dmax=Cd4(lowerIdxw4_Dmax);
    upperIdxw4_Cd_Dmax=Cd4(upperIdxw4_Dmax);
    sl_Clw4_Dmax=(upperIdxw4_Cl_Dmax-lowerIdxw4_Cl_Dmax)/(upperIdxw4_alpha_Dmax-lowerIdxw4_alpha_Dmax);
    sl_Cdw4_Dmax=(upperIdxw4_Cd_Dmax-lowerIdxw4_Cd_Dmax)/(upperIdxw4_alpha_Dmax-lowerIdxw4_alpha_Dmax);
    co_Clw4_Dmax=lowerIdxw4_Cl_Dmax-sl_Clw4_Dmax*lowerIdxw4_alpha_Dmax;
    co_Cdw4_Dmax=lowerIdxw4_Cd_Dmax-sl_Cdw4_Dmax*lowerIdxw4_alpha_Dmax;
    C_lw4_Dmax(ww)=sl_Clw4_Dmax*alphaw4_Dmax+co_Clw4_Dmax;
    C_dw4_Dmax(ww)=sl_Cdw4_Dmax*alphaw4_Dmax+co_Cdw4_Dmax;
    dLw4_Dmax=C_lw4_Dmax(ww)*dS4;
    Lw4_Dmax(ww)=dLw4_Dmax+Lw4_Dmax(ww-1);
    C_l4_Dmax=Lw4_Dmax(ww)/(.5*S(q));
    ddw4_Dmax=C_dw4_Dmax(ww)*dS4;
    dw4_Dmax(ww)=ddw4_Dmax+dw4_Dmax(ww-1);
    C_d4_Dmax=dw4_Dmax(ww)/(.5*S(q));
    
    
    
    
    
    
    
    
    
    db5=(1-sp4)*b/2/ww_max;
    alphaw5=-washout*db5*(ww-1)/(b/2)+alphaw4;
    alphaw5_Dmax=-washout*db5*(ww-1)/(b/2)+alphaw4_Dmax;
    b5=db5*(ww-1)+sp4*b/2;
    c5=(c_t-c_r)*2/b*b5+c_r;
    dS5=c5*db5;
    S5(ww)=dS5+S5(ww-1);
    
    lowerIdxw5 = find(alphaw5 >= Alpha5, 1, 'last');
    if lowerIdxw5 == length(Alpha5)
    upperIdxw5 = lowerIdxw5;
    else
    upperIdxw5 = lowerIdxw5 + 1;
    end
    
    
    lowerIdxw5_Dmax = find(alphaw5_Dmax >= Alpha5, 1, 'last');
    if lowerIdxw5_Dmax == length(Alpha5)
    upperIdxw5_Dmax = lowerIdxw5_Dmax;
    else
    upperIdxw5_Dmax = lowerIdxw5_Dmax + 1;
    end
    
    lowerIdxw5_alpha=Alpha5(lowerIdxw5);
    upperIdxw5_alpha=Alpha5(upperIdxw5);
    
    lowerIdxw5_alpha_Dmax=Alpha5(lowerIdxw5_Dmax);
    upperIdxw5_alpha_Dmax=Alpha5(upperIdxw5_Dmax);
    
    lowerIdxw5_Cl=Cl5(lowerIdxw5);
    upperIdxw5_Cl=Cl5(upperIdxw5);
    lowerIdxw5_Cd=Cd5(lowerIdxw5);
    upperIdxw5_Cd=Cd5(upperIdxw5);
    sl_Clw5=(upperIdxw5_Cl-lowerIdxw5_Cl)/(upperIdxw5_alpha-lowerIdxw5_alpha);
    sl_Cdw5=(upperIdxw5_Cd-lowerIdxw5_Cd)/(upperIdxw5_alpha-lowerIdxw5_alpha);
    co_Clw5=lowerIdxw5_Cl-sl_Clw5*lowerIdxw5_alpha;
    co_Cdw5=lowerIdxw5_Cd-sl_Cdw5*lowerIdxw5_alpha;
    C_lw5(ww)=sl_Clw5*alphaw5+co_Clw5;
    C_dw5(ww)=sl_Cdw5*alphaw5+co_Cdw5;
    dLw5=C_lw5(ww)*dS5;
    Lw5(ww)=dLw5+Lw5(ww-1);
    C_l5=Lw5(ww)/(.5*S(q));
    
    
    
    
    lowerIdxw5_Cl_Dmax=Cl5(lowerIdxw5_Dmax);
    upperIdxw5_Cl_Dmax=Cl5(upperIdxw5_Dmax);
    lowerIdxw5_Cd_Dmax=Cd5(lowerIdxw5_Dmax);
    upperIdxw5_Cd_Dmax=Cd5(upperIdxw5_Dmax);
    sl_Clw5_Dmax=(upperIdxw5_Cl_Dmax-lowerIdxw5_Cl_Dmax)/(upperIdxw5_alpha_Dmax-lowerIdxw5_alpha_Dmax);
    sl_Cdw5_Dmax=(upperIdxw5_Cd_Dmax-lowerIdxw5_Cd_Dmax)/(upperIdxw5_alpha_Dmax-lowerIdxw5_alpha_Dmax);
    co_Clw5_Dmax=lowerIdxw5_Cl_Dmax-sl_Clw5_Dmax*lowerIdxw5_alpha_Dmax;
    co_Cdw5_Dmax=lowerIdxw5_Cd_Dmax-sl_Cdw5_Dmax*lowerIdxw5_alpha_Dmax;
    C_lw5_Dmax(ww)=sl_Clw5_Dmax*alphaw5_Dmax+co_Clw5_Dmax;
    C_dw5_Dmax(ww)=sl_Cdw5_Dmax*alphaw5_Dmax+co_Cdw5_Dmax;
    dLw5_Dmax=C_lw5_Dmax(ww)*dS5;
    Lw5_Dmax(ww)=dLw5_Dmax+Lw5_Dmax(ww-1);
    C_l5_Dmax=Lw5_Dmax(ww)/(.5*S(q));
    
    
    ddw5=C_dw5(ww)*dS5;
    dw5(ww)=ddw5+dw5(ww-1);
    C_d5=dw5(ww)/(.5*S(q));
    
    
    ddw5_Dmax=C_dw5_Dmax(ww)*dS5;
    dw5_Dmax(ww)=ddw5_Dmax+dw5_Dmax(ww-1);
    C_d5_Dmax=dw5_Dmax(ww)/(.5*S(q));
    
    C_lt=C_l1 + C_l2 + C_l3 + C_l4 + C_l5;
    C_dt=C_d1 + C_d2 + C_d3 + C_d4 + C_d5;
    
    C_lt_Dmax=C_l1_Dmax + C_l2_Dmax + C_l3_Dmax + C_l4_Dmax + C_l5_Dmax;
    C_dt_Dmax=C_d1_Dmax + C_d2_Dmax + C_d3_Dmax + C_d4_Dmax + C_d5_Dmax;
    
    
end

L_s_2d=C_l_alpha; 
L_s_3d=C_L_alpha;
% C_L=.46;
C_l=L_s_2d/L_s_3d*C_L;

if C_lt<C_l
    alpha_r=alpha_r+(alpha_r_high-alpha_r_low)/2^www;
else
    alpha_r=alpha_r-(alpha_r_high-alpha_r_low)/2^www;   
end

alpha=alpha_r+washout;

end

C_D=C_dt+C_L^2/(pi*AR*e);
D_w=C_D*.5*rho*V_inf^2*S(q);

C_D_Dmax=C_dt_Dmax+C_L_max_W^2/(pi*AR*e);
D_max_0=C_D_Dmax*.5*rho*V_inf^2*S(q);

% Alpha5=[	-9.5	-9.25	-9	-8.75	-8.5	-7.25	-7	-6.75	-6.5	-6.25	-5.75	-5.5	-5.25	-5	-4.75	-4.25	-4	-3.75	-3.5	-3.25	-3	-2.75	-2.5	-2.25	-2	-1.75	-1.5	-1.25	-1	-0.75	-0.5	-0.25	0	0.25	0.5	0.75	1	1.25	1.5	1.75	2	2.25	2.5	2.75	3	3.25	3.5	3.75	4	4.25	4.5	4.75	5	5.25	5.5	5.75	6	6.25	6.5	6.75	7	7.25	7.5	7.75	8	8.25	8.5	8.75	9	9.25	9.5	9.75	10	10.25	10.5];%	10.75	11	11.25	11.5	11.75
% Cl5=[	-0.3718	-0.3743	-0.3778	-0.382	-0.3881	-0.4941	-0.4756	-0.4535	-0.4276	-0.4045	-0.3342	-0.3011	-0.2745	-0.2557	-0.2371	-0.2058	-0.1827	-0.1588	-0.1344	-0.1091	-0.0833	-0.0561	-0.0291	-0.0055	0.0206	0.0475	0.0725	0.0982	0.1235	0.1488	0.172	0.1967	0.2215	0.2466	0.2717	0.2966	0.3203	0.3326	0.3403	0.3534	0.3704	0.4359	0.4849	0.5188	0.5547	0.5902	0.6171	0.6386	0.6607	0.6829	0.705	0.7272	0.7481	0.7738	0.7985	0.8192	0.8412	0.8631	0.8841	0.9044	0.9242	0.9437	0.9622	0.98	0.9951	1.0103	1.0261	1.0381	1.053	1.0671	1.0823	1.0947	1.1104	1.1204	1.1284];%	1.1283	1.1238	1.1152	1.0974	1.0813
% Cd5=[	0.08509	0.08065	0.07611	0.07165	0.06704	0.05427	0.05046	0.04775	0.0445	0.04145	0.03648	0.03348	0.03062	0.02829	0.0262	0.01905	0.01798	0.017	0.01616	0.01525	0.01448	0.01453	0.0146	0.01176	0.01124	0.01111	0.01019	0.00917	0.0089	0.00869	0.0081	0.00779	0.00761	0.00751	0.00748	0.00751	0.00768	0.00892	0.00916	0.00848	0.00811	0.00821	0.00841	0.00857	0.00872	0.00889	0.00909	0.00929	0.00945	0.00962	0.00981	0.01002	0.01034	0.01026	0.01028	0.01066	0.01095	0.01126	0.01166	0.01213	0.01265	0.01319	0.01383	0.01452	0.01546	0.01642	0.01735	0.01882	0.02009	0.02164	0.02278	0.02451	0.02517	0.02686	0.02826];%	0.03038	0.0328	0.03552	0.03921	0.04269
% Cdp5=[	0.08356	0.07913	0.0746	0.07015	0.06556	0.05238	0.04848	0.0457	0.04231	0.03913	0.03376	0.03052	0.02734	0.02466	0.02226	0.0144	0.01319	0.01209	0.01112	0.0101	0.00919	0.00915	0.00913	0.00611	0.0056	0.00542	0.00448	0.00334	0.00303	0.00279	0.00214	0.00175	0.00146	0.00124	0.00109	0.00098	0.00091	0.00115	0.00143	0.00157	0.0017	0.002	0.00219	0.00233	0.00248	0.00266	0.00286	0.00308	0.00324	0.00339	0.00359	0.00379	0.00412	0.00399	0.00395	0.00419	0.00451	0.00488	0.00534	0.00591	0.00649	0.00711	0.00782	0.00858	0.00959	0.01065	0.01167	0.01326	0.01465	0.01633	0.01767	0.01958	0.02039	0.02227	0.02386];%	0.02622	0.02889	0.03186	0.03584	0.03956
% Cm5=[	-0.0306	-0.032	-0.0334	-0.035	-0.0371	-0.0417	-0.0449	-0.0472	-0.0501	-0.0515	-0.0557	-0.0579	-0.0585	-0.0571	-0.0555	-0.0521	-0.0515	-0.051	-0.0504	-0.0499	-0.0494	-0.0491	-0.0487	-0.048	-0.0478	-0.0475	-0.0471	-0.0458	-0.0452	-0.0446	-0.0436	-0.0429	-0.0422	-0.0416	-0.041	-0.0403	-0.0395	-0.0369	-0.0338	-0.0311	-0.0286	-0.0369	-0.0417	-0.0432	-0.0452	-0.0471	-0.0471	-0.0459	-0.0449	-0.0439	-0.0428	-0.0418	-0.0406	-0.0403	-0.0398	-0.0385	-0.0374	-0.0362	-0.035	-0.0336	-0.0321	-0.0306	-0.029	-0.0273	-0.0252	-0.0231	-0.0212	-0.0187	-0.0168	-0.0148	-0.013	-0.0109	-0.0092	-0.0068	-0.0041];%	-0.0004	0.0036	0.0075	0.0114	0.014
% sp4=.54;
    
    
else
if M<1
    

L_s_2d=C_l_alpha; 
L_s_3d=C_L_alpha;
% C_l=C_L*1.17/(cos(sweep))^2; 
% C_L=.35;
C_l=L_s_2d/L_s_3d*C_L;

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
alpha=((C_l-lowerIdx_Cl)/(upperIdx_Cl-lowerIdx_Cl))*(upperIdx_alpha-lowerIdx_alpha)+lowerIdx_alpha;

if lowerIdx_alpha==upperIdx_alpha
C_d=Cd(lowerIdx);
else
C_d=sl_Cd*alpha+co_Cd;
end

C_D=C_d+C_L^2/(pi*AR*e);
D_w=C_D*.5*rho*V_inf^2*S(q);
alpha_inc=alpha*(cos(sweep))^2;

C_l_Dmax=C_L_max_W*L_s_2d/L_s_3d;
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
L_s_2d=C_l_alpha;               
L_s_3d=C_L_alpha;
C_l=C_L*L_s_2d/L_s_3d;
C_l_Dmax=C_L_max_W;
end

end

alpha_ii=(C_L/(pi*AR*e))*180/pi;
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

D_p=.5*rho*V_inf^2*S_wet*c_f;
D_p_f=.5*rho*V_inf^2*S_fus*c_f;

if M>1
% --- Maximum induced drag coefficient ---
c_d_i_max=K*C_L_max_W^2;
% --- Parasite drag force ---
D_p=.5*rho*V_inf^2*S_wet*c_f;

if aircraft_case==8
% --- Induced drag force ---
D_i=c_d_i*.5*rho*V_inf^2*S(q);
% --- Maximum induced drag force ---
D_i_max=c_d_i_max*.5*rho*V_inf^2*S(q);
else
% --- Induced drag force ---
D_i=c_d_i*.5*rho*V_inf^2*S(q);
% --- Maximum induced drag force ---
D_i_max=c_d_i_max*.5*rho*V_inf^2*S(q);
% --- Lift-dependent supersonic wave drag ---
% C_d_wave_wing_i=4*alpha^2/((M^2-1)^.5);
end

if M_n >= 1
    C_d_wave_wing_i = (C_L^2/4)*sqrt(M_n^2 - 1)/cos(sweep);
    C_d_wave_wing_i_max = (C_L_max_W^2/4)*sqrt(M_n^2 - 1)/(cos(sweep));
else 
    C_d_wave_wing_i = 0;    % do not use Ackeret
    C_d_wave_wing_i_max=0;
end

if aircraft_case==8
    
% % --- Max lift-dependent supersonic wave drag ---
% C_d_wave_wing_i_max=4*alpha_max^2/((M^2-1)^.5);
% --- Maximum total wave drag ---
D_wave_max = (C_d_wave_wing_i_max) * 0.5*rho*V_inf^2*S(q);   
% --- Lift-dependent wave-drag force ---
D_wave_i=C_d_wave_wing_i*.5*rho*V_inf^2*S(q);
% --- Max lift-dependent wave-drag force ---
D_wave_i_max=C_d_wave_wing_i_max*.5*rho*V_inf^2*S(q); 
else
    
% % --- Max lift-dependent supersonic wave drag ---
% C_d_wave_wing_i_max=4*alpha_max^2/((M^2-1)^.5);
% --- Maximum total wave drag ---
D_wave_max = (C_d_wave_wing_i_max) * 0.5*rho*V_inf^2*S(q);   
% --- Lift-dependent wave-drag force ---
D_wave_i=C_d_wave_wing_i*.5*rho*V_inf^2*S(q);
% --- Max lift-dependent wave-drag force ---
D_wave_i_max=C_d_wave_wing_i_max*.5*rho*V_inf^2*S(q);

end
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
CDi_H = 0;
CD_H = CD0_H + CDi_H;
D_H = q_d*S_H*CD_H;

% ---- Vertical tail ----
AR_V = b_V^2/S_V;
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

if M>1
    
% --- Zero-lift drag force ---
c_d_0_t=(C_d_0+Cd_engine_s+CD0_H+CD0_V);
D_0= c_d_0_t*.5*rho*V_inf^2*S(q); 

end

c_d_0_c=C_d_0_cc + Cd_engine_s+CD0_H+CD0_V;

% else
%     
% % --- Zero-lift drag force ---
% c_d_0_t=(C_d_0+Cd_engine_s+CD0_H+CD0_V);
% D_0= c_d_0_t*.5*rho*V_inf^2*S(q); 
% c_d_0_c=C_d_0_cc + Cd_engine_s+CD0_H+CD0_V;
% 
% end

if M>=1
    
if aircraft_case==8    
% --- Total cruise drag ---
D = D_0 + D_i + D_wave_i;
CD=D/(.5*rho*V_inf^2*S(q));
else
% --- Total cruise drag ---
D = D_0 + D_i + D_wave_i;
CD=D/(.5*rho*V_inf^2*S(q));
end

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

if BR==0
    FPR=0;
end

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
T_0_5 = TIT_d - (W_c + BR*W_ff)/(Cp*(1+f)*eta_m);
T_0_5_Dmax = TIT_d_Dmax - (W_c + BR*W_f_Dmax)/(Cp*(1+f)*eta_m);
T_0_5_sl = TIT_d - (W_c_sl + BR*W_f_sl)/(Cp*(1+f_sl)*eta_m);
T_0_5_max = TIT_d_high - (W_c + BR*W_ff)/(Cp*(1+f)*eta_m);
% --- actual turbine exit temperature ---
T_0_5s = TIT_d - (TIT_d-T_0_5)/eta_t;
T_0_5s_Dmax = TIT_d_Dmax - (TIT_d_Dmax-T_0_5)/eta_t;
T_0_5s_sl = TIT_d - (TIT_d-T_0_5_sl)/eta_t;
T_0_5s_max = TIT_d_high - (TIT_d_high-T_0_5_max)/eta_t;
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

% --- Core exhaust velocity from energy equation ---
V_e_core_max = (sqrt(2*Cp*(T_0_5_max - T_e_max)));

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
if Th<D/N_en    
    TIT_d=TIT_d+(TIT_d_high-TIT_d_low)/2^k;    
else   
    TIT_d=TIT_d-(TIT_d_high-TIT_d_low)/2^k;    
end

if Th<D_max/N_en    
    TIT_d_Dmax=TIT_d_Dmax+(TIT_d_high-TIT_d_low)/2^k;    
else   
    TIT_d_Dmax=TIT_d_Dmax-(TIT_d_high-TIT_d_low)/2^k;    
end

end

% --- Including afterburner ---

n_ab=0; %Initial value for afterburner fuel fraction compared to the stoichometric ratio
if (Th+1)<D/N_en && Afterburner==1
   
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

if T_k<D/N_en    
    n_ab=n_ab+(n_ab_high-n_ab_low)/2^kk;    
else
    
    n_ab=n_ab-(n_ab_high-n_ab_low)/2^kk;    
end

end   

end

n_ab_Dmax=0;
if (Th_Dmax+1)<D_max/N_en && Afterburner==1
   
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

if T_k_Dmax<D_max/N_en    
    n_ab_Dmax=n_ab_Dmax+(n_ab_high-n_ab_low)/2^kk_Dmax;    
else
    
    n_ab_Dmax=n_ab_Dmax-(n_ab_high-n_ab_low)/2^kk_Dmax;    
end

end   

end


% --- Inverse Thrust-specific fuel consumption ---
if (Th+1)<D/N_en && Afterburner==1
    TSFC_k=T_k/m_f_ab_k;    
else
    TSFC_k=Th/m_f;
end

if (Th_Dmax+1)<D_max/N_en && Afterburner==1
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
  
if (Th+1)<D/N_en && type>0 && Afterburner==1

% --- Thrust specific fuel consumption ---

if D/N_en<(T_k+1)
TFC=1/TSFC_k;
else
TFC=1/0;
end


elseif type==0 && Afterburner==1  
    
TFC=(m_f_ab_k_e)/(D/N_en);

elseif type==0 && Afterburner==0

if D/N_en<(Th_e+1)
    
TFC=(m_f_e)/(D/N_en);

else

TFC=1/0;

end

else
    
if D/N_en<(Th+1)
 
TFC=(m_f)/(Th);

else
 
TFC=1/0;

end
        
end

if (Th_Dmax+1)<D_max/N_en && type>0

% --- Thrust specific fuel consumption ---
if Afterburner==1
if D_max/N_en<(T_k_Dmax+1)
TFC_Dmax=1/TSFC_k_Dmax;    
else
TFC_Dmax=1/0;
end

else
    
if D_max/N_en<(Th_Dmax+1)
TFC_Dmax=1/TFC;
else
TFC_Dmax=1/0;

end

end

end

%Propeller equations (derived)
if propeller==1
    if type==0
    m_f_p=(((D/N_en+rho*A_inlet_e*V_inf^2)/(rho*A_inlet_e*V_inf))^2*.5*rho*A_inlet_e*V_inf-.5*rho*A_inlet_e*V_inf^3)/(E_sp*eta_propeller_e);
    m_f_p_Dmax=(((D_max/N_en+rho*A_inlet_e*V_inf)^2/(rho*A_inlet_e*V_inf))^2*.5*rho*A_inlet_e*V_inf-.5*rho*A_inlet_e*V_inf^3)/(E_sp*eta_propeller_e);
    else
    m_f_p=(((D/N_en+rho*A_inlet*V_inf^2)/(rho*A_inlet*V_inf))^2*.5*rho*A_inlet*V_inf-.5*rho*A_inlet*V_inf^3)/(E_sp*eta_propeller);
    m_f_p_Dmax=(((D_max/N_en+rho*A_inlet*V_inf^2)/(rho*A_inlet*V_inf))^2*.5*rho*A_inlet*V_inf-.5*rho*A_inlet*V_inf^3)/(E_sp*eta_propeller);
    end
    
TFC=m_f_p/(D/N_en);
TFC_Dmax=m_f_p_Dmax/(D_max/N_en);
       
end
    
if M>2
if TPS==1
% --- Conductive heat-transfer flux through TPS ---
q_cond=(Tw/1.8-T_sl)/(t_ins/k_ins+t_wall/k_wall);
% --- Fuel temperature-rise energy balance ---
T_fuel_B=(q_cond*S_cooling+m_f_m*N_en*Cp_f*T_sl)/(m_f_m*N_en*Cp_f);
end
end

%% ---- metric -> Raymer English ----
N2lb = 1/4.44822;  lb2N = 4.44822;
m2ft = 1/0.3048;   m2_to_ft2 = m2ft^2;

W_dg_lb  = W_cc_q(q-1) * N2lb;
W_l_lb   = W_l  * N2lb;
S_w_ft   = S(q) * m2_to_ft2;
S_csw_ft = S_csw* m2_to_ft2;
S_ht_ft  = S_H * m2_to_ft2;
S_vt_ft  = S_V * m2_to_ft2;
S_cs_ft  = S_cs * m2_to_ft2;
L_ft     = L    * m2ft;
D_ft     = D_f  * m2ft;
W_ft     = W_fuse * m2ft;
L_t_ft   = L_t  * m2ft;
F_w_ft   = F_w  * m2ft;
B_h_ft   = b_H  * m2ft;
L_m_in   = L_m  / 0.0254;
L_n_in   = L_n  / 0.0254;
L_m_ft   = L_m  * m2ft;
L_n_ft   = L_n  * m2ft;
D_e_ft   = D_e  * m2ft;
L_d_ft   = L_d  * m2ft;
L_tp_ft  = L_tp * m2ft;
L_ec_ft  = L_a * m2ft;

if type2==2
S_fw_ft  = S_fw * m2_to_ft2;
end

if Afterburner==0
T_lb     = Th_dry * N2lb;
else
T_lb     = Th_wet * N2lb;
end

W_en_lb  = W_eng_e * N2lb;
W_uav_lb = W_uav * N2lb;
V_t=W_f(q-1)/g/rho_f;
V_t_gal  = V_t  * 264.172;
S_f_wet_ft = pi * D_ft * L_ft;
Q_psf    = 0.5*rho*V_inf^2 * 0.0208854;
SFC = 35304.11*TFC;
K_ws = .75*((1+2*lambda)/(1+lambda))*b*tan(sweep)/L;

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
    W_VT_lb   = 0.073*(1+.2*Ht_Hv)^0.225*W_dg_lb^0.376*N_z^0.376*Q_psf^.122 ...
    *S_vt_ft^0.873*(100*tdc/cos(sweep))^-.49;
    W_tail_lb = W_HT_lb+W_VT_lb;
    W_av_lb   = 2.117 * W_uav_lb^0.933; 
    W_ac_lb   = 0.265*W_dg_lb^0.52*Np^0.68*W_av_lb^0.17*M^0.08;
    b_ft      = sqrt(S_w_ft*AR);
    W_fc_lb   = 0.053*L_ft^1.536*b_ft^0.371*(N_z*W_dg_lb*1e-4)^0.80;
    W_fs_lb   = 2.49*V_t_gal^0.726*(1/(1+Vi_Vt))^0.363*N_t^0.242*N_en^0.157;
    W_ein_lb  = 2.575*W_en_lb^0.922*N_en; 
    
    % ===== GA 15.55-15.58 =====
    W_hyd_lb  = 0.001 * W_dg_lb;                                                                            
    W_elec_lb = 12.57 * (W_fs_lb + W_av_lb)^0.51;                         
    W_furn_lb = 0;                                                        
    W_inst_lb = 0;
    W_apu_lb  = 0;
    W_misc_lb = 0; 
    W_nac_lb = 0; 
    W_hg_lb=0;
    W_ec_lb=0;
    W_s_lb=0;
    W_ecool_lb=0;
    W_fw_lb=0;
    
elseif type2 == 0
%     transport 15.25-15.42
    W_wing_lb = 0.0051*(W_dg_lb*N_z)^0.557*S_w_ft^0.649*AR^0.5 ...
        *tdc^(-0.4)*(1+lambda)^0.1*(cos(sweep_q))^(-1.0)*S_csw_ft^0.1;
    W_fus_lb  = 0.3280*K_door*K_Lg*(W_dg_lb*N_z)^0.5*L_ft^0.25 ...
        *S_f_wet_ft^0.302*(1+K_ws)^0.04*(L_ft/D_ft)^0.10;
    Vstall_kts = V_stall_sl*1.94384;
    W_gear_lb = 0.0106*K_mp*W_l_lb^0.888*N_l^0.25*L_m_ft^0.4 ...
        *N_mw^0.321*N_mss^(-0.5)*Vstall_kts^0.1 ...
        + 0.032*K_np*W_l_lb^0.646*N_l^0.2*L_n_ft^0.5*N_nw^0.45;
    W_HT_lb   = 0.0379*K_uht*(1+F_w_ft/B_h_ft)^(-0.25)*W_dg_lb^0.639 ...
        *N_z^0.10*S_ht_ft^0.75*L_t_ft^(-1.0)*(cos(sweep_q))^(-1.0);
    W_VT_lb   = 0.0026*(1+Ht_Hv)^0.225*W_dg_lb^0.556*N_z^0.536 ...
        *L_t_ft^(-0.5)*S_vt_ft^0.5;
 
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_av_lb   = 1.73*W_uav_lb^0.983;
    Vpr_ft3   = (pi*D_ft^2/4)*L_ft;
    W_ac_lb   = 62.36*Np^0.25*(Vpr_ft3/1000)^0.604*W_uav_lb^0.10;
    W_fc_lb   = 145.9*Nf^0.554*(1+Nm/max(Nf,1))^(-1)*S_cs_ft^0.20;
    W_fs_lb   = 2.405*V_t_gal^0.606*(1+Vi_Vt)^(-1)*(1+Vp_Vt)*N_t^0.5;
    W_ein_lb  = N_en*W_en_lb*1.3;
    
    W_s_lb=49.19*(N_en*W_en_lb/1000)^.541;
    
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
    W_ec_lb  = 5*N_en+.8*L_ec_ft;    % per nacelle [lb]
    W_nac_lb = 0.6724 * K_ng * N_Lt_ft^0.10 * N_w_ft^0.294 ...
         * N_z^0.119 * W_ec_lb^0.611 * N_en^0.984 * S_n_ft^0.224;
    W_hg_lb=0;
    W_ecool_lb=0;
    W_fw_lb     = 0;
else
%     fighter/delta
    W_wing_lb = 0.0103*K_dw*K_vs*(W_dg_lb*N_z)^0.5*S_w_ft^0.622*AR^0.785 ...
        *tdc^(-0.4)*(1+lambda)^0.05*(cos(sweep_q))^(-1.0)*S_csw_ft^0.04;
    W_fus_lb  = 0.499*K_dwf*W_dg_lb^0.35*N_z^0.25*L_ft^0.5*D_ft^0.849*W_ft^0.685;
    W_MLG_lb  = K_cb*K_tpg*(W_l_lb*N_l)^0.25*L_m_in^0.973;
    W_NLG_lb  = (W_l_lb*N_l)^0.290*L_n_in^0.5*N_nw^0.525;
    W_gear_lb = W_MLG_lb + W_NLG_lb;
    W_HT_lb   = 3.316*(1+F_w_ft/B_h_ft)^(-2.0)*((W_dg_lb*N_z)/1000)^0.260*S_ht_ft^0.806;
    W_VT_lb   = 0.452*K_rht*(1+Ht_Hv)^0.5*(W_dg_lb*N_z)^0.488*S_vt_ft^0.718 ...
        *M^0.341*L_t_ft^(-1.0)*(1+Sr_Svt)^0.348*AR_V^0.223 ...
        *(1+lambda)^0.25*(cos(sweep_V))^(-0.323);
    W_tail_lb = W_HT_lb + W_VT_lb;
    
    if Ls_Ld==0
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*D_e_ft;
    else
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*Ls_Ld^(-0.373)*D_e_ft;
    end
    
    W_mnt_lb  = 0.013*N_en^0.795*T_lb^0.579*N_z;
    W_esc_lb  = 0.01*W_en_lb^0.717*N_en*N_z;
    W_tp_lb   = 3.5*D_e_ft*L_tp_ft*N_en;
    W_ein_lb  = N_en*W_en_lb + W_inl_lb + W_mnt_lb + W_esc_lb + W_tp_lb;
    W_fs_lb   = 7.45 * (W_f(q-1)*.225)^0.47 * (1 + Vi_Vt)^(-0.095) * (1 + Vp_Vt)^(-0.095) * N_t^0.066 * N_en^0.052 * (D/N_en * SFC / 1000)^0.249;
    W_fc_lb   = 36.28*M^0.003*S_cs_ft^0.489*N_s^0.484*N_c^0.127;
      W_s_lb    = 0;
 

if type ==0
    T_e_lb    = T_lb;                                      
else
    T_e_lb    = (T_sl*.225);                                       
end
    
    W_ecool_lb   = 4.55*D_e_ft*L_d_ft*N_en;
    W_oil_lb  = 37.82 * N_en^1.023;                                       
    W_ec_lb= 10.5 * N_en^1.008 * L_ec_ft^0.222;                        
    W_strt_lb = 0.025 * T_e_lb^0.760 * N_en^0.72;                         
    W_inst_lb = 8.0 + 36.37*N_en^0.676*N_t^0.237 + 26.4*(1+N_ci)^1.356;   
    W_hyd_lb  = 37.23 * K_vsh * N_u^0.664;   
    L_a_ft    = L/.3048;
    N_gen     = N_en;
    W_elec_lb = 172.2 * K_mc * R_kva^0.152 * N_c^0.10 * L_a_ft^0.10 ...
                * N_gen^0.091;                                            
    W_av_lb   = 2.117 * W_uav_lb^0.933;                                   
    W_furn_lb = 217.6 * N_c;                                              
    W_ac_lb   = 201.6 * ((W_uav_lb + 200*N_c)/1000)^0.735;                
    W_misc_lb = 3.2e-4 * W_dg_lb;                                         
    W_apu_lb  = 0;
    W_ein_lb  = W_ein_lb + W_oil_lb + W_ec_lb + W_strt_lb;
    W_nac_lb = 0;
    W_hg_lb=0;
    W_fw_lb     = 1.13*S_fw_ft;
    
end

%% ---- fudge factors (keep Raymer exponents; scale constants) ----
switch type2
    case 0
        if M<1
        K_wing = 1.6; K_fuse = 1.6; K_gear = 1;
        K_tail = 1.6; K_prop = 1.6; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
       
        % Concorde / Tu-144  (type2==0 transport eqs)
        K_wing = 1.3;   % slender delta; K_dw=0.768 already in 15.1
        K_fuse = 1.3;   % long intake tunnels + ogive, not a tube airliner
        K_gear = 1;   % 8-wheel main trucks, high W_l
        K_tail = 1.3;   % S_ht ~ 0; do not inflate a missing tail
        K_prop = 1.3;   % four Olympus + 2-D ramps / reversers
        K_fuel = 1;   % 15.16 under-runs ~96 t of integral fuel
        K_sys  = 1;   % hyd + elec + intake control
        K_furn = 1;   % 100 pax cabin; not 737 K_furn=2.5
        K_oew  = 1;   % leftover ops / unusable / heat structure
        end
    case 1
        if M<1
        % light jet (Citation CJ3 class): pressurized, T-tail, retractable
    % only if you are running the CJ3 — comment the piston block out
    K_wing = 1.5;   % laminar wing + fuel in wing
    K_fuse = 1.5;   % 15.49 without the pressurization add-on under-runs
    K_gear = 1;   % retractable
    K_tail = 1.5;   % T-tail (H_t_Hv=1 already helps if coded)
    K_prop = 1.5;   % use book engine weight
    K_fuel = 1;   % 15.54 is a light-airplane fuel system
    K_sys  = 1;   % hyd 15.55 is ~14 lb on a CJ3; real system is larger
    K_furn = 1;   % interior / pressurization leftover
    K_oew  = 1;
        else
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        end
    otherwise
        if M<2.5
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
            
        if skin==1
            K_mat=1;
        elseif skin==2
            K_mat=1.7;
        else
            K_mat=1.8;
        end
        K_wing = K_mat;   % honeycomb + high-temp skins; 15.1 under-runs
        K_fuse = K_mat;   % long inlet tunnels + heat structure
        K_gear = 1;   % high W_l, bicycle/truck gear not in 15.4–15.5
        K_tail = K_mat;   % canards / twin fins if S_ht, S_vt are small
        K_prop = K_mat;   % 15.10–15.15 inlet+install light vs 6-engine pack
        K_fuel = 1;   % 15.16 vs ~300000 lb fuel system / tanks
        K_sys  = 1;   % hyd + elec + ECS at Mach 3
        K_furn = 1;   % no airliner cabin; leave 15.22 as-is
        K_oew  = 1;   % leftover ops / unusable / TPS not in Ch. 15
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
W_hg       = W_hg_lb * lb2N;
W_ec       = W_ec_lb * lb2N;
W_s        = W_s_lb * lb2N;
W_ecool    = W_ecool_lb * lb2N;
W_fw       = W_fw_lb * lb2N;

if aircraft_case >9 || aircraft_case<9
   W_chine=0;
end



W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_chine  + W_ec + W_s + W_ecool + W_fw;

if M>2
if TPS==1
    
W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_tps+W_chine + W_ec + W_s + W_ecool + W_fw;

end
end

    
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
    alpha_j=alpha;
    C_d_0_j=c_d_0_c;
    sweep_xc_j=sweep_xc;
    Re_w_j=Re_w;
    alpha_3D_j=alpha*L_s_3d/L_s_2d;
    C_L_j=C_L;
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
%     C_d_0_j=C_d_0;
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
    alpha_k=alpha_j;
%     C_d_0_kk=C_d_0_j;
    sweep_xc_k=sweep_xc_j;
    Re_w_k=Re_w_j;
    alpha_3D_k=alpha_3D_j;
    C_L_k = C_L_j;
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
T_0_5_e=  (TIT_e - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e)*eta_m_e));
T_0_5_e_max=(TIT_e - (W_c_e + (BR_e)*W_f_e)/(Cp*(1+f_e_max)*eta_m_e));
% --- actual turbine exit temperature ---
T_0_5s_e = TIT_e - (TIT_e-T_0_5_e)/eta_t_e; %https://www.sciencedirect.com/topics/materials-science/turbine-efficiency 
T_0_5s_e_max = TIT_e - (TIT_e-T_0_5_e_max)/eta_t_e;
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

if Th_e_0<D/N_en
    TIT_e_0=TIT_e_0+(TIT_e_high-TIT_e_low)/2^ii;
else
    TIT_e_0=TIT_e_0-(TIT_e_high-TIT_e_low)/2^ii;  
end

if Th_e_0<D_max/N_en
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
    V_e_core_ab_k_e=(D/N_en+m_core_e*V_inf- m_o_f_e*(V_e_f_e - V_inf))/(m_core_e*(1+f_e+f_6_k_e_i));
else    
    V_e_core_ab_k_e=(D/N_en+m_core_e*V_inf)/(m_core_e*(1+f_e+f_6_k_e_i));
end

if BR_e>0
    V_e_core_ab_k_e_Dmax=(D_max/N_en+m_core_e*V_inf- m_o_f_e*(V_e_f_e - V_inf))/(m_core_e*(1+f_e+f_6_k_e_i_Dmax));
else    
    V_e_core_ab_k_e_Dmax=(D_max/N_en+m_core_e*V_inf)/(m_core_e*(1+f_e+f_6_k_e_i_Dmax));
end

% --- Existing-engine afterburner total temperature ---
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

if BR_e>0
    T_k_e_max = (m_core_e + m_f_ab_k_e_max) * (V_e_core_ab_k_e_max)- m_core_e*V_inf + m_o_f_e*(V_e_f_e - V_inf);
else    
    T_k_e_max = (m_core_e + m_f_ab_k_e_max) * (V_e_core_ab_k_e_max)-m_core_e*V_inf;
end
      

if Afterburner==1
  
TFC=(m_f_ab_k_e)/(D/N_en);

else
    
if D/N_en<(Th_e_0+1)
    
TFC=(m_f_e)/(D/N_en);

else
    
TFC=1/0;

end

end

if type==0 && Afterburner==1  
    
TFC=(m_f_ab_k_e)/(D/N_en);

elseif type==0 && Afterburner==0

if D/N_en<(Th_e+1)
    
TFC=(m_f_e)/(D/N_en);

else

TFC=1/0;

end

else
    
if D/N_en<(Th_e+1)
 
TFC=(m_f)/(D/N_en);

else
 
TFC=1/0;

end
        
end


if propeller==1
    if type==0
    m_f_p=(((D/N_en+rho*A_inlet_e*V_inf^2)/(rho*A_inlet_e*V_inf))^2*.5*rho*A_inlet_e*V_inf-.5*rho*A_inlet_e*V_inf^3)/(E_sp*eta_propeller_e);
    m_f_p_Dmax=(((D_max/N_en+rho*A_inlet_e*V_inf)^2/(rho*A_inlet_e*V_inf))^2*.5*rho*A_inlet_e*V_inf-.5*rho*A_inlet_e*V_inf^3)/(E_sp*eta_propeller_e);
    else
    m_f_p=(((D/N_en+rho*A_inlet*V_inf^2)/(rho*A_inlet*V_inf))^2*.5*rho*A_inlet*V_inf-.5*rho*A_inlet*V_inf^3)/(E_sp*eta_propeller);
    m_f_p_Dmax=(((D_max/N_en+rho*A_inlet*V_inf^2)/(rho*A_inlet*V_inf))^2*.5*rho*A_inlet*V_inf-.5*rho*A_inlet*V_inf^3)/(E_sp*eta_propeller);
    end
    
TFC=m_f_p/(D/N_en);
TFC_Dmax=m_f_p_Dmax/(D_max/N_en);
       
end

if M>2
if TPS==1
% --- Conductive heat-transfer flux through TPS ---
q_cond=(Tw/1.8-T_sl)/(t_ins/k_ins+t_wall/k_wall);
% --- Fuel temperature-rise energy balance ---
T_fuel_B=(q_cond*S_cooling+m_f_m*N_en*Cp_f*T_sl)/(m_f_m*N_en*Cp_f);
end
end


%% ---- metric -> Raymer English ----
N2lb = 1/4.44822;  lb2N = 4.44822;
m2ft = 1/0.3048;   m2_to_ft2 = m2ft^2;

W_dg_lb  = W_cc_q(q-1) * N2lb;
W_l_lb   = W_l  * N2lb;
S_w_ft   = S(q) * m2_to_ft2;
S_csw_ft = S_csw* m2_to_ft2;
S_ht_ft  = S_H * m2_to_ft2;
S_vt_ft  = S_V * m2_to_ft2;
S_cs_ft  = S_cs * m2_to_ft2;
L_ft     = L    * m2ft;
D_ft     = D_f  * m2ft;
W_ft     = W_fuse * m2ft;
L_t_ft   = L_t  * m2ft;
F_w_ft   = F_w  * m2ft;
B_h_ft   = b_H  * m2ft;
L_m_in   = L_m  / 0.0254;
L_n_in   = L_n  / 0.0254;
L_m_ft   = L_m  * m2ft;
L_n_ft   = L_n  * m2ft;
D_e_ft   = D_e  * m2ft;
L_d_ft   = L_d  * m2ft;
L_tp_ft  = L_tp * m2ft;
L_ec_ft  = L_a * m2ft;

if type2==2
S_fw_ft  = S_fw * m2_to_ft2;
end

if Afterburner==0
T_lb     = Th_dry * N2lb;
else
T_lb     = Th_wet * N2lb;
end

W_en_lb  = W_eng_e * N2lb;
W_uav_lb = W_uav * N2lb;
V_t=W_f(q-1)/g/rho_f;
V_t_gal  = V_t  * 264.172;
S_f_wet_ft = pi * D_ft * L_ft;
Q_psf    = 0.5*rho*V_inf^2 * 0.0208854;
SFC = 35304.11*TFC;
K_ws = .75*((1+2*lambda)/(1+lambda))*b*tan(sweep)/L;

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
    W_VT_lb   = 0.073*(1+.2*Ht_Hv)^0.225*W_dg_lb^0.376*N_z^0.376*Q_psf^.122 ...
    *S_vt_ft^0.873*(100*tdc/cos(sweep))^-.49;
    W_tail_lb = W_HT_lb+W_VT_lb;
    W_av_lb   = 2.117 * W_uav_lb^0.933; 
    W_ac_lb   = 0.265*W_dg_lb^0.52*Np^0.68*W_av_lb^0.17*M^0.08;
    b_ft      = sqrt(S_w_ft*AR);
    W_fc_lb   = 0.053*L_ft^1.536*b_ft^0.371*(N_z*W_dg_lb*1e-4)^0.80;
    W_fs_lb   = 2.49*V_t_gal^0.726*(1/(1+Vi_Vt))^0.363*N_t^0.242*N_en^0.157;
    W_ein_lb  = 2.575*W_en_lb^0.922*N_en; 
    
    % ===== GA 15.55-15.58  =====
    W_hyd_lb  = 0.001 * W_dg_lb;                                                                            
    W_elec_lb = 12.57 * (W_fs_lb + W_av_lb)^0.51;                         
    W_furn_lb = 0;                                                        
    W_inst_lb = 0;
    W_apu_lb  = 0;
    W_misc_lb = 0; 
    W_nac_lb = 0; 
    W_hg_lb=0;
    W_ec_lb=0;
    W_s_lb=0;
    W_ecool_lb=0;
    W_fw_lb=0;
    
elseif type2 == 0
%     transport 15.25-15.42
    W_wing_lb = 0.0051*(W_dg_lb*N_z)^0.557*S_w_ft^0.649*AR^0.5 ...
        *tdc^(-0.4)*(1+lambda)^0.1*(cos(sweep_q))^(-1.0)*S_csw_ft^0.1;
    W_fus_lb  = 0.3280*K_door*K_Lg*(W_dg_lb*N_z)^0.5*L_ft^0.25 ...
        *S_f_wet_ft^0.302*(1+K_ws)^0.04*(L_ft/D_ft)^0.10;
    Vstall_kts = V_stall_sl*1.94384;
    W_gear_lb = 0.0106*K_mp*W_l_lb^0.888*N_l^0.25*L_m_ft^0.4 ...
        *N_mw^0.321*N_mss^(-0.5)*Vstall_kts^0.1 ...
        + 0.032*K_np*W_l_lb^0.646*N_l^0.2*L_n_ft^0.5*N_nw^0.45;
    W_HT_lb   = 0.0379*K_uht*(1+F_w_ft/B_h_ft)^(-0.25)*W_dg_lb^0.639 ...
        *N_z^0.10*S_ht_ft^0.75*L_t_ft^(-1.0)*(cos(sweep_q))^(-1.0);
    W_VT_lb   = 0.0026*(1+Ht_Hv)^0.225*W_dg_lb^0.556*N_z^0.536 ...
        *L_t_ft^(-0.5)*S_vt_ft^0.5;
 
    W_tail_lb = W_HT_lb + W_VT_lb;
    W_av_lb   = 1.73*W_uav_lb^0.983;
    Vpr_ft3   = (pi*D_ft^2/4)*L_ft;
    W_ac_lb   = 62.36*Np^0.25*(Vpr_ft3/1000)^0.604*W_uav_lb^0.10;
    W_fc_lb   = 145.9*Nf^0.554*(1+Nm/max(Nf,1))^(-1)*S_cs_ft^0.20;
    W_fs_lb   = 2.405*V_t_gal^0.606*(1+Vi_Vt)^(-1)*(1+Vp_Vt)*N_t^0.5;
    W_ein_lb  = N_en*W_en_lb*1.3;
    
    W_s_lb=49.19*(N_en*W_en_lb/1000)^.541;
    
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
    W_ec_lb  = 5*N_en+.8*L_ec_ft;    % per nacelle [lb]
    W_nac_lb = 0.6724 * K_ng * N_Lt_ft^0.10 * N_w_ft^0.294 ...
         * N_z^0.119 * W_ec_lb^0.611 * N_en^0.984 * S_n_ft^0.224;
    W_hg_lb=0;
    W_ecool_lb=0;
    W_fw_lb     = 0;
else
%     fighter/delta
    W_wing_lb = 0.0103*K_dw*K_vs*(W_dg_lb*N_z)^0.5*S_w_ft^0.622*AR^0.785 ...
        *tdc^(-0.4)*(1+lambda)^0.05*(cos(sweep_q))^(-1.0)*S_csw_ft^0.04;
    W_fus_lb  = 0.499*K_dwf*W_dg_lb^0.35*N_z^0.25*L_ft^0.5*D_ft^0.849*W_ft^0.685;
    W_MLG_lb  = K_cb*K_tpg*(W_l_lb*N_l)^0.25*L_m_in^0.973;
    W_NLG_lb  = (W_l_lb*N_l)^0.290*L_n_in^0.5*N_nw^0.525;
    W_gear_lb = W_MLG_lb + W_NLG_lb;
    W_HT_lb   = 3.316*(1+F_w_ft/B_h_ft)^(-2.0)*((W_dg_lb*N_z)/1000)^0.260*S_ht_ft^0.806;
    W_VT_lb   = 0.452*K_rht*(1+Ht_Hv)^0.5*(W_dg_lb*N_z)^0.488*S_vt_ft^0.718 ...
        *M^0.341*L_t_ft^(-1.0)*(1+Sr_Svt)^0.348*AR_V^0.223 ...
        *(1+lambda)^0.25*(cos(sweep_V))^(-0.323);
    W_tail_lb = W_HT_lb + W_VT_lb;
    
    if Ls_Ld==0
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*D_e_ft;
    else
    W_inl_lb  = 13.29*K_vg*L_d_ft^0.643*K_d^0.182*N_en^1.498*Ls_Ld^(-0.373)*D_e_ft;
    end
    
    W_mnt_lb  = 0.013*N_en^0.795*T_lb^0.579*N_z;
    W_esc_lb  = 0.01*W_en_lb^0.717*N_en*N_z;
    W_tp_lb   = 3.5*D_e_ft*L_tp_ft*N_en;
    W_ein_lb  = N_en*W_en_lb + W_inl_lb + W_mnt_lb + W_esc_lb + W_tp_lb;
    W_fs_lb   = 7.45 * (W_f(q-1)*.225)^0.47 * (1 + Vi_Vt)^(-0.095) * (1 + Vp_Vt)^(-0.095) * N_t^0.066 * N_en^0.052 * (D/N_en * SFC / 1000)^0.249;
    W_fc_lb   = 36.28*M^0.003*S_cs_ft^0.489*N_s^0.484*N_c^0.127;
      W_s_lb    = 0;
 

if type ==0
    T_e_lb    = T_lb;                                      
else
    T_e_lb    = (T_sl*.225);                                       
end
    
    W_ecool_lb   = 4.55*D_e_ft*L_d_ft*N_en;
    W_oil_lb  = 37.82 * N_en^1.023;                                       
    W_ec_lb= 10.5 * N_en^1.008 * L_ec_ft^0.222;                        
    W_strt_lb = 0.025 * T_e_lb^0.760 * N_en^0.72;                         
    W_inst_lb = 8.0 + 36.37*N_en^0.676*N_t^0.237 + 26.4*(1+N_ci)^1.356;   
    W_hyd_lb  = 37.23 * K_vsh * N_u^0.664;   
    L_a_ft    = L/.3048;
    N_gen     = N_en;
    W_elec_lb = 172.2 * K_mc * R_kva^0.152 * N_c^0.10 * L_a_ft^0.10 ...
                * N_gen^0.091;                                            
    W_av_lb   = 2.117 * W_uav_lb^0.933;                                   
    W_furn_lb = 217.6 * N_c;                                              
    W_ac_lb   = 201.6 * ((W_uav_lb + 200*N_c)/1000)^0.735;                
    W_misc_lb = 3.2e-4 * W_dg_lb;                                         
    W_apu_lb  = 0;
    W_ein_lb  = W_ein_lb + W_oil_lb + W_ec_lb + W_strt_lb;
    W_nac_lb = 0;
    W_hg_lb=0;
    W_fw_lb     = 1.13*S_fw_ft;
    
end

%% ---- fudge factors (keep Raymer exponents; scale constants) ----
switch type2
    case 0
        if M<1
        K_wing = 1.6; K_fuse = 1.6; K_gear = 1;
        K_tail = 1.6; K_prop = 1.6; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
       
        % Concorde / Tu-144  (type2==0 transport eqs)
        K_wing = 1.3;   % slender delta; K_dw=0.768 already in 15.1
        K_fuse = 1.3;   % long intake tunnels + ogive, not a tube airliner
        K_gear = 1;   % 8-wheel main trucks, high W_l
        K_tail = 1.3;   % S_ht ~ 0; do not inflate a missing tail
        K_prop = 1.3;   % four Olympus + 2-D ramps / reversers
        K_fuel = 1;   % 15.16 under-runs ~96 t of integral fuel
        K_sys  = 1;   % hyd + elec + intake control
        K_furn = 1;   % 100 pax cabin; not 737 K_furn=2.5
        K_oew  = 1;   % leftover ops / unusable / heat structure
        end
    case 1
        if M<1
        % light jet (Citation CJ3 class): pressurized, T-tail, retractable
    % only if you are running the CJ3 — comment the piston block out
    K_wing = 1.5;   % laminar wing + fuel in wing
    K_fuse = 1.5;   % 15.49 without the pressurization add-on under-runs
    K_gear = 1;   % retractable
    K_tail = 1.5;   % T-tail (H_t_Hv=1 already helps if coded)
    K_prop = 1.5;   % use book engine weight
    K_fuel = 1;   % 15.54 is a light-airplane fuel system
    K_sys  = 1;   % hyd 15.55 is ~14 lb on a CJ3; real system is larger
    K_furn = 1;   % interior / pressurization leftover
    K_oew  = 1;
        else
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        end
    otherwise
        if M<2.5
        K_wing = 1; K_fuse = 1; K_gear = 1;
        K_tail = 1; K_prop = 1; K_fuel = 1;
        K_sys  = 1; K_furn = 1; K_oew  = 1;
        else
            
        if skin==1
            K_mat=1;
        elseif skin==2
            K_mat=1.7;
        else
            K_mat=1.8;
        end
        K_wing = K_mat;   % honeycomb + high-temp skins; 15.1 under-runs
        K_fuse = K_mat;   % long inlet tunnels + heat structure
        K_gear = 1;   % high W_l, bicycle/truck gear not in 15.4–15.5
        K_tail = K_mat;   % canards / twin fins if S_ht, S_vt are small
        K_prop = K_mat;   % 15.10–15.15 inlet+install light vs 6-engine pack
        K_fuel = 1;   % 15.16 vs ~300000 lb fuel system / tanks
        K_sys  = 1;   % hyd + elec + ECS at Mach 3
        K_furn = 1;   % no airliner cabin; leave 15.22 as-is
        K_oew  = 1;   % leftover ops / unusable / TPS not in Ch. 15
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
W_hg       = W_hg_lb * lb2N;
W_ec       = W_ec_lb * lb2N;
W_s        = W_s_lb * lb2N;
W_ecool    = W_ecool_lb * lb2N;
W_fw       = W_fw_lb * lb2N;

if aircraft_case >9 || aircraft_case<9
   W_chine=0;
end



W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_chine  + W_ec + W_s + W_ecool + W_fw;

if M>2
if TPS==1
    
W_e(q) = W_wing + W_fus + W_gear + W_tail + W_eng_inst + W_fuel_sys ...
       + W_fc + W_av + W_ac + W_hyd + W_elec + W_inst + W_furn + W_apu + W_misc + W_hg + W_tps+W_chine + W_ec + W_s + W_ecool + W_fw;

end
end

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
    Th_dd(dd)=D_dd+W_dd(dd-1)*sin(theta_dd_B)+W_dd(dd-1)/g*acc_dd;
% --- Horizontal velocity ---
    Vx_dd=V_dd(dd)*cos(theta_dd_B);
% --- Horizontal descent distance increment ---
    dx_dd=dtime_dd*Vx_dd;
% --- Horizontal descent total distance ---
    x_dd(dd)=dx_dd+x_dd(dd-1);
% --- Thrust specific fuel consumption
    TSFC_dd(dd)=(T_dd/T)^.5*TFC;
% --- Overall efficiency ---
    no_dd(dd)=V_dd(dd)/(TSFC_dd(dd)*E_sp);
% --- Descent fuel weight increment ---
    dW_dd(dd)=(Th_dd(dd)*V_dd(dd)/no_dd(dd))/E_sp*g*dtime_dd;
% --- Total descent fuel weight ---
    W_dd(dd)=W_dd(dd-1)+dW_dd(dd);     
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

end

TOGW_alt(zzz)=W_cc_q(q);

if (TOGW_alt(zzz)<zz && TOGW_alt(zzz)>0) || TOGW_alt(zzz-1)==0
    zz=TOGW_alt(zzz);
    TOGW_p=W_cc_q;
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
%         C_d_0_1=C_d_0;
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
        sweep_xc_i=sweep_xc;
        C_d_0_i=c_d_0_c;
        Re_w_i=Re_w;
        alpha_i=alpha;
        alpha_i_3D=alpha*L_s_3d/L_s_2d;
        C_L_i=C_L;
        if M<1
        Th_i=Th_e_0;
        Th_i_max=D_max/N_en;
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
%         C_d_0_1=C_d_0_k;
        rho_1=rho_k;
        W_l_1=W_l_k;
        V_inf_1=V_inf_k;
        W_cc_1=W_cc_i_min;
        K_1=K_k;
        TOGW_11=TOGW_i_k;
        TOGW_111=TOGW_i_kk;
        Th_sl_i=Th_sl_k;
        Th_max_1=T_max_k;
        Th_i_Dmax=D_max_i/N_en;
        c_l_i=c_l_k;
        c_r_i=c_r_k;
        c_t_i=c_t_k;
        b_i=b_k;
        sweep_i=sweep_k;
        C_d_0_i=C_d_0_k;
        sweep_xc_i=sweep_xc_k;
        Re_w_i=Re_w_k;
        alpha_i=alpha_k;
        alpha_i_3D=alpha_3D_k;
        C_L_i=C_L_k;
    end
    alt_i=alt;
    TOGW_1=TOGW_alt(zzz);
    Drag=D;
    LdD_i=LdD(q);
    W_e_1=W_e(q);
    TFC_1=TFC;
    m_f_m_i=m_f_m;
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
%         C_d_0_1=C_d_0;
        W_l_1=W_l;
        W_cc_1=W_cc_q(q);
        K_1=1/(pi*AR*e);
        TOGW_11=TOGW_i(q);
        TOGW_111=W_dd_q(q);
        Th_ee=Th_e_max;
        Th_ab=T_k_e_max;
        Drag=D;
        Th_i_Dmax=D_max/N_en;
        c_l_i=C_l;
        c_r_i=c_r;
        c_t_i=c_t;
        b_i=b;
        sweep_i=sweep;
        C_d_0_i=c_d_0_c;
        sweep_xc_i=sweep_xc;
        Re_w_i=Re_w;
        alpha_i=alpha;
        alpha_i_3D=alpha*L_s_3d/L_s_2d;
        C_L_i=C_L;
        if M<1
        Th_i=Drag/N_en;
        Th_i_max=D_max/N_en;
        else
        Th_i=Drag/N_en;
        Th_i_max=D_max/N_en;
        end
    else
        LdD_1=LdD_k;
        Drag=D;
        TIT_i=TIT_ii;
        CPR_i=CPR_ii;
        BR_i=BR_ii;
        Th_i=Drag/N_en;
        FPR_i=FPR_ii;
        S_1=S_k;
%         C_d_0_1=C_d_0_k;
        rho_1=rho_k;
        W_l_1=W_l_k;
        V_inf_1=V_inf_k;
        W_cc_1=W_cc_i_min;
        K_1=K_k;
        TOGW_11=TOGW_i_k;
        TOGW_111=TOGW_i_kk;
        Th_sl_i=Th_sl_k;
        Th_max_1=T_max_k;
        Th_i_max=D_max/N_en;
        Th_i_Dmax=D_max/N_en;
        c_l_i=c_l_k;
        c_r_i=c_r_k;
        c_t_i=c_t_k;
        b_i=b_k;
        sweep_i=sweep_k;
        C_d_0_i=C_d_0_k;
        sweep_xc_i=sweep_xc_k;
        Re_w_i=Re_w_k;
        alpha_i=alpha_k;
        alpha_i_3D=alpha_3D_k;
        C_L_i=C_L_k;
    end
    alt_i=alt;
    LdD_i=LdD(q);
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

% if aircraft_case<11
if M>1
fprintf('TOGW=%f\nW_e=%f\nRange=%f\nLift to drag ratio=%f\nDrag_per_engine=%f\nS=%f\nAlt=%f\nV=%f\nV_a=%f\nc_r=%f\nc_t=%f\nb=%f\nSweep=%f\nC_l=%f\nC_l_incompressible=%f\nC_L=%f\nalpha=%f\nC_d_0 (use in airfoil opt. code) = %f\nMax thickness sweep (sweep_xc)=%f\nRe_wing (semispan)=%f\nalpha_3D=%f\n\n',var_min,W_e_1,Range_cruise+x_dd_i+x_cc_i,LdD_i,Drag/N_en,S_1,alt_i,V_i,V_a,c_r_i, c_t_i,b_i, sweep_i*180/pi, c_l_i,c_l_i*(1-M^2)^.5,C_L_i,alpha, C_d_0_i, sweep_xc_i, Re_w_i, alpha_i_3D)   
else
fprintf('TOGW=%f\nW_e=%f\nRange=%f\nLift to drag ratio=%f\nDrag_per_engine=%f\nS=%f\nAlt=%f\nV=%f\nV_a=%f\nc_r=%f\nc_t=%f\nb=%f\nSweep=%f\nC_l=%f\nC_l_incompressible=%f\nC_L=%f\nalpha=%f\nC_d_0 (use in airfoil opt. code) = %f\nMax thickness sweep (sweep_xc)=%f\nRe_wing (semispan)=%f\n\n',var_min,W_e_1,Range_cruise+x_dd_i+x_cc_i,LdD_i,Drag/N_en,S_1,alt_i,V_i,V_a,c_r_i, c_t_i,b_i, sweep_i*180/pi, c_l_i,c_l_i*(1-M^2)^.5,C_L_i,alpha, C_d_0_i, sweep_xc_i, Re_w_i)   
end
% else
% fprintf('TOGW=%f\nW_e=%f\nRange=%f\nLift to drag ratio=%f\nDrag_per_engine=%f\nS=%f\nAlt=%f\nV=%f\nV_a=%f\nc_r=%f\nc_t=%f\nb=%f\nSweep=%f\nC_l=%f\nC_l_incompressible=%f\nC_d_0 (use in airfoil opt. code) = %f\nMax thickness sweep (sweep_xc)=%f\nRe_wing_root=%f\n\n',var_min,W_e_1,Range_cruise+x_dd_i+x_cc_i,LdD_i,Drag/N_en,S_1,alt_i,V_i,V_a,c_r_i, c_t_i,b_i, sweep_i*180/pi, c_l_i,c_l_i*(1-M^2)^.5, C_d_0_i, sweep_xc_i, Re_w_i)
% end

if M>1
if Afterburner==1 && type==0
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet_e,Drag/N_en,TIT_i,T_0_6_i,TFC_1,T_0_6_k_e_Dmax,M_6_e,M_6_Dmax)
elseif Afterburner==1 && type==1
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Drag/N_en,TIT_i,T_0_6_i,TFC_1,T_0_6_k_Dmax_min_j,M_6_k,M_6_k_Dmax)
elseif Afterburner==0 && type==0   
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet_e,Drag/N_en,TIT_i,TFC_1,Th_i,Drag/N_en)
else
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_required=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Drag/N_en,TIT_i,TFC_1,Th_i,Drag/N_en)   
end

else
if Afterburner==1 && type==0
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nThrust_afterburner_per_engine=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,D/N_en,TIT_i,T_0_6_i,TFC_1,T_0_6_k_e_Dmax,M_6_e,M_6_Dmax)
elseif Afterburner==1 && type==1
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nThrust_afterburner_per_engine=%f\nTIT=%f\nT_0_6=%f\nTSFC=%f\nT_0_6_max=%f\nM_6=%f\nM_6_Dmax=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,D_k/N_en,TIT_i,T_0_6_i,TFC_1,T_0_6_k_Dmax_min_j,M_6_k,M_6_k_Dmax)
else   
fprintf('CPR=%f\nBR=%f\nFPR=%f\nA_inlet=%f\nThrust_turbomachinery_per_engine=%f\nTIT=%f\nTSFC=%f\nTh=%f\nD/n=%f\n',CPR_i,BR_i,FPR_i,A_inlet,Th_i,TIT_i,TFC_1,Th_i,Drag/N_en)
end
end

if aircraft_case>0

S_err=(S0-S_1)/S0*100;
b_err=(b0-b_i)/b0*100;
c_r_err=(c_r0-c_r_i)/c_r0*100;
TOGW_err=(W_cc_1-TOGW_i(1))/W_cc_q(1)*100;
We_err=((W_e_1)-W_e(1))/(W_e(1))*100;
Wf_err=(abs(TOGW_i(1)-W_e(1)-W_payload)-abs(W_cc_1-W_e_1-W_payload))/abs(TOGW_i(1)-W_e(1)-W_payload)*100;
alt_err=(Alt0-alt_i)/Alt0*100;
R_err=(R0-(Range_cruise+x_dd_i+x_cc_i))/R0*100;
L_D_err=(L_D-LdD_1)/L_D*100;
TSFC_err=(TSFC0-TFC_1)/TSFC0*100;

if sweep_solver==1
sweep_err=(sweep_i-sweep0)/sweep0*100;
fprintf('\n\nS_error=%f\nb_error=%f\nc_r_error=%f\nTOGW_error=%f\nWe_error=%f\nWf_error=%f\nalt_error=%f\nR_error=%f\nL/D_error=%f\nTSFC_error=%f\nSweep_error=%f\n',S_err,b_err,c_r_err,TOGW_err,We_err,Wf_err,alt_err,R_err,L_D_err,TSFC_err,sweep_err)
else
fprintf('\n\nS_error=%f\nb_error=%f\nc_r_error=%f\nTOGW_error=%f\nWe_error=%f\nWf_error=%f\nalt_error=%f\nR_error=%f\nL/D_error=%f\nTSFC_error=%f\n',S_err,b_err,c_r_err,TOGW_err,We_err,Wf_err,alt_err,R_err,L_D_err,TSFC_err)
end

end

figure(1)
plot(alt_zzz(2:end),TOGW_alt(2:end),'.r')
xlabel ('Altitude [m]')
ylabel ('TOGW [N]')
title ('Design Point')

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
    Th_climb=Drag/N_en+W_cc_1*rate_of_climb/V_lof/N_en;
    alpha_0_climb=Th_climb/Th_wet;
    elseif type==0 && Afterburner==0
    Th_0=Th_dry*(rho_0/rho_sl)^nn;
    alpha_0=Th_0/Th_dry;
    Th_climb=Drag/N_en+W_cc_1*rate_of_climb/V_lof/N_en;
    alpha_0_climb=Th_climb/Th_dry;
    else
    Th_0=Th_sl_i*(rho_0/rho_sl)^nn;
    alpha_0=Th_0/Th_sl_i;
    Th_climb=Drag/N_en+W_cc_1*rate_of_climb/V_lof/N_en;
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
    TdW_to(zzz)=((V_lof^2+2*(Takeoff_field_length-V_lof*ttt)*mu_2*g+2*(Takeoff_field_length-V_lof*ttt)*C_d_0_to*.5*rho_0*V_lof^2/W_cc_1)*W_cc_1/(2*(Takeoff_field_length-V_lof*ttt)*g))/W_cc_1*WdS(zzz)/(W_cc_1/S_1);
    WdS_land=rho_0*g*Landing_field_length*W_l_1/(.5*rho_0*V_s^2*S_1)/(1.69*(beta_land));
    
    if type==0 && Afterburner==1
    TdW_cruise_0(zzz)=Th_i_Dmax*N_en/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_1(zzz)=Th_wet_cruise*N_en/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    
    elseif type==0 && Afterburner==0
    TdW_cruise_0(zzz)=Th_i_Dmax*N_en/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_1(zzz)=Th_dry_cruise*N_en/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    
    else
    TdW_cruise_0(zzz)=Th_i_Dmax*N_en/TOGW_11*TOGW_11/S_1/(WdS(zzz));
    TdW_cruise_1(zzz)=Th_max_1*N_en/TOGW_111*TOGW_111/S_1/(WdS(zzz));
    end
    
    if type==0 && Afterburner==1
    TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/N_en*rate_of_climb/V_lof))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_climb_1(zzz)=(Th_i_Dmax-(TOGW_11/N_en*rate_of_climb/V_inf_1))*N_en/TOGW_11*TOGW_11/S_1/(WdS(zzz));

    elseif type==0 && Afterburner==0
    TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/N_en*rate_of_climb/V_lof))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_climb_1(zzz)=(Th_i_Dmax-(TOGW_11/N_en*rate_of_climb/V_inf_1))*N_en/TOGW_11*TOGW_11/S_1/(WdS(zzz));

    else
   TdW_climb_0(zzz)=(Th_i_Dmax+(W_cc_1/N_en*rate_of_climb/V_lof))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
   TdW_climb_1(zzz)=(Th_i_Dmax+(TOGW_11/N_en*rate_of_climb/V_inf_1))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));

    end
        
    if type==0 && Afterburner==1
    TdW_turn_0(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    elseif type==0 && Afterburner==0
    TdW_turn_0(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    else
    TdW_turn_0(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    TdW_turn_1(zzz)=(Th_i_Dmax)*N_en/(W_cc_1*N_z)*W_cc_1/S_1/(WdS(zzz));
    end
     
    if type==0 && Afterburner==1
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    elseif type==0 && Afterburner==0
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    else
    TdW_a_0(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
    TdW_a_1(zzz)=(Th_i_Dmax-(W_cc_1/N_en/g*acc))*N_en/W_cc_1*W_cc_1/S_1/(WdS(zzz));
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
hold on
plot(WdS,TdW_comb)
if Afterburner==0 && type==0
plot((TOGW_11/S_1),(Th_dry_cruise*N_en/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_dry*N_en/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
elseif Afterburner==1 && type==0
plot((TOGW_11/S_1),(Th_wet_cruise*N_en/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_wet*N_en/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
else
plot((TOGW_11/S_1),(Th_max_1*N_en/TOGW_11),'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5);
hold on
plot((W_cc_1/S_1),(Th_sl_i*N_en/W_cc_1),'o', 'Color', 'b', 'MarkerFaceColor', 'b', 'MarkerEdgeColor', 'b', 'MarkerSize', 5);
end
legend('stall','land','takeoff','cruise','climb_1','turn','combined','end of climb','start of climb','location','N')
xlim([0 WdS_max])
% ylim([0 2])
xlabel('Wing loading (W/S) [N/m^2]')
ylabel('Thrust to weight ratio (T/W) [-]')
title('Constraint Analysis')

figure(3)
plot(qqq,TOGW_p,'o','Color', 'r', 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'r', 'MarkerSize', 5)
xlabel('Iteration number')
ylabel('Takeoff gross weight [N]')
title('Convergence Check')












