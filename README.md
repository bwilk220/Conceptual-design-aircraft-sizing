Overview

UPDATE: errors below 10% at book TOGW values for 64/70 output variables. 

In the pdf of the file named “verification of scripts - percent errors_B.pdf”, the percent errors for several outputs of the two aircraft sizing scripts are in the first part of the pdf. Output figures of the aircraft sizing and airfoil optimization scripts are also in the pdf using the Concorde as an example. This file also explains the methods used in the aircraft sizing and airfoil optimization scripts in order. Next, the sources are listed where formulas were obtained.

In the script with the file named “Aircraft_sizing_AT_q_2.m”, verification was performed using the known takeoff gross weight and cruise altitude for the aircrafts listed below. In the file named “Aircraft_sizing_AT.m” verification was performed for the aircrafts listed below by allowing the takeoff gross weight to iterate and converge to some value. The errors are bigger when takeoff gross weight is allowed to iterate and converge to a value rather than being the book value for each aircraft because of the snowball effect: errors add with increased number of iterations of the takeoff gross weight. 

Both of these aircraft sizing scripts (“Aircraft_sizing_AT_q_2.m” and “Aircraft_sizing_AT.m”) have all inputs of the aircrafts listed below already built into the script, so the script is lengthy but saves effort. 

The aircrafts below were used for the verification of the methods of the two aircraft sizing scripts. 

•	aircraft_case = 1   Concorde

•	aircraft_case = 2   Tu-144D

•	aircraft_case = 3   B737-200

•	aircraft_case = 4   B777-200ER

•	aircraft_case = 5   A320-200

•	aircraft_case = 6   CJ-3

•	aircraft_case = 7   2000EX

•	aircraft_case = 8  XB-70 Valkyrie

•	aircraft_case = 9  SR-71 Blackbird

•	aircraft_case = 10  Lockheed L2000

•	aircraft_case = 11  Boeing 707-120

•	aircraft_case = 12  Lockheed C5 Galaxy (C5A)




Outputs

The output figures for the file named “Aircraft_sizing_AT_q_2.m” can be ignored. Only the outputs in the command window are useful in this file at the book takeoff gross weight and book design altitude. In the file named “Aircraft_sizing_AT.m”, the first plot is a TOGW vs design altitude plot, the second plot is a carpet plot, and the third plot is a takeoff gross weight convergence check. The outputs of the command window are given: 

ALL OUTPUT UNITS METRIC:

TOGW = takeoff gross weight, 

W_e = empty weight, 

range = total range with climb, cruise, and descent combined 

S = wing area 

alt = design cruise altitude 

V = aircraft velocity

V_a = indicated airspeed 

c_r = root chord length

c_t = tip chord length 

b = wingspan

sweep = wing leading edge sweep angle

c_l = sectional lift coefficient

C_L = wing 3D lift coefficient 

alpha = geometric angle of attack

C_d_0 = zero lift angle of attack to use in the airfoil optimization script that leaves out the wing zero lift drag coefficient since it is calculated in that script 

Max thickness sweep = sweep angle at airfoil max thickness location as a fraction of the chord

Re_wing_root = Reynolds number at the wing semispan at cruise

alpha_3D = actual 3D angle of attack for the wing (not geometric angle of attack)

CPR = compressor pressure ratio of engine

BR = bypass ratio of engine

FPR = Fan pressure ratio of engine

A_inlet = engine inlet area

TIT = turbine inlet temperature

T_0_6 = afterburner total temperature (if afterburner used)

TSFC = Thrust specific fuel consumption

T_0_6_max = afterburner total temperature at max drag condition at the beginning of cruise (if afterburner used)







The output figures for the file named “airfoil optimization_N” contains a plot of the best lift to drag ratio airfoil and a sectional lift distribution plot of this best airfoil. The output figures for the file named “airfoil optimization O” contains figures of the drag polar, lift curve slope, quarter chord moment coefficient curve, and an angle of attack vs lift to drag ratio curve. The outputs of the command window are given:

ALL OUTPUTS METRIC TO SOLVE FOR HIGHEST L/D AIRFOIL: 

aoa = best airfoil geometric angle of attack

x_crest_u = x location as a fraction of the chord along the airfoil of maximum thickness of the upper surface of best airfoil

y_crest_u = y location as a fraction of the chord along the airfoil of maximum thickness of the upper surface of best airfoil

x_crest_l = x location as a fraction of the chord along the airfoil of maximum thickness of the lower surface of best airfoil

y_crest_u = y location as a fraction of the chord along the airfoil of maximum thickness of the lower surface of best airfoil

L/D = best airfoil lift to drag ratio (including all drag types of aircraft)




How to run aircraft sizing and airfoil optimization scripts

Next, to run either aircraft sizing script (“Aircraft_sizing_AT_q_2.m” or “Aircraft_sizing_AT.m”) on line 57, set aircraft_case=0 if you want to size an aircraft based on custom inputs. Set aircraft_case=1-12 if you want to use an existing aircraft used for verification of the scripts. All the inputs for each verification aircraft are prefilled. For a custom aircraft, fill out all inputs on lines 64-329.

Next, to run the “airfoil_optimization_N” script, fill out all inputs on lines 20-70. The two aircraft sizing scripts can be used to help fill out these inputs. To run the “airfoil_optimization_O” script, fill out the same inputs and the airfoil max thickness variables on lines 82,85,88,and 91 within the for-loops because the analysis is done only on a single airfoil in this code. The “airfoil_optimization_N” code can be used to obtain these four input variables for “airfoil_optimization_O”.

Note: AI was used as a tool to help obtain many of the formulas but for each equation AI was used, the formula was double checked using textbooks and reputable websites or the formula was derived. These scripts do not use any proprietary confidential information.
