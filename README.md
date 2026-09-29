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
%  --------------------------------------------------------------
%  AIRCRAFT CASES
%  --------------------------------------------------------------
%    aircraft_case = 0   Custom aircraft
%    aircraft_case = 1   Concorde
%    aircraft_case = 2   B-58 Hustler
%    aircraft_case = 3   Tu-144D
%    aircraft_case = 4   F-16C
%    aircraft_case = 5   B737-200
%    aircraft_case = 6   B777-200ER
%    aircraft_case = 7   A320-200
%    aircraft_case = 8   CJ-3
%    aircraft_case = 9   2000EX
%    aircraft_case = 10  XB-70 Valkyrie
%    aircraft_case = 11  SR-71 Blackbird
 
The script file named “Aircraft_sizing_S.m” is for iterating takeoff gross weight whereas the script file named “Aircraft_sizing_S_q_2.m” is for the non-iterating book takeoff gross weight value. 
The pdf named “verification of scripts - percent errors.pdf” shows the percent errors of each of the aircrafts used for verification of the scripts. 
