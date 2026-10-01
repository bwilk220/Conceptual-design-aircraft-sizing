Overview

In the pdf of the file named “verification of scripts - percent errors.pdf”, the percent errors for several outputs of the two aircraft sizing scripts are in the first part of the pdf. Output figures of the aircraft sizing and airfoil optimization scripts are also in the pdf using the Concorde as an example. This file also explains the methods used in the aircraft sizing and airfoil optimization scripts in order. Next, the pdf file gives the inputs used in the sizing scripts and in the airfoil optimization scripts using the Concorde as an example. 

In the script with the file named “Aircraft_sizing_S_q_2.m”, verification was performed using the known takeoff gross weight and cruise altitude for the aircrafts listed below. In the file named “Aircraft_sizing_S.m” verification was performed for the aircrafts listed below by allowing the takeoff gross weight to iterate and converge to some value. The errors are bigger when takeoff gross weight is allowed to iterate and converge to a value rather than being the book value for each aircraft because of the snowball effect: errors add with increased number of iterations of the takeoff gross weight. 

Both of these aircraft sizing scripts (“Aircraft_sizing_S_q_2.m” and “Aircraft_sizing_S.m”) have all inputs of the aircrafts listed below already built into the script, so the script is lengthy but saves effort. 

The aircrafts below were used for the verification of the methods of the two aircraft sizing scripts. 

•	aircraft_case = 1   Concorde
•	aircraft_case = 2   B-58 Hustler
•	aircraft_case = 3   Tu-144D
•	aircraft_case = 4   F-16C
•	aircraft_case = 5   B737-200
•	aircraft_case = 6   B777-200ER	•	aircraft_case = 7   A320-200
•	aircraft_case = 8   CJ-3
•	aircraft_case = 9   2000EX
•	aircraft_case = 10  XB-70 Valkyrie
•	aircraft_case = 11  SR-71 Blackbird




Outputs

The output figures for the file named “Aircraft_sizing_S_q_2.m” can be ignored. Only the outputs in the command window are useful in this file at the book takeoff gross weight and book design altitude. In the file named “Aircraft_sizing_S.m”, the first plot is a takeoff gross weight convergence check, the second plot is a carpet plot, and the third plot is a TOGW vs design altitude plot. 

The output figures for the file named “airfoil optimization” contains a plot of the best lift to drag ratio airfoil and a sectional lift distribution plot of this best airfoil. The output figures for the file named “airfoil optimization B” contains figures of the drag polar, lift curve slope, quarter chord moment coefficient curve, and an angle of attack vs lift to drag ratio curve. 






How to run aircraft sizing and airfoil optimization scripts

Next, to run either aircraft sizing script (“Aircraft_sizing_S_q_2.m” or “Aircraft_sizing_S.m”) on line 55, set aircraft_case=0 if you want to size an aircraft based on custom inputs. Set aircraft_case=1-11 if you want to use an existing aircraft used for verification of the scripts. All the inputs for each verification aircraft are prefilled. For a custom aircraft, fill out all inputs on lines 63-509.

Next, to run the “airfoil_optimization” script, fill out all inputs on lines 20-63. The two aircraft sizing scripts can be used to help fill out these inputs. To run the “airfoil_optimization_B” script, fill out the same inputs and the airfoil variables on lines 74,79,83,and 87 within the for-loops because the analysis is done only on a single airfoil in this code. The “airfoil_optimization” code can be used to obtain these four input variables for “airfoil_optimization_B”.
