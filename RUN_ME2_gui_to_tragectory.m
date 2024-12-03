%% AER 822 –W2021 FINAL MISSION PROFILE
% 1.Take off at a selected initial position of 10 degrees below the horizontal 
% elevation position and stabilize at SLF with demonstrated high speed and steady 
% state accuracy in elevation positioning.
% 
% 2.Maintain the elevation angle and travel in clockwise direction for 120 degrees 
% with an adequate speed and stop travelling with demonstrated tracking accuracy 
% (position and velocity errors) and steady state accuracy in both travel and 
% elevation positioning. 
% 
% 3.Starting with zero travel speed, travel in counterclockwise direction for 
% 100 degrees with highest possible speed and fully stop at the end with an overshoot 
% of no more than 10 degrees, while maintaining demonstrated small tracking errors 
% in both travel and elevation.
% 
% 4.Continue travel in counterclockwise direction for 20 degrees with slow speed 
% and fully stop at the end with an overshoot of no more than 1 degree, while 
% maintaining demonstrated small tracking errors in both travel and elevation.
% 
% 5.Land at the original initial position as fast as possible but with minimum 
% overshoot (as small as possible but no more than 1 degree.

Maneuver1_aggression=20;% deg/                                                                                                                                                                                        (see step1 req)
Maneuver2_aggression=5;% deg/s trav speed during initial CW travel (see step2 req)
Maneuver3_aggression=3;% deg/s trav speed returning CCW (see step3 req)
Maneuver4_aggression=0.3;% deg/s trav speed for final trav segment (see step4 req)
Maneuver5_aggression=3.5;% deg/s elev speed during landing (see step5 req)
All_Maneuver_smoothness=16;%1/sec sets the smoothness of the manuvers 
All_Delay_betwn_actns=28;%sec waiting: adjusts the wait time between sending trajectory comands
All_Sim_time=300;%sec running: sets how long the simulation will run for