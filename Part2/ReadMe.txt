%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% To implement A* algorithm on differential drive robot to find optimal path 
% between two given points. The Start Point has been set as the position of
% robot as given in the scene, and Goal Point is given interactively by the
% User.
% The Code expands each node by considering holonomic constraints and hence
% results in the expansion tree like structure.
% 
% Code By: Mayank Pathak
%          115555037
%
% Dependencies: This code uses four helper functions named 
% 'RRL_Generator, 'Scale_Map','expandNode','export_velocity'. All of them
% are saved in the same folder as this code.
%
% Notes: Code by github user name 'Shaotujia' has been very helpful in 
% understanding the trajectory generation for differential drive robot.
% This code may contain some functions from the source.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Run the file named 'ENPM661_RRL_Astar. You can control the variables like
Display_Live, Video_Write, resoultion etc.