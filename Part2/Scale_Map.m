%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% Helper function for main file name'ENPM661_RRL_Astar.m'
% 
% Code By: Mayank Pathak
%          115555037
%
% This Code scales the given world coordinate according to the resolution
% given by the main program.
%
% Dependencies: None
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function map_point = Scale_Map(world_point, resultion)
    
    % scaling map by the resolution defined
    map_point_x = round(world_point(1)/resultion);
    map_point_y = round(world_point(2)/resultion);
 
    map_point = [map_point_x map_point_y];
    
    
end