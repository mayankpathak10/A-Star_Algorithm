%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 2 - Spring 2018
% 
% This function takes Obstacle edge coordinates, Workspace dimentions and
% generates a arena (Work Space) that is pre-processed for the allowed 
% Coordinates. For each point in the Work Space dimentions, it checks 
% wether it lies in Obstacle Space or in the Work Space.
% 
% The obstacle space is determined using Half plane Method. 
% We define equation of lines of each obstacle and use inequality to find
% weather the query point satiesfies all the inequalities or not. If it
% does then it lies inside or on the boundary of obstacle. If it is inside
% or on obstacle, it will be assigned a value of 1, otherwise 0. 
%  The result of this function is a Pre-Processed Work Space.
% 
% Then the main Program will expland only the nodes with value 0.
%
% Code By: Mayank Pathak
%          115555037
%%%%



function [Arena] = ObstacleSpace_generator(Orectangle,Opolygon,Ocircle,Arena)
                                                    
for i = 1:size(Arena,1)        % iterate about x axis
    for j = 1:size(Arena,2)    % iterate about y axis
        q_x = j;
        q_y = i;
        
%   Implementing inequality for rectangle
        if (ge(q_x,Orectangle(1,1)) && le(q_x,Orectangle(1,3)) &&...
           ge(q_y,Orectangle(2,1)) && le(q_y,Orectangle(2,3)))
             Arena(i,j) = 0;
%   Implementing inequality for Circle
        elseif le(sqrt((q_x-Ocircle(1))^2 + (q_y-Ocircle(2))^2),15)
             Arena(i,j) = 0;
        end
%   Implementing inequality for polygon
        [In,On] = inpolygon(q_x,q_y,Opolygon(1,:),Opolygon(2,:));
       
        if In || On
            Arena(i,j) = 0;
        end
    end
end
end



