%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% Helper function for main file name'ENPM661_RRL_Astar.m'
% 
% Code By: Mayank Pathak
%          115555037
%
% This Code contains the function to xport velocities in the format
% prescribed by the professor. The output velocities are saved in a .txt
% file. 
%
% Dependencies: None.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function export_velocity(path, time_interval)


% initialize velocity
linear.x = 0;
linear.y = 0;
linear.z = 0;

angular.x = 0;
angular.y = 0;
angular.z = 0;

% time and time_interval
timeStamp = 0;

% initialze file
fileID = fopen('velo_list.txt','w');
fprintf(fileID,'%6s %12s %18s %24s %30s %36s %42s\r\n','timeStamp','linear.x',...
    'linear.y', 'linear.z', 'angular.x', 'angular.y', 'angular.z');

    for i = length(path) : -1 : 1
        
        % mode 1
        if (path(i).mode == 1)
            linear.x = 0.1;         % 0.1 m/s
            angular.z = 0.1;        % 0.1 radian/s
        end
        
        % mode 2
        if (path(i).mode == 2)
            linear.x = 0.1;         % 0.1 m/s
            angular.z = 0.05;        % 0.05 radian/s
        end
        
        % mode 3
        if (path(i).mode == 3)
            linear.x = 0.1;         % 0.1 m/s
            angular.z = 0;        % 0 radian/s
        end
        
        % mode 4
        if (path(i).mode == 4)
            linear.x = 0.1;         % 0.1 m/s
            angular.z = -0.05;        % -0.05 radian/s
        end
        
        % mode 5
        if (path(i).mode == 5)
            linear.x = 0.1;         % 0.1 m/s
            angular.z = -0.1;        % 0.05 radian/s
        end
        
        % set for each line
        line = [timeStamp linear.x linear.y linear.z angular.x angular.y angular.z];
        
        % update timeStamp
        timeStamp = timeStamp + time_interval;
        
        % print line to text file
        fprintf(fileID,'%6.2f %12.2f %18.2f %24.2f %30.2f %36.2f %42.2f\r\n', line);
        
    end
    
% close file
fclose(fileID);
        
            

end