%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% Helper function for main file name'ENPM661_RRL_Astar.m'
% 
% Code By: Mayank Pathak
%          115555037
%
% This Code generates the Map of Robotics Realization Lab (RRL) of UMD and
% also generates a bitmap to indentify location of points. All nodes lying
% on or in obstacles are valued as 1 and all other nodes are values as 0.
%
% Dependencies: None
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function bitmap = RRL_generator(resolution,turtle_R)

% Table geometries
% [Center_x Cetner_y width Length]

table_1 = [1.825 8.225 0.8 2];
table_2 = [3.525 8.225 0.8 2];
table_3 = [6.20 4.85 1.599 3.19];
table_4 = [9.952 4.85 1.599 3.19];
table_5 = [12.7 8.95 1.6 1.1];
table_6 = [14.3 5.125 0.8 2];
table_7 = [14.3 3.125 0.8 2];

% width and height of RRL (in meters)
frame = [15, 10];
map_width = abs(frame(1));
map_height = abs(frame(2));

% create bitmap for RRL
bitmap_width = round(map_width/resolution);
bitmap_height = round(map_height/resolution);

% Creating bitmap with of RRL size
bitmap = zeros(bitmap_width, bitmap_height);


Obstacles = [table_1;table_2;table_3;table_4;table_5;table_6;table_7];

for i = 1:length(Obstacles)
    table = Obstacles(i,:);
    table_center_x = table(1);
    table_center_y = table(2);
    table_length = table(3);
    table_height = table(4);
    
    Corner1 = [table_center_x-(table_length/2) table_center_y-(table_height/2)];
    Corner2 = [table_center_x+(table_length/2) table_center_y+(table_height/2)];
    
    
    LeftDown_map = Scale_Map(Corner1,resolution);
    RightUp_map = Scale_Map(Corner2, resolution);
    Obstacle_map = [LeftDown_map table_length/resolution table_height/resolution];

    rectangle('Position', Obstacle_map, 'EdgeColor','k', 'LineWidth',3), hold on;
    
     
% Assigning value 1 to all nodes lying inside obstacle, including
% clearance for robot radius
    x_left = LeftDown_map(1)-turtle_R;
    if (x_left < 1)
        x_left = 1;
    end
    
    x_right = RightUp_map(1)+turtle_R;
    if (x_right > bitmap_width)
        x_right = bitmap_width;
    end
    

    y_down = LeftDown_map(2)-turtle_R;
    if (y_down < 1)
        y_down = 1;
    end
    
    y_up = RightUp_map(2)+turtle_R;
    if (y_up > bitmap_height)
        y_up = bitmap_height;
    end
    

    x_range = round([x_left x_right]);
    y_range = round([y_down y_up]);
    

    for p = x_range(1) : x_range(2)
        for q = y_range(1): y_range(2)
            bitmap(p, q) = 1;
        end
    end

end

% set graph plot x,y size

ax = gca;
xlim = bitmap_width;
ylim = bitmap_height;
ax.XLim = [0 xlim];
ax.YLim = [0 ylim];


% Draw Grid Environment
xp3 = [0 ,0, 150, 0 ,0];
yp3 = [0 ,100, 100 ,100 ,0];
drawnow
plot(xp3,yp3);
alpha(1);

hold on;





