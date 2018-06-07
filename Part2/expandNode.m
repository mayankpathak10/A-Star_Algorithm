%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% Helper function for main file name'ENPM661_RRL_Astar.m'
% 
% Code By: Mayank Pathak
%          115555037
%
% This Code expands the current node in consideration and returns the
% resulting sub trees(set of Current Children Nodes).
%
% Dependencies: None
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [nodeSubtree, numOfNodes] = expandNode(Node, NodeSet, closeSet, openSeth, bitmap, time_interval, resolution,Display_Live)
        
 
    SizeOfNodeSet = length(NodeSet);
    numOfNodes = 0;
    nodeSubtree = [];
    
    start_c = Node.actual_coord;
    start_o = Node.orient;
    
   
    x_start = start_c(1);
    y_start = start_c(2);
    
    linear_velo_1 = 0.1 / resolution;               
    angular_velo_1 = 0.1;                   
    [point_1, path_1] = FindPath(linear_velo_1, angular_velo_1, start_o, time_interval, x_start, y_start);
    p1_c = [round(point_1(1)) round(point_1(2))];     
    
    % check if the point_1 is within the bitmap
    if ((is_within_bitmap(p1_c, bitmap)) && bitmap(p1_c(1),p1_c(2))~=1)
        
        % assign node_1 element
        node_1.coordinate = p1_c;               
        node_1.orient = point_1(3);
        node_1.actual_coord = [point_1(1) point_1(2)];    
        
        % check if LeftNode exists in closeSet or openSet or inside Obstacle
             if  ~openSeth.containsValue(num2str(node_1.coordinate))
                if ~closeSet.containsValue(num2str(node_1.coordinate))
            
                    numOfNodes = numOfNodes + 1;                % count node numbers in the subtree
                    node_1.info = SizeOfNodeSet + numOfNodes;     % Set up node info for new node
                    node_1.parent = Node.info;                % set up parent node for new node
                    node_1.trajectory = path_1;           % set up trajectory
                    node_1.gScore = Node.gScore + point_1(4);   % compute gScore for LeftNode
                    node_1.mode = 1;                                % set mode number for node
                    
                    nodeSubtree = [nodeSubtree node_1];
                    
                    %draw trajectory
                    if Display_Live == true
                        drawnow
                    end
                    line(path_1(1,:), path_1(2,:));
                    hold on;
                end
            end
    end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     % expand node using mode 2
%     [endpoint_2, trajectory_2] = mode(2, init_coord, init_theta, resolution, time_interval);
    linear_velo_2 = 0.1 /resolution;        % linear velocity 0.1m/s        
    angular_velo_2 = 0.05;                   % angular velocity 0.5 radian/s
    [point_2, path_2] = FindPath(linear_velo_2, angular_velo_2, start_o, time_interval, x_start, y_start);

    p2_c = [round(point_2(1)) round(point_2(2))];
    
    % check if the end_point_1 is within the bitmap
    if ((is_within_bitmap(p2_c, bitmap)) && bitmap(p2_c(1),p2_c(2))~=1)
        
        % assign node_2 element
        node_2.coordinate = p2_c;                   % coordinate on NodeMap
        node_2.orient = point_2(3);
        node_2.actual_coord = [point_2(1) point_2(2)];    % acutal coordindate
        
        % check if LeftNode exists in closeSet or openSet or inside Obstacle
             if  ~openSeth.containsValue(num2str(node_2.coordinate))
                if ~closeSet.containsValue(num2str(node_2.coordinate))
            
                    numOfNodes = numOfNodes + 1;              % count node numbers in the subtree
                    node_2.info = SizeOfNodeSet + numOfNodes; % Set up node info for new node
                    node_2.parent = Node.info;                % set up parent node for new node
                    node_2.trajectory = path_2;               % set up trajectory
                    node_2.gScore = Node.gScore + point_2(4); % compute gScore for LeftNode
                    node_2.mode = 1;                          % set mode number for node
                    
                    nodeSubtree = [nodeSubtree node_2];
                    
                    %draw trajectory
                    if Display_Live == true
                        drawnow
                    end
                    line(path_2(1,:), path_2(2,:));
                    hold on;
                end
            end
    end
    
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % expand node using mode 3
    %[endpoint_3, trajectory_3] = mode(3, init_coord, init_theta, resolution, time_interval);
    linear_velo_3 = 0.1 /resolution;        % linear velocity 0.1m/s        
    angular_velo_3 = 0;                   % angular velocity 0.5 radian/s
    
    [point_3, path_3] = FindPath(linear_velo_3, angular_velo_3, start_o, time_interval, x_start, y_start);
    
    p3_c = [round(point_3(1)) round(point_3(2))];
    
    % check if the end_point_1 is within the bitmap
    if ((is_within_bitmap(p3_c, bitmap)) && bitmap(p3_c(1),p3_c(2))~=1)
        
        % assign node_3 element
        node_3.coordinate = p3_c;                   % coordinate on NodeMap
        node_3.orient = point_3(3);
        node_3.actual_coord = [point_3(1) point_3(2)];    % actual coordinate
        
        % check if LeftNode exists in closeSet or openSet or inside Obstacle
             if  ~openSeth.containsValue(num2str(node_3.coordinate))
                if ~closeSet.containsValue(num2str(node_3.coordinate))
            
                    numOfNodes = numOfNodes + 1;                % count node numbers in the subtree
                    node_3.info = SizeOfNodeSet + numOfNodes;     % Set up node info for new node
                    node_3.parent = Node.info;                % set up parent node for new node
                    node_3.trajectory = path_3;           % set up trajectory
                    node_3.gScore = Node.gScore + point_3(4);   % compute gScore for LeftNode
                    node_3.mode = 3;                                % set mode number for node
                    
                    nodeSubtree = [nodeSubtree node_3];
                    
                    %draw trajectory
                    if Display_Live == true
                        drawnow
                    end
                    line(path_3(1,:), path_3(2,:));
                    hold on;
                    
                end
            end
    end
     
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % expand node using mode 4
    %[endpoint_4, trajectory_4] = mode(4, init_coord, init_theta, resolution, time_interval);
     linear_velo_4 = 0.1 /resolution;        % linear velocity 0.1m/s        
    angular_velo_4 = -0.05;                   % angular velocity 0.5 radian/s
    
    [point_4, path_4] = FindPath(linear_velo_4, angular_velo_4, start_o, time_interval, x_start, y_start);
    p4_c = [round(point_4(1)) round(point_4(2))];
    
    % check if the end_point_1 is within the bitmap
    if ((is_within_bitmap(p4_c, bitmap)) && bitmap(p4_c(1),p4_c(2))~=1)
        
        % assign node_4 element
        node_4.coordinate = p4_c;                       % cooridnate on NodeMap
        node_4.orient = point_4(3);
        node_4.actual_coord = [point_4(1) point_4(2)];        % actual cooridnate 
        
        % check if LeftNode exists in closeSet or openSet or inside Obstacle
             if  ~openSeth.containsValue(num2str(node_4.coordinate))
                if ~closeSet.containsValue(num2str(node_4.coordinate))
            
                    numOfNodes = numOfNodes + 1;                % count node numbers in the subtree
                    node_4.info = SizeOfNodeSet + numOfNodes;     % Set up node info for new node
                    node_4.parent = Node.info;                % set up parent node for new node
                    node_4.trajectory = path_4;           % set up trajectory
                    node_4.gScore = Node.gScore + point_4(4);   % compute gScore for LeftNode
                    node_4.mode = 4;                                % set mode number for node
                    
                    nodeSubtree = [nodeSubtree node_4];
                    
                    %draw trajectory
                    if Display_Live == true
                        drawnow
                    end
                    line(path_4(1,:), path_4(2,:));
                    hold on;
                    
                end
            end
    end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % expand node using mode 5
%     [endpoint_5, trajectory_5] = mode(5, init_coord, init_theta, resolution, time_interval);
     linear_velo_5 = 0.1 /resolution;        % linear velocity 0.1m/s        
    angular_velo_5 = -0.1;                   % angular velocity 0.5 radian/s
    
    [point_5, path_5] = FindPath(linear_velo_5, angular_velo_5, start_o, time_interval, x_start, y_start);
    p5_c = [round(point_5(1)) round(point_5(2))];
    
    % check if the end_point_1 is within the bitmap
    if ((is_within_bitmap(p5_c, bitmap)) && bitmap(p5_c(1),p5_c(2))~=1)
        
        % assign node_5 element
        node_5.coordinate = p5_c;
        node_5.orient = point_5(3);
        node_5.actual_coord = [point_5(1) point_5(2)];
        % check if LeftNode exists in closeSet or openSet or inside Obstacle
             if  ~openSeth.containsValue(num2str(node_5.coordinate))
                if ~closeSet.containsValue(num2str(node_5.coordinate))
            
                    numOfNodes = numOfNodes + 1;                % count node numbers in the subtree
                    node_5.info = SizeOfNodeSet + numOfNodes;     % Set up node info for new node
                    node_5.parent = Node.info;                % set up parent node for new node
                    node_5.trajectory = path_5;           % set up trajectory
                    node_5.gScore = Node.gScore + point_5(4);   % compute gScore for LeftNode
                    node_5.mode = 5;                                % set mode number for node
                    
                    nodeSubtree = [nodeSubtree node_5];
                    
                    %draw trajectory
                    if Display_Live == true
                        drawnow
                    end
                    line(path_5(1,:), path_5(2,:));
                    hold on;
                    
                end
            end
    end
end
    
    
function [endpoint, trajectory] = FindPath(v_init, w_init, theta_init, t_min, x_start, y_start)

% initialize the function of displacement along x and y axis
%delta_x = @(t) cos(w_init .* t + theta_init) .* (v_init);
%delta_y = @(t) sin(w_init .* t + theta_init) .* (v_init);



% initialize move_cost
move_cost = 0;

% Initialize the start point to draw a increment line
seg_X = x_start;
seg_Y = y_start;

%Initialize step
step = t_min/20;

for t = 0: step : t_min

% obtain the distance between reached point to goal 
% deltaX = integral(delta_x, 0 , t, 'ArrayValued',true);
% deltaY = integral(delta_y, 0 , t, 'ArrayValued',true);
deltaX = cos(w_init * t + theta_init) * (v_init * t);
deltaY = sin(w_init * t + theta_init) * (v_init * t);


% compute cost of movement
move_cost = move_cost + sqrt(deltaX^2 + deltaY^2);

% Get end point of small increment
seg_X(end + 1) = x_start + deltaX;
seg_Y(end + 1) = y_start + deltaY;

end

% round the end point to integer
%seg_X = [seg_X round(seg_X(end))];
%seg_Y = [seg_Y round(seg_Y(end))];

% get the orientation of endpoint
end_theta = w_init * t_min + theta_init;
% assign value for end point coordinate
endpoint = [seg_X(end) seg_Y(end) end_theta move_cost];


% store trajectory for future draw
trajectory = [seg_X; seg_Y];

% draw trajectory
% h = line(seg_X, seg_Y);
% hold on;
end

function within_map = is_within_bitmap(point_coordinate, bitmap)

x = round(point_coordinate(1));         % assign coordinate x
y = round(point_coordinate(2));         % assign coordinate y

bitmap_length = length(bitmap(:,1));    % row_size of bitmap
bitmap_height = length(bitmap(1,:));    % col_size of bitmap

if (x > 0) && (x < bitmap_length+1) && (y > 0) && (y < bitmap_height+1)
    within_map = true;
else
    within_map = false;
end

end