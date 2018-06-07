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

clc;
clear;


%%%% Control Variables  %%%%
resolution = 0.1;       % Scale Factor
Write_Video = false;    % To save video file. true = save video
tolerance = 4;          % Allowance for considering neighbor nodes of goal 
time_interval = 5;      % Time interval between nodes exapsion
turtle_R = 2;           % Radius of Turtle bot
Display_Live = true;    % To Display Live expansion of Nodes

%%%% Drawing RRL Graph %%%%%
figure;
bitmap = RRL_generator(resolution,turtle_R);

% Start Node Location
% Based on Turtle Bot Location on Scene of RRL Given
Start_Node = [.35 .85];
Start_node.coordinate = Scale_Map(Start_Node, resolution);
plot(Start_node.coordinate(1), Start_node.coordinate(2),'g.','MarkerSize',10);
hold on

% Prompting and selecting Goal Node location from user
title('\fontsize{20}Now Please Select a Goal Point');
[gx,gy] = ginput(1);
gx = round(gx)/10; gy = round(gy)/10;
Goal_Node = [gx gy];
Goal_node.coordinate = Scale_Map(Goal_Node, resolution);


%%% Plotting the obtained results
plot(Goal_node.coordinate(1), Goal_node.coordinate(2),'r.','MarkerSize',10);
drawnow;
title('\fontsize{20}Computing the Optimal Path');
hold on 

% For saving Video     
v = VideoWriter('newfile.avi');
open(v)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                PERFORMING SEARCH                                 %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%% Initializing search paramenters %%%%
Start_node.orient = pi/2;
Start_node.parent = 0;
Start_node.trajectory = [];
Start_node.info = 1;
Start_node.gScore = 0;
Start_node.fScore = Start_node.gScore + hScore_estimate(Start_node, Goal_node);
Start_node.actual_coord = Start_node.coordinate;
Start_node.mode = [];




% Defining Openset and Closeset
openSet = [];
closeSet = java.util.Hashtable;
openSeth = java.util.Hashtable;


% add start node to openSet
openSet = [openSet Start_node];
openSeth.put(Start_node.info, Start_node.coordinate);

% NodeSet to store all nodes
NodeSet = Start_node;

% initialize 2D NodeMap to store node 
emptyNode.coordinate = [];
emptyNode.orient = [];
emptyNode.trajectory = [];
emptyNode.parent = [];
emptyNode.info = [];
emptyNode.gScore = [];
emptyNode.fScore = [];
emptyNode.actual_coord = []; 
emptyNode.mode = [];

% create a NodeMap with same size of bitmap
bitmap_length = length(bitmap(:,1));
bitmap_height = length(bitmap(1,:));
NodeMap(bitmap_length, bitmap_height) = emptyNode;


% while loop for A* algorithm
while ~isempty(openSet) || reachGoal == true
    
    % find the node with lowest fScore and remove that node from openSet
    Best_Child=1;
    for i=1:length(openSet)
        if openSet(i).fScore < openSet(Best_Child).fScore
            Best_Child=i;
        end
    end
    
    currentNode = openSet(Best_Child);
    openSet(Best_Child) = [];       %remove this node from openSet
        
    openSeth.remove(currentNode.info); % remove current node from openSeth

    
    dist = sqrt((currentNode.coordinate(1) - Goal_node.coordinate(1))^2 ...
              + (currentNode.coordinate(2) - Goal_node.coordinate(2))^2);
    if dist < tolerance
        reachGoal = true;
        break
    else
        reachGoal = false;
    end
    
    % expanding CurrentNode
    subtree = expandNode(currentNode,NodeSet,closeSet,openSeth,bitmap,time_interval,resolution,Display_Live);
   
    if Write_Video == true
        frame = getframe();
        img = frame2im(frame);
        writeVideo(v,img);
    end
    
    
% To update NodeMap and Parent for current Node
    for i = 1 : length(subtree)
        
        neighbor = subtree(i);
        NodeMapCoord = subtree(i).coordinate;
        % get the node from NodeMap
        NodeOnMap = NodeMap(NodeMapCoord(1), NodeMapCoord(2));
        if (isempty(NodeOnMap.coordinate))
            subtree(i).fScore =  subtree(i).gScore + hScore_estimate(subtree(i), Goal_node);
            NodeMap(NodeMapCoord(1), NodeMapCoord(2)) = subtree(i);
        else
            if (neighbor.gScore < NodeOnMap.gScore)
            subtree(i).fScore =  subtree(i).gScore + hScore_estimate(subtree(i), Goal_node);
            NodeMap(NodeMapCoord(1), NodeMapCoord(2)) = subtree(i);
            else
                subtree(i) = NodeOnMap;
            end
            
        end
    end
  
    NodeSet = [NodeSet subtree];
    
    % add subtree to openSet
    openSet = [openSet subtree];
    
    % add subtree to openSetHash
    if ~isempty(subtree)
        for n = 1 : length(subtree)
            openSeth.put(subtree(n).info, num2str(subtree(n).coordinate));
        end
    end
    
    % add currentNode to closeSet
    closeSet.put(currentNode.info, num2str(currentNode.coordinate));
    
end

 if reachGoal == true 
    % initialize the path from Node to startNode
    path = currentNode;

    while (currentNode.parent ~= 0)
        currentNode = NodeSet(currentNode.parent);      
        path = [path currentNode];                      

    end

    % draw trajectory 
    total_trajectory = [];
    for i = length(path) : -1 : 1
        total_trajectory = [total_trajectory path(i).trajectory];
    end

    line(total_trajectory(1,:),total_trajectory(2,:), 'color', 'green', 'LineWidth', 3);
    hold on;
    
    if Write_Video == true
    frame = getframe();
    img = frame2im(frame);
    writeVideo(v,img);
    end
    close(v);
    % export linear and angular velocity to text file
    export_velocity(path,time_interval);

    return;
end


%%%% For H-Score using Euclidean Distance
function dist = hScore_estimate(node1,node2)
    dist = sqrt((node1.coordinate(1) - node2.coordinate(1))^2 ...
              + (node1.coordinate(2) - node2.coordinate(2))^2);
end




