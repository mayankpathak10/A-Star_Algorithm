%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ENPM 661 Planning For Autonomous Robotics
% Project 3 - Spring 2018
% To implement A* algorithm on the scene given in Project 2 and find 
% optimal path between two given points. The Start and Goal Points are
% given interactively by the User.All the expanded Nodes are displayed as 
% Peach colored dots.
%
% Dependencies: This code uses a function named 'ObstacleSpace_Generator,
% which is saved as 'ObstacleSpace_Generator.m' with this file.
%
% Notes: Livr Display can be turned on by assigning 'Display_LIve' = 1.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Defining Obstacle Parameters and Initializing Variables
clear all
clc 

size_x = 250;
size_y = 150;
rect_x = [55,55,105,105];
rect_y = [67,112,112,67];
poly_x = [120,158,165,188,168,145,120];
poly_y = [55,51,89,51,14,14,55];
circ_x = 180;
circ_y = 120;
circ_r = 15;
Orectangle = [rect_x;rect_y];
Opolygon = [poly_x;poly_y];
Ocircle = [circ_x;circ_y;circ_r];
Arena = ones(size_y,size_x);

% if =s 1 then displays live expansion
Display_Live = 1;

%  Calling the function to make Obstacle Space
[Arena] = ObstacleSpace_generator(Orectangle,Opolygon,Ocircle,Arena);
                         

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SET THE MAP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure;
% draw grid environment
xp3 = [0 ,250, 250, 0 ,0];
yp3 = [0 ,0, 150 ,150 ,0];
drawnow
plot(xp3,yp3);
alpha(0.3);

V = [0 0; 0 size_y; size_x size_y; size_x 0];
F = [1 2 3 4];
patch('Faces',F,'Vertices',V,'FaceColor','blue','FaceAlpha',0.4);
hold on;



% Defining Rectangular and Polygonal Obstacle

o1x = 55; o1y = 67; o1xw = 50; o1yw = 45;
rect_pos = [ o1x o1y o1xw o1yw];
rectangle('Position',rect_pos,'FaceColor','b');

OX1 = [120 158 165 188 168 145 120];
OY1 = [55 51 89 51 14 14 55];
patch(OX1,OY1,'blue');


% Defining Circular Obstacle
center_x=180;
center_y = 120;
t = 0:0.01:2*pi;
x = 15*cos(t)+ center_x;
y = 15*sin(t) + center_y;
drawnow
plot(x,y),
fill(x,y,'b');

title(' \fontsize{20} Please Select a Start Point');
% start locations
[ sx,sy] = ginput(1);
sx = round(sx); sy = round(sy);
Start_Node = [sx sy];

%%% Check inputs for boundaries

while Arena(sy,sx) == 0
    title('\fontsize{20}Please Select a Correct Start Point');
    [sx,sy] = ginput(1);
    sx = round(sx); sy = round(sy);
    Start_Node = [sx sy];
end

title('\fontsize{20}Now Please Select a Goal Point');
[gx,gy] = ginput(1);
gx = round(gx); gy = round(gy);
Goal_Node = [gx gy];

%%% Check inputs for boundaries
while Arena(gy,gx) == 0
    title('\fontsize{20}Please Select a Correct Goal Point');
    [gx,gy] = ginput(1);
    gx = round(gx); gy = round(gy);
    Goal_Node = [gx gy];
end

title('\fontsize{20}Computing the Optimal Path');
plot(Start_Node(1),Start_Node(2),'g.','MarkerSize',20);
plot(Goal_Node(1),Goal_Node(2),'r.','MarkerSize',20);
drawnow;


% Color code
% Green - For Initial Node
% Red   - For Goal Node
% Pink  - For Neighbourhood Node
% Gray  - For Visited Node

% NodesInfo = [NodeNo, ParentNodeNo, CostToCome, totalCost, Status]
% CostToCome: The distance between the current node and start location, based on costToCome function.
% TotalCost: The sum of heuristic cost associated with current node and CostToCome. 
% Heuristic is the euclidean distance betweeen current node and goal node
% Status: It shows whether the node is open or not. 1 indicates closed node and 0 indicates open node.
Nodes = [];
NodeInfo = [];
Nodes(:,:,1) = Start_Node;
CostToCome = 0;
totalCost = heuristicCost(Start_Node, Goal_Node);
NodesInfo(:,:,1) = [1,0,0,totalCost, 1];  
NoOfNodes = 2; 
nodeNo = 1;

NodeSet.Nodes = Nodes;
NodeSet.NodesInfo = NodesInfo;

flag = 0;
tic 

%%%% Implementing A*

while 1
    CurrentNode = Nodes(:,:,nodeNo);
    c_x = CurrentNode(1);
    c_y = CurrentNode(2);
    for x = -1:1
        for y = -1:1
                NeighborNode = [CurrentNode(1) + x, CurrentNode(2) + y];
      
%         Enforcing Boundary Conditions and Not current condition.
              if ( ~((x == 0) && (y == 0)) && c_x+x>0 && c_y+y>0 &&...
                      c_x<size_x && c_y<size_y)                  
                    if(Arena(NeighborNode(1,2),NeighborNode(1,1))== 1)                
                       % To check whether it is aready expanded
                        result = find(all(bsxfun(@eq, Nodes, NeighborNode)));
                        if isempty(result)
                            CostToCome = NodesInfo(1,3,nodeNo) + costToCome(x,y);
                            CostToGo = heuristicCost(NeighborNode, Goal_Node);
                            totalCost = CostToCome + CostToGo;
                            Nodes(:,:,NoOfNodes) = NeighborNode;
                            NodesInfo(:,:,NoOfNodes) = [NoOfNodes, nodeNo, CostToCome, totalCost, 1];
                            NoOfNodes = NoOfNodes + 1;
                            if Display_Live == 1
                                drawnow
                            end
                            scatter(NeighborNode(1), NeighborNode(2), 5, [1 0.7 0.7], 'filled');
 
                            if isequal(NeighborNode,Goal_Node)
                                flag = 1;
                            end
                        else
                      
                            if NodesInfo(1,5,result) ~= 0
                                oldCostToCome = NodesInfo(1,3,result);
                                CostToCome = NodesInfo(1,3,nodeNo) + costToCome(x,y);
                                if CostToCome < oldCostToCome
                                    CostToGo = heuristicCost(NeighborNode, Goal_Node);
                                    totalCost = CostToCome + CostToGo;
                                    % Update the parent node number,CostToCome and Total Cost
                                    NodesInfo(1,2:4,result) = [nodeNo, CostToCome, totalCost];                            
                                end
                            end
                        end
                    end
              end
        end
    end
        
    if flag == 1
        break;
    end
       
    NodesInfo(1,5,nodeNo) = 0;
    leastCost = inf;
    for i = 1:NoOfNodes-1
        if NodesInfo(1,5,i) ~= 0
            if NodesInfo(1,4,i) < leastCost
                leastCost = NodesInfo(1,4,i);
                nodeNo = NodesInfo(1,1,i);
            end
        end
    end  
end

%%%% BackTracing the parent node.
i = 1;
path = [];
path(:,1) = Goal_Node; % Adding Goal_Node to the path
traceNode = NodesInfo(1,2,NoOfNodes-1); 
while traceNode ~= 0
    i = i + 1;
    path(:,i) = Nodes(:,:,traceNode);
    traceNode = NodesInfo(1,2,traceNode);
end
path(:,i+1) = Start_Node; % Adding the Start_Node to the path

% Plot the path
drawnow
plot(path(1,:),path(2,:),'k');
scatter(path(1,:),path(2,:),10,[0.5 0.5 0.5],'filled');
plot(Start_Node(1),Start_Node(2),'g.','MarkerSize',20);
plot(Goal_Node(1),Goal_Node(2),'r.','MarkerSize',20);
toc % Stop the timer


% Function to calculate the heuristic Cost i.e. the euclidean distance
% between two nodes
function distance = heuristicCost(node1, node2)
    
    distance = sqrt(power((node1(1) - node2(1)),2) + ...
                    power((node1(2) - node2(2)),2));  
end


% Function to calculate the Cost to Come between 2 nodes
% costToCome = 1 for straight and 1.4 for Diagonal
function cost = costToCome(x,y)
   
    
    if abs(x) == abs(y) 
        cost = 1.4;
    else
        cost = 1;
    end
end