%% Points

nrows = 500;
ncols = 500;

obstacle = false(nrows,ncols);

[x,y] = meshgrid (1:ncols, 1:nrows);

%% Obstacles

%creating obstacles
obstacle (y<10 | y>490 | x<10 | x>490) = true; % boundaries
obstacle (y > 150 & y < 250 & x > 150 & x < 250) = true;

figure;
imshow(~obstacle);

axis ([0 ncols 0 nrows]);
axis xy; %flips the y axis over

axis on;
xlabel ('x axis');
ylabel ('y axis');

title('Graph');
grid on

%% RRT Parameters

Max_Connect_Length = 500; % length of the node
Goal_found = false; % setting to false at first
Segments = 100; % Using segmentation to check for obstacles
                % Divide the edge between new node and neighbors into segments
                % Checking if all points on the line are away from
                % STILL WORKING ON THIS NOT COMPLETE YET
                % obstacles 

%% RRT Algorithm

Node_count = 1; % setting node counter to 1 at first for starting node
map = zeros(size(obstacle)); % setting the whole map to 0
map(obstacle) = 1; % setting the obstacles as 1
Tree_Connections = []; % holding the nodes and the tree

% Start Node
Start_node = [450, 450]; % We're assuming a starting node
Tree_Connections = [sub2ind(size(map), Start_node(1), Start_node(2)) Inf];
    % Adding the start node to the tree array
map(Start_node(2), Start_node(1)) = 2; % Marking the node as a 2
hold on
plot(Start_node(2), Start_node(1), 'b^')
hold off

% Goal Node
Goal_node = [50, 50];
hold on
plot(Goal_node(2), Goal_node(1), 'mv')
hold off

while(true)
        %% generating node at a random location
        Node_X = randi(ncols);
        Node_Y = randi(nrows);
        
        % Check if is through obstacle
        if (map(Node_Y, Node_X) == 1 || map(Node_Y, Node_X) == 2)
            continue;
        end
        
        %% Connect new node to closest node
        % For loop checks nodes within the max_length and puts in
        % neighbor_distances
        neighbor_distances = [];
        for i = 1: numel(Tree_Connections(:,1)) % 1 to number of elements in Tree_Connections[nodes]
            if (Tree_Connections(i,1) == Inf)
                break
            end
            [row, col] = ind2sub(size(map),Tree_Connections(i,1));
            
            % checks if in range
            if (norm([Node_Y, Node_X]-[row,col]) > Max_Connect_Length)
                continue
            end
            
            % ADD SOMETHING WITH SEGMENTATION
            
            % adding the node as a connection node (with the ID and the
            % distance to the new node)
            neighbor_distances = [neighbor_distances; [Tree_Connections(i,1), norm([Node_Y, Node_X]-[row,col])]];
                
        end
        
        %% Choose the closest node to connect to
        if (size(neighbor_distances) > 0)
            
            % Add node to the map and set parent to closest node
            map(Node_Y, Node_X) = 2;
            Node_count = Node_count + 1;
            distances_sorted = sortrows(neighbor_distances, 2);
            Tree_Connections = [Tree_Connections; sub2ind(size(map), Node_Y, Node_X) distances_sorted(1,1)];
            % adding the node from the distances_sorted because choosing
            % from the last part of the sorted array
            
            % Plotting the new node
            hold on
            plot(Node_X, Node_Y, 'r*')
            hold off
            
            % Plot edge between new node and its parent
            [row, col] = ind2sub( size(map), distances_sorted(1,1));
            hold on
            line([Node_X, col], [Node_Y,row])
            hold off
            
        else
            continue;
        end
        
        %% checking if we are close to the goal
        placeholder = [Node_Y, Node_X]-Goal_node;
        if(norm(placeholder) < Max_Connect_Length)
            Node_count = Node_count + 1;
            
            % Adding the goal_node to map
            Tree_Connections(Node_count, 1) = sub2ind(size(map), Goal_node(1), Goal_node(2));
            % Setting goal node parent to the new node
            Tree_Connections(Node_count,2) = sub2ind(size(map), Node_Y, Node_X);
            map(Goal_node(1),Goal_node(2))=2;
            
            %plot edge between node and goal
            hold on
            line([Node_X, Goal_node(2)], [Node_Y, Goal_node(1)])
            hold off
            
            Goal_found = true;
            
        end
        
        %% mark the path
        if (Goal_found)
            route = [];
            route = make_path(map, Nodes, Tree_Connections)
            % STILL WORKING ON MAKING THE PATH
            % plot the route
        end
        
        %% break if goal found
        if (Goal_found)
            break
        end

end
