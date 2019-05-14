function ans = door_detector(nearby_door_right,nearby_door_left,scan) %input: ranges
%% Process ranges
x= [];
y=[];

% Transform data from scan
for n = 1:length(scan)
    % remove points that are less than 15 milli away from scanner
    if scan(n) > 15 
        xn = cosd(-30 + (n-1)*(240/682)) * scan(n);
        yn = sind(-30 + (n-1)*(240/682)) * scan(n);
        x = [x; xn];
        y = [y; yn];
    end

end
points=[x,y];

%% Set rightpoints and leftpoints
leftpoints = []; % [x,y]
rightpoints = [];

% Søker bare for 1 meter frem her.
search_range = 1000;

% deler opp i venstre og høyre liste
for i = 1:length(x)
    if points(i,1) < 0 && points(i,2) < search_range % �nsker ikke � kutte liste mer, for out of range problem i l�kke
        leftpoints = [points(i,:);leftpoints];
    end
    if points(i,1) > 0 && points(i,2) < search_range
        rightpoints = [rightpoints;points(i,:)];
    end
end


%% Set thresholds and parameters. door list

% alt i milli. 
door_threshold = 60; % How big norm represents a door? Hvor stor topp på deriviert. Tune opp/ned.
door_distance = 200; % How close should the robot be to the door before is it denoted as detected? distanse fra første dørkarm
detect_door_left = false;
detect_door_right = false;
L_index = 0; % kun brukt for plotting
R_index = 0; % kun brukt for plotting

% [x,y,bol, bol] 3:bol=1=right bol=0=left, 4:bol=1=detected
doors = getdoors();

%%  Detect right doors
if ~isempty(nearby_door_right)
    
    first_doorframe_pos = [0,0];
    max_norm = 0;
    index = 0;
    % norm between every second point to reduce noise.  
    for n = 1:length(rightpoints(:,1)) - 2 
        % Calculate norm, OBS: benches and elevator could be detected
        norm_right(n) = norm(rightpoints(n,:) - rightpoints(n+2,:));
        % only interested in the first norm that passes threshold. this
        % is the doorframe
        if norm_right(n) > door_threshold
            max_norm = norm_right(n);
            index = n;
            first_doorframe_pos = [rightpoints(index,1),rightpoints(index,2)];
            break;
        end
    end
    
    % If the distance to the door is less than door_distance, we
    % want a DOOR event to occur, and list the door as detected
    if norm(first_doorframe_pos(2)) > 0 && norm(first_doorframe_pos(2)) < door_distance
        R_index = index; % used for plotting
        set_door_detected(nearby_door_right(8));
        detect_door_right = true; % SET GLOBAL RIGHT DOOR TO TRUE
    end
    
end

%%  Detect left doors
if ~isempty(nearby_door_left)
    first_doorframe_pos = [0,0];
    max_norm = 0;
    index = 0;
    
    % norm between every second point to reduce noise.  
    for n = 1:length(leftpoints(:,1)) - 2
        % Calculate norm, OBS: benches and elevator could be detected
        norm_right(n) = norm(leftpoints(n,:) - leftpoints(n+2,:));
        % only interested in the first norm that passes threshold. this
        % is the doorframe
        if norm_right(n) > door_threshold
            max_norm = norm_right(n);
            index = n;
            first_doorframe_pos = [leftpoints(index,1),leftpoints(index,2)];
            break;
        end
    end

    if norm(first_doorframe_pos(2)) > 0 && norm(first_doorframe_pos(2)) < door_distance
        L_index = index;
        set_door_detected(nearby_door_left(8));
        detect_door_left = true; % SET GLOBAL LEFT DOOR TO TRUE
    end
end

% Booleans that execute an event in cause of true
ans = [detect_door_right ,detect_door_left];

%% PLOTTING. Coment out when running

% for n = 1:length(rightpoints(:,1)) - 2 % norm between every second point to reduce noise.  
%     % Find all norms, OBS: benches and elevator could be detected
%     norm_right(n) = norm(rightpoints(n,:) - rightpoints(n+2,:));
% end
% 
% for n = 1:length(leftpoints(:,1)) - 2 % norm between every second point to reduce noise.  
%     % Find all norms, OBS: benches and elevator could be detected
%     norm_left(n) = norm(leftpoints(n,:) - leftpoints(n+2,:));
% end
% 
% 
% % Plot rangescan and doors found
% figure(1);
% plot(leftpoints(:,1),leftpoints(:,2))
% hold on
% plot(rightpoints(:,1),rightpoints(:,2))
% hold on
% if L_index ~=0
%     plot(leftpoints(L_index,1),leftpoints(L_index,2),'o')
%     hold on
% end
% if R_index ~=0
%     plot(rightpoints(R_index,1),rightpoints(R_index,2),'*')
% end
% hold off
% figure(3)
% plot(norm_left)
% figure(4)
% plot(norm_right)

end



