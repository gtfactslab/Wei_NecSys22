function [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,v_0,w)
% safety verification of closing the node v_0.
% input:
% G -- Graph
% R -- routes
% S -- schedules (a set of departure times of UAVs)
% v_0 -- the node being closed
% output: 
% safe_set -- |G.V|x1 vector that indicates safety of each node
% C_check{v} -- [start, end, #rerouted2v, #unaffected2v, c_v >= #rerouted2v + #unaffected2v?]
N = length(S);
safe_set = zeros(length(G.V),1);
num_set = [];
% Compute occupation time for each UAV at each node according to current schedule
occupation = cell(length(G.V),1);
for i = 1:length(G.V)
    occupation{i} = zeros(N,3);
end
for i = 1:N
    dpT = S(i);
    route = Demand_R(i);    % find the required route
    route_v = R{route,5};   % nodes along the route
    for k_v = 2:length(route_v)
        v = route_v(k_v);
        occupation{v}(i,1) = i;
        occupation{v}(i,2) = dpT+R{route,4}(1,k_v-1);
        occupation{v}(i,3) = dpT+R{route,4}(2,k_v-1)+w;
    end
end

% Verify safty of schedule when all nodes are functioning (0-node close)
if isempty(v_0)
    % check the beginning of every section for the aggregated of UAVs at
    % that time
    num_set = zeros(N,length(G.V));     % number of UAVs at v after k'th UAV landing at the node
    for v = 1:length(G.V)
        if ismember(v,G.ori)
            continue;
        end
        for k = 1:N
            land_early = occupation{v}(k,2);  % earliest arrival time at v
            % count the number of UAVs whose staying time (M) include land_early
            land_inclusion = (occupation{v}(:,2) <= land_early) & (occupation{v}(:,3) > land_early);
            num_set(k,v) = sum(land_inclusion,1);   
        end
    end
    C_check = [];
    safe_set(:) = G.C>=max(num_set',[],2);
else    % v_0 is closed

    % compute G'.Delta with the node disabled
    v0_ind = find(v_0 == G.V,1);
    Delta_close = G.Delta;
    for k = 1:size(G.Delta,2)
        if Delta_close(v0_ind,k) == 1
            Delta_close(v0_ind,:) = -Delta_close(v0_ind,:);
        end
    end
    
    % Find rerouted UAVs at each node
    N_R = size(R,1);    % # of routes
    affected_R = zeros(N_R,1);  % affected routes
    affected_v_mat = zeros(N_R,length(G.V));    % affected nodes
    affected_e_mat = zeros(N_R,size(G.Delta,2));    % affected 
    % reroute2v{v} = [start time, end time, # of UAVs rerouted to v (possibly)]
    reroute2v = cell(length(G.V),1);
    % Time [start, end] needs to check at v
    check_reroute = cell(length(G.V),1);
    for v = 1:length(G.V)
        check_reroute{v} = zeros(N,2);
    end

    for r = 1:N_R
        demand_ind = (Demand_R ==r);    % UAVs through R
        route_v = R{r,5};   % nodes along the route
        % find affected routes, nodes and edges
        if ismember(v_0,route_v)    
            affected_R(r) = 1; 
            position_in_route = find(route_v == v_0,1);
            affected_v_mat(r,1:position_in_route) = route_v(1:position_in_route);
            affected_e_mat(r,1:position_in_route-1) = R{r,5}(1:position_in_route-1);
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % consider cases for position_in_route = 1 or 2            
            if position_in_route == 2
                % the UAVs are drived back to origin
                continue;
            end
            % position_in_route > 2  
            for k_v = 2:position_in_route-2
                tail = route_v(k_v-1);
                head = route_v(k_v);
                if k_v ==2  % tail is the origin
                    check_reroute{head}(demand_ind,1) = S(demand_ind);                    
                else
                    check_reroute{head}(demand_ind,1) = occupation{tail}(demand_ind,2)+w; % inf{M}+w

                end
                check_reroute{head}(demand_ind,2) = occupation{head}(demand_ind,3); % inf{M}+w
            end
            % merge 2 consecutive intervals for nodes connected to v_0
            tail_1 = route_v(position_in_route-2);
            head_1 = route_v(position_in_route-1);
            check_reroute{head_1}(demand_ind,1) = occupation{tail_1}(demand_ind,2)+w;
            check_reroute{head_1}(demand_ind,2) = occupation{v_0}(demand_ind,3)-w; 
            
        end
    end
    
    % compute reroute2v
    for v = 1:length(G.V)
        bounds = unique(check_reroute{v});
        bounds = bounds(bounds~=0);
        if isempty(bounds)==1
            continue;
        end
        reroute2v{v} = zeros(length(bounds),3);
        reroute2v{v}(1,:) = [0,bounds(1),0];
        for k = 1:length(bounds)-1
            inclusion_ind = (check_reroute{v}(:,1) <= bounds(k)) & (check_reroute{v}(:,2) > bounds(k));
            num_reroute = sum(inclusion_ind,1);
            reroute2v{v}(k+1,:) = [bounds(k),bounds(k+1),num_reroute];          
        end
    end
    
        
    % time-varying index set (unaffected UAVs)
    % Remain = {[start, end], unaffected indices of UAVs}
    check_affected = unique(occupation{v_0}(:,2));  % sorted
    check_affected = check_affected(check_affected~=0);
    Remain = cell(length(check_affected),2);
    Remain{1,1} = [0, check_affected(1)];
    Remain{1,2} = ones(N,1);
    for k = 1:length(check_affected)-1
        checking = check_affected(k);
        affected = (occupation{v_0}(:,2) <= checking) & (occupation{v_0}(:,3) > checking);
        remaining = 1-affected;
        Remain{k+1,1} = [checking, check_affected(k+1)];
        Remain{k+1,2} = remaining;
    end    
    % Find max of unaffected UAVs at a time for each node v
    % unaffected2v{v} = [start, end, # of unaffected UAVs]
    unaffected2v = cell(length(G.V),1);
    for v = 1:length(G.V)
        bounds_M = unique(occupation{v}(:,2:3));
        bounds_M = bounds_M(bounds_M~=0);
        if isempty(bounds_M)
            continue;
        end
        bounds_all = unique([bounds_M;check_affected]);   
        unaffected2v{v} = zeros(length(bounds_all),3);
        unaffected2v{v}(1,:) = [0,bounds_all(1),0]; % initialize with 0 and revise later
        for k = 1: length(bounds_all)-1
            checking = bounds_all(k);
            remain_find = find(check_affected > checking,1);
            if isempty(remain_find)
                RI = ones(N,1);
            else
                RI = Remain{remain_find,2}; % index vector of unaffected UAVs                
            end
            larger_ind = find(bounds_M>checking,1);
            inclusion_ind = (occupation{v}(:,2).*RI <= checking) & (occupation{v}(:,3).*RI > checking);
            temp_max = sum(inclusion_ind,1);
            for k2 = larger_ind:length(bounds_M)
                inclusion_ind = (occupation{v}(:,2).*RI <= bounds_M(k2)) & (occupation{v}(:,3).*RI > bounds_M(k2));
                temp_max = max(temp_max, sum(inclusion_ind,1));
            end
            unaffected2v{v}(k+1,:) = [bounds_all(k),bounds_all(k+1),temp_max];
        end
        
        
        
    end
    
    % check: c_v >= #rerouted2v + #unaffected2v
    % C_check{v} = [start, end, #rerouted2v, #unaffected2v, c_v >= #rerouted2v + #unaffected2v?]
    C_check = cell(length(G.V),1);
    safe_set = zeros(length(G.V),1);
    for v = 1: length(G.V)
        if isempty(unaffected2v{v}) & isempty(reroute2v{v})
            safe_set(v) = 1;
            C_check{v} = [];
            continue;
        elseif isempty(unaffected2v{v})
            bounds_t = reroute2v{v}(:,2);
        elseif isempty(reroute2v{v})
            safe_set(v) = 1;
            C_check{v} = [];
            continue;        
        else
            bounds_t = unique([unaffected2v{v}(:,2);reroute2v{v}(:,2)]);
        end
        C_check{v} = zeros(length(bounds_t),5);
        C_check{v}(1,:) = [0,bounds_t(1),0,max(unaffected2v{v}(:,3)),1];
        for k = 1:length(bounds_t)-1
            checking = bounds_t(k);
            reroute2v_ind = (reroute2v{v}(:,1) <= checking ) & (reroute2v{v}(:,2) > checking);
            if isempty(unaffected2v{v})
                unaffected2v_ind = 0;
            else
                unaffected2v_ind = (unaffected2v{v}(:,1) <= checking ) & (unaffected2v{v}(:,2) > checking); 
            end
                       
            % because of partition, both contain at most one "1"         
            if sum(reroute2v_ind)==0 & sum(unaffected2v_ind)==0
                reroute2v_end = 0;
                unaffected2v_end = 0;
            elseif sum(reroute2v_ind)==0
                reroute2v_end = 0;
                unaffected2v_end = unaffected2v{v}(unaffected2v_ind,3);
                if unaffected2v_end == 0 
                    unaffected2v_end = max(unaffected2v{v}(find(unaffected2v_ind,1):end,3));
                end
            elseif sum(unaffected2v_ind)==0
                reroute2v_end =reroute2v{v}(reroute2v_ind,3);
                unaffected2v_end = 0;
            else
                reroute2v_end =reroute2v{v}(reroute2v_ind,3);
                unaffected2v_end = unaffected2v{v}(unaffected2v_ind,3);
                if unaffected2v_end == 0 
                    unaffected2v_end = max(unaffected2v{v}(find(unaffected2v_ind,1):end,3));
                end
            end
            
            C_check{v}(k+1,1:4) = [bounds_t(k), bounds_t(k+1), reroute2v_end, unaffected2v_end];
            C_check{v}(k+1,5) = (sum(C_check{v}(k+1,3:4),2) <=G.C(v));
        end
        safe_set(v) = ~ismember(0,C_check{v}(:,5));
    end
    % all affected nodes and edges
%     affected_v = unique(affected_v_mat);
%     affected_v = affected_v(affected_v~=0);    
%     affected_e = unique(affected_e_mat);
%     affected_e = affected_e(affected_e~=0);    
    
end










end