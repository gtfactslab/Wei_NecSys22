%% Network parameters
% Graph
G.V = [1;2;3;4;5;6;7];    % vertices
% G.C = [100;5;4;2;4;3;5];    %capacity of vertices
% G.C = [100;50;40;20;40;30;50];
G.C = [100;8;4;2;4;3;5];
G.ori = [1];
G.dest = [6,7];

G.Delta = [-1,0,0,0,0,0,0;...
           1,-1,-1,0,0,0,0;...
           0,1,0,-1,-1,0,0;...
           0,0,1,0,0,-1,0;...
           0,0,0,0,1,1,-1;...
           0,0,0,1,0,0,0;...
           0,0,0,0,0,0,1];      % Incidence matrix

X_l = [8;3;2;4;5;1;3];
X_u = [10;5;6;8;6;5;6];

G.W_l = diag(X_l);          % Edge weights: upperbound
G.W_u = diag(X_u);          % Edge weights: lowerbound


% Routes
R = cell(3,4);

R{1,1} = 1;             % path number
R{1,2} = [1,2,4];       % edges of the path 
R{1,3} = zeros(1,length(R{1,2}));       % aggregated uncertainty at each dest
R{1,4} = zeros(2,length(R{1,2}));       % aggregated travel time at each dest

R{2,1} = 2;         
R{2,2} = [1,2,5,7];
R{2,3} = zeros(1,length(R{2,2}));
R{2,4} = zeros(2,length(R{2,2}));

R{3,1} = 3;
R{3,2} = [1,3,6,7];
R{3,3} = zeros(1,length(R{3,2}));
R{3,4} = zeros(2,length(R{3,2}));

% for each routes, the nodes along the route
R2V = cell(size(R,1),2);
for k_R = 1:size(R,1)
    R2V{k_R,1} = R{k_R,1};    
    route = R{k_R,2};
    R2V{k_R,2} = zeros(1,length(route)+1);
    R2V{k_R,2}(1) = find(G.Delta(:,route(1))<0,1);
    for k = 1:length(route)
         R2V{k_R,2}(k+1) = find(G.Delta(:,route(k))>0,1);
    end
end

R(:,5) = R2V(:,2);

V2R = cell(length(G.V),2);
for v = 1:length(G.V)
    V2R{v,1} = v;
    V2R{v,2} = zeros(1,size(R,1));
    for k_R = 1:size(R,1)
        if sum(ismember(R2V{k_R,2},v))
             V2R{v,2}(k_R) = R{k_R,1};
        end
    end
    V2R{v,2} = V2R{v,2}(V2R{v,2}~=0);
end

% waiting time
w = 1;

% find all converging nodes and corresponding routes
% [the node that has route passing through, corresponding route] = time
% needs to reserve 
for k_R = 1:size(R,1)
    route = R{k_R,2};
    a_agr = 0;
    b_agr = 0;
    for k_2 = 1:length(route)
        if k_2 == 1
            a_agr = a_agr + G.W_l(route(k_2),route(k_2));
            b_agr = b_agr + G.W_u(route(k_2),route(k_2));
        else
            a_agr = a_agr + G.W_l(route(k_2),route(k_2))+w;
            b_agr = b_agr + G.W_u(route(k_2),route(k_2))+w;
        end
        R{k_R,3}(k_2) = b_agr - a_agr;
        R{k_R,4}(1,k_2) =a_agr;     % earliest arrival of k_2'th dest(node)
        R{k_R,4}(2,k_2) =b_agr;     % latest arrival of k_2'th dest(node)     
    end
end

%% Case study parameters
v_0 = 5;    % closing node
% N = 1000;     % number of demands
% max_dpT = 800;   % maximum of departure time (when generating)

% S = zeros(N,1);     % schedules 
% Demand_R = zeros(N,1);  % routes for demands
% S = round(rand(N,1)*max_dpT,1);
% Demand_R = randi(size(R,1),N,1);   % choose routes randomly (uniform distr)

% save('sche_ex11.mat','S');
% save('demand_ex11.mat','Demand_R');

% 20-UAV example
% load('sche_ex0.mat','S');
% load('demand_ex0.mat','Demand_R');

% 100-UAV example
% load('sche_ex1.mat','S');
% load('demand_ex1.mat','Demand_R');

% maximum size 1000

%% Computation/Verification (1 backup node)
% Verify that S satisfies the safety constraint when all nodes are
% available
% [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,[],w);
 
% Verify that S satisfies the safety constraint when v_0 is disabled
% [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,v_0,w);

% % check safety for the 11 datasets when node 5 is closed
% max_file = 11;
% verify_time = zeros(max_file,1);
% size_arr = zeros(max_file,1);
% for i =1:max_file
%     name_sche = ['sche_ex',num2str(i-1),'.mat'];
%     name_demand = ['demand_ex',num2str(i-1),'.mat'];
%     load(name_sche,'S');
%     load(name_demand,'Demand_R');
%     size_arr(i) = size(S,1);
%     tStart = tic; 
%     [safe_set, C_check] = verification_1_node(G,R,Demand_R,S,v_0,w);
%     tEnd = toc(tStart);
%     verify_time(i) = tEnd;
% end
% save('verification','verify_time');
% save('size_info','size_arr');

%% plots

% plot N vs computation time
load('verification','verify_time');
load('size_info','size_arr');

figure
set(gcf,'Units','Inches')
set(gcf,'Position',[4 4 4 3])
set(gca,'units','inches')
set(gcf, 'PaperUnits','inches');        
set(gcf, 'PaperSize', [4 3]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 4 3]);
% curve
plot(size_arr, verify_time,'Linewidth',2);
hold on
% dots
plot(size_arr, verify_time,'ko','Linewidth',2);
ylabel('Computation Time (second)')
xlabel('Schedule Size')
% saveas(gcf,'size_time','pdf')
saveas(gcf,'size_time','fig')


% plot verification of size 20
load('sche_ex0.mat','S');
load('demand_ex0.mat','Demand_R');
[safe_set, C_check] = verification_1_node(G,R,Demand_R,S,v_0,w);
% 
% 
% 
v_check = 2;
size_bound = size(C_check{v_check},1);
x = C_check{v_check}(:,1);  % x_start of bar
dx = C_check{v_check}(:,2)-C_check{v_check}(:,1);     % width of bar
y1 = zeros(1,size_bound);  % y_start of rerouted bar (always 0)
dy1 = C_check{v_check}(:,3);  % height of rerouted bar
y2 = dy1;  % y_start of unaffected bar
dy2 =  C_check{v_check}(:,4);  % height of unaffected bar


figure
set(gcf,'Units','Inches')
set(gcf,'Position',[4 4 4 3])
set(gca,'units','inches')
set(gcf, 'PaperUnits','inches');        
set(gcf, 'PaperSize', [4 3]);
set(gcf, 'PaperPositionMode', 'manual');
set(gcf, 'PaperPosition', [0 0 4 3]);
hold on
for ii=1:length(x)
    rectangle('position',[x(ii) y1(ii) dx(ii) dy1(ii)],'LineWidth',1.5) % N_v
    hold on
    rectangle('position',[x(ii) y2(ii) dx(ii) dy2(ii)],'FaceColor','c') % N_R
    hold on
end
plot([0 max(C_check{v_check}(:,2))+1],[G.C(v_check),G.C(v_check)],'b-.','Linewidth',2)
axis([0 max(C_check{v_check}(:,2))+1 0 max([max(sum(C_check{v_check}(:,3:4),2)),G.C(v_check)])+4])

hold on
text(42,13,'N_{v_2}','Color','black','FontSize',12)
rectangle('position',[48 12.8 7 1],'LineWidth',1.5)

text(42,11.5,'N_R','Color','black','FontSize',12)
rectangle('position',[48 11 7 1],'FaceColor','c')

text(42,9.5,'C_{v_2}','Color','black','FontSize',12)
plot([48 55],[10 10], 'b-.','Linewidth',2)
hold off
ylabel('Number of UAVs')
xlabel('t_c (minute)')
% saveas(gcf,'case_study20','pdf')
saveas(gcf,'case_study20','fig')


