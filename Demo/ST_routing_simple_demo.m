clear all
close all
clc

%% Add CASADI package in the matlab path here:



%% load a example crowd model , OR replace it with other crowd model
load SP_model_2.mat
P_mu = SP_model_1.mean;
P_sigma = SP_model_1.covariance; 
figure(1)
for i=1:length(P_mu(:,1,:))
scatter3(P_mu(1,:,i)',P_mu(2,:,i)',zeros(length(P_mu(:,:,1)),1)+i,'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);hold on
end



%% Map environment
map.XYMAX=20; % map size 
% obstacle setting
nObstacle = 2; 
obstacle=round(rand([nObstacle,2])*map.XYMAX*0.8); % index of obstacle
obstacle=obstacle+1;
% obstacle = [25,10; 10,25; 25,40; 40,25];
obstacle_radius=1;
safe_d=1;


%% Initilization 
start = [18,18];
goal  = [3,3];
validpath = [];
v_n =1.8; % Travel speed (m/s)
t = 1;% SP model time interval
endPoint2goal=inf; 
detect_goal_node=start;
it=0;
end_node=[inf,inf];
time_start = 0;
T_index = time_start;
time_start = 0;
cross_list=[];
n = 2;
inf_width = 2; 
am = 100;
selected_node =start;
selected_node1 =start;
selected_node2 =start;

%% Optimization

tic

 while endPoint2goal >= 2

opti = casadi.Opti();

wp = opti.variable(n,3);             

% COST1 : guide the path towards goal
d_e2g = sqrt((wp(2,1)-goal(1,1))^2 +(wp(2,2)-goal(1,2))^2) ;

% COST2: extra travel time caused by potential local pedestrain avoiding
index_list=[];

if it < length(P_mu(1,1,:))
    for j=1:length(P_mu)       
            d2p = sqrt((wp(2,1)-P_mu(1,j,it+1))^2 +(wp(2,2)-P_mu(2,j,it+1))^2);
            is_inside = -1 ./ ( 1+exp(-100*d2p+100*inf_width) )+1              
            index_list = [index_list,is_inside];
    end 
end

T_extra = sum(index_list);

d_i =  ((wp(1,1)-wp(2,1))^2 +(wp(1,2)-wp(2,2))^2) ;
d_ii = sqrt(d_i);
b = 1/v_n;
v = 1/(6.9*sum(index_list)+b);

T_all = d_i / v^2; % norm travel time from point to point

t = t^2;
opti.subject_to( T_all == t )


%fixed time axis 
if it>=1   
   T_index = end_node(1,3);  
else    
   T_index = 0; 
end
T_index = [T_index;t];
opti.subject_to (wp(2,3) == sum(T_index));          


% Obstacle avoiding
 dobs=0;
cross_list = [];
for j=1:length(obstacle(:,1))
     dobs1= sqrt((wp(2,1)-obstacle(j,1))^2 + (wp(2,2)-obstacle(j,2))^2) ;
     opti.subject_to(  dobs1  >= (obstacle_radius+safe_d) ); 
   
end


% Cost Function
delta = 10^12 ;
alpha2 = 10^20 ;
cost = d_e2g - delta * v 

% Optimization objective
opti.minimize(  cost  );

% Constraints

% opti.subject_to( 0 <= wp <= map.XYMAX );
 if it >= 1
    opti.subject_to( wp(1,1)== end_node(1,1) );
    opti.subject_to( wp(1,2)== end_node(1,2));
    opti.subject_to( wp(1,3) == end_node(1,3) );
 else
    opti.subject_to( wp(1,1)==start(1) );
    opti.subject_to( wp(1,2)==start(2) );
    opti.subject_to( wp(1,3) == time_start );
 end

% Solution extract
opti.solver('ipopt');
sol = opti.solve();
path = sol.value(wp);
T_extra = sol.value(T_extra);
T_all = sol.value(T_all);
d_ii = sol.value(d_ii);
v = sol.value(v);
cross_list = sol.value(cross_list);
opti.debug.show_infeasibilities();

% Prepare for next iteration
detect_goal_node=[path(end,1),path(end,2)];
it = it + 1;
end_node=path(2,:);
selected_node=path(1,:);
validpath = [validpath;selected_node];
endPoint2goal = sqrt(  (   detect_goal_node(1,1)-goal(1,1)  )^2 +  (  detect_goal_node(1,2)-goal(1,2) )^2   );   
figure(1)
plot3(path(:,1),path(:,2),path(:,3),'-','color',rand(1,3),'LineWidth',2.5);hold on
plot3(start(1),start(2),time_start,'gO','LineWidth',2);hold on

for i2=1:length(obstacle(:,1))
  
  r=obstacle_radius;
  center=obstacle(i2,:);
  height=map.XYMAX+1;
  color = [0.25, 0.25, 0.25]; % color of each cyl
  nSides = 100;   % number of "sides" of the cyl
  plotCylinderWithCaps(r,center,height,nSides,color);

end
xlabel X(m)
ylabel Y(m)
zlabel('Time(s)')
xlim([0 map.XYMAX+1])
ylim([0 map.XYMAX+1])
zlim([0 map.XYMAX+1])
grid on

end

toc

figure(2)
for i=1:length(P_mu(:,1,:))
scatter3(P_mu(1,:,i)',P_mu(2,:,i)',zeros(length(P_mu(:,:,1)),1)+i,'MarkerEdgeColor','k','MarkerFaceColor',[0 .75 .75]);hold on
end


for i2=1:length(obstacle(:,1))
  
  r=obstacle_radius;
  center=obstacle(i2,:);
  height=map.XYMAX+1;
  color = [0.25, 0.25, 0.25]; % color of each cyl
  nSides = 100;   % number of "sides" of the cyl
  plotCylinderWithCaps(r,center,height,nSides,color);

end

tf = norm(validpath(end,1:2)-goal)/v_n + validpath(end,3);
goal = [goal,tf];
validpath = [validpath;goal];
validpath(1,:,:)=[];
start = [validpath(1,1,1),validpath(1,2,1),validpath(1,3,1)]
validpath=[start;validpath];
plot3(validpath(:,1),validpath(:,2),validpath(:,3),'-c','LineWidth',2);hold on
plot3(validpath(:,1),validpath(:,2),validpath(:,3),'b--o','LineWidth',1);hold on
% plot3(start(1),start(2),time_start,'gO','LineWidth',2);hold on
plot3(validpath(1,1,1),validpath(1,2,1),validpath(1,3,1),'gO','LineWidth',2);hold on
plot3(goal(1),goal(2),validpath(end,3),'rO','LineWidth',2);hold on
xlabel X(m)
ylabel Y(m)
zlabel('Time(s)')
xlim([0 map.XYMAX+1])
ylim([0 map.XYMAX+1])
if length(validpath(:,1))> map.XYMAX
zlim([0 length(validpath(:,1))])
else
zlim([0 map.XYMAX+1])
end
grid on

figure(3)
g = 2;
x1 = 0:g:map.XYMAX;
x2 = 0:g:map.XYMAX;
x_size = (map.XYMAX / 0.5) + 1;
[X1,X2] = meshgrid(x1,x2);
X = [X1(:) X2(:)];
History_GMM = zeros(length(x1),length(x2),length(P_mu(:,1,:)))
for k=1:length(P_mu(:,1,:))
    
   gm = gmdistribution(P_mu(:,:,k)',P_sigma(:,:,:,k));
   y_gm = pdf(gm,X);
   y_gm = y_gm./max(y_gm);
   y_gm = reshape(y_gm,length(x2),length(x1));
   History_GMM(:,:,k) = y_gm;
end

g=2;
T = length(P_mu(:,1,:)); % step number
x = 0:g:map.XYMAX;   % first dimension independent variable
y = 0:g:map.XYMAX;   % second dimension independent variable
z = 0:1:T-1;   % third dimension independent variable
[X, Y, Z] = meshgrid(x, y, z);  % form the 3D grid
cut =[0:2:30];
slice(X, Y, Z,  History_GMM,[], [],cut); hold on
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
plot3(validpath(:,1),validpath(:,2),validpath(:,3),'-r','LineWidth',2);hold on
% plot3(start(1),start(2),time_start,'gO','LineWidth',2);hold on
plot3(validpath(1,1,1),validpath(1,2,1),validpath(1,3,1),'gO','LineWidth',2);hold on
plot3(goal(1),goal(2),validpath(end,3),'yO','LineWidth',2);hold on

% plot obstacles
for i2=1:length(obstacle(:,1))
  
  r=obstacle_radius;
  center=obstacle(i2,:);
  height=length(validpath(:,1));
  color = [0.4940 0.1840 0.5560]; % color of each cyl
  nSides = 100;   % number of "sides" of the cyl
  plotCylinderWithCaps(r,center,height,nSides,color);

end
xlabel X(m)
ylabel Y(m)
zlabel('Time(s)')
alpha(0.4) 
xlim([0 map.XYMAX+1])
ylim([0 map.XYMAX+1])
if length(validpath(:,1))> map.XYMAX
zlim([0 length(validpath(:,1))])
else
zlim([0 map.XYMAX+1])
end
grid on
% colormap(cool)
N = 256; % new CT length
CT0 = [1 1 1; 0 0.85 0.85; 0 1 1; 0 0 1; 0 1 0; 1 1 0; 1 0 0];
x = [0 48 84 128 168 199 256]/256;
xf = linspace(0,1,N);
CT = interp1(x,CT0,xf,'linear','extrap'); hold on
colormap(CT)
colorbar


function [h1, h2, h3] = plotCylinderWithCaps(r,cnt,height,nSides,color)
[X,Y,Z] = cylinder(r,nSides);
X = X + cnt(1); 
Y = Y + cnt(2); 
Z = Z * height; 
h1 = surf(X,Y,Z,'facecolor',color,'LineStyle','none');
h2 = fill3(X(1,:),Y(1,:),Z(1,:),color);
h3 = fill3(X(2,:),Y(2,:),Z(2,:),color);
end  %only needed if this is within a script

function [crosses] = line_crosses_circle(A, B, center, radius)
 % Calculate the squared distances from A and B to the center
    dist_sq_A = sum((A - center).^2);
    dist_sq_B = sum((B - center).^2);
    
    % Calculate the squared distance between the two points A and B
    dist_sq_AB = sum((B - A).^2);
    
    % Calculate the expression to check if the line crosses the circle
    cross_check = (dist_sq_A - radius^2) * (dist_sq_B - radius^2) * dist_sq_AB;

    % If the result is negative, it means the line crosses the circle
    crosses = (cross_check < 0);
end



