clear all;close all;clc
% Load SP_model data
load('SP_model_thor.mat');

% Initialize figure for the spatial-temporal plot


coordinates =0;
time = 0;

savePedPos = struct([]);
% Loop through unique time values
for t = 1: length(SP_model)
    figure(1)
    coordinates = SP_model(t).Coordinates;
    time = SP_model(t).Time;    
    x = coordinates(:, 1);
    y = coordinates(:, 2);
%     x = coordinates(:, 1);
%     y = 30-coordinates(:, 2);
    pedc = [x,y]';
    savePedPos(t).a= pedc;
    z = zeros(length(coordinates),1)+time;
    scatter3(x, y, z,'MarkerEdgeColor','k','MarkerFaceColor',[0 0.75 0.75],'LineWidth',0.1);hold on % Use '.' for individual points, customize as needed
    %scatter3(x, y, z,'b+');hold on % Use '.' for individual points, customize as needed
end
% 
% P_sigma = zeros(2,length())

% Customize the plot appearance
xlabel('X Coordinate');
ylabel('Y Coordinate');
zlabel('Time (seconds)');
xlim([0 50]);
ylim([0 50]);
axis equal
title('3D Spatial-Temporal Plot of Pedestrian Movements');

grid on; % Add a grid to the plot
hold off;

% plot GMM
figure(2)

savePedPos(t).a

% surface map setting
map.XYMAX = 100; % map scale
g = 2;
x1 = 0:g:map.XYMAX;
x2 = 0:g:map.XYMAX;
x_size = (map.XYMAX / 0.5) + 1;
[X1,X2] = meshgrid(x1,x2);
X = [X1(:) X2(:)];
History_GMM = zeros(length(x1),length(x2),length(savePedPos))

P_sigma_fake = [0.3388, 0; 0, 0.2039];


for k=1:length(savePedPos)
    
   P_sigma = zeros(2,2,length(savePedPos(k).a));
   
   for j = 1:length(savePedPos(k).a)
   
   P_sigma(:,:,j) = P_sigma_fake;
   
   end
   
   gm = gmdistribution(savePedPos(k).a',P_sigma);
   y_gm = pdf(gm,X);
   y_gm = y_gm./max(y_gm);
   y_gm = reshape(y_gm,length(x2),length(x1));
   History_GMM(:,:,k) = y_gm;
   
end


g=2;
T = length(savePedPos) % step number


x = 0:g:map.XYMAX;   % first dimension independent variable
y = 0:g:map.XYMAX;   % second dimension independent variable

z = 0:1:T-1;   % third dimension independent variable
[X, Y, Z] = meshgrid(x, y, z);  % form the 3D grid





cut =[0:10:length(savePedPos)];
slice(X, Y, Z,  History_GMM,[], [],cut); hold on
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
alpha(0.5) 
N = 256; % new CT length
CT0 = [1 1 1; 0 0.85 0.85; 0 1 1; 0 0 1; 0 1 0; 1 1 0; 1 0 0];
x = [0 48 84 128 168 199 256]/256;
xf = linspace(0,1,N);
CT = interp1(x,CT0,xf,'linear','extrap'); hold on
colormap(CT)
colorbar

% Customize the plot appearance
xlabel('X Coordinate');
ylabel('Y Coordinate');
zlabel('Time (seconds)');
xlim([0 50]);  % Set x-axis limits
ylim([0 50]); % Set y-axis limits
zlim([0 T-1]); % Optionally, set z-axis limits

% axis equal
title('3D Spatial-Temporal Plot of Pedestrian Movements');







