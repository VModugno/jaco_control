clear variables
close all
clc

allpath=which('plot_result.m');
path=fileparts(allpath);

%% loqd ground truth
p_ground=[];
fid = fopen('cart_pos.txt');
tline = fgetl(fid);
while ischar(tline)
    tline = fgetl(fid); % in this way i cut out the first line
    if(tline == -1)
        disp('read position completed');
    else
        tlin = strsplit(tline);
        app=[];
        for i=1:size(tlin,2)
        app = [app,str2double(tlin{1,i})];
        end
        p_ground = [ p_ground;app];
    end 
end
%% read results 

path_joints = 'joint_pos.mat';
path_cart = 'cart_pos.mat';
path_index = 'index.mat';


path_joints = strcat(path,'/',path_joints);
path_cart =   strcat(path,'/',path_cart);
path_index =  strcat(path,'/',path_index);

all_joints = load(path_joints,'-ascii');
all_cartesian = load(path_cart,'-ascii');
index = load(path_index,'-ascii');

%% plot results
dim_tit=16;
dim_lab=14;
dim_leg=12;

tolSmall=15;
tolHigh=40;

figure
plot(all_joints);
grid on;
leg = legend('q1','q2','q3','q4','q5','q6');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','northeast');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('deg'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

figure
plot(all_cartesian(:,1:3));
grid on;
legend('X','Y','Z');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('meter'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

figure
plot(all_cartesian(:,4:6));
grid on;
legend('roll','pitch','yaw');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','Best');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('rad'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

figure; hold on;
plot3(p_ground(:,1),p_ground(:,2),p_ground(:,3),'b');
plot3(all_cartesian(:,1),all_cartesian(:,2),all_cartesian(:,3),'r');
plot3(p_ground(1,1),p_ground(1,2),p_ground(1,3),'go','MarkerFaceColor','g','MarkerSize',10); % plot starting point
plot3(p_ground(end,1),p_ground(end,2),p_ground(end,3),'ko','MarkerFaceColor','k','MarkerSize',10); % plot ending point
grid on;
xlab=xlabel('X'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('Y'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');
zlab=zlabel('Z');
set(zlab,'FontSize',dim_lab,'Interpreter','latex');
leg=legend('ground truth','actual trajectory');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','Best');

% mean and average between the desired trajectory and the executed one 
number_of_sample = size(all_cartesian,1);
p_ground = p_ground(index,:);
p_err = p_ground - all_cartesian(:,1:3);
p_mean_x = sum(p_err(:,1))/number_of_sample;
p_mean_y = sum(p_err(:,2))/number_of_sample;
p_mean_z = sum(p_err(:,3))/number_of_sample;

p_var_x =  var(p_err(:,1));
p_var_y =  var(p_err(:,2));
p_var_z =  var(p_err(:,3));

