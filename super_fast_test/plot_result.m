clear variables
close all
clc

allpath=which('plot_result.m');
path=fileparts(allpath);

%% loqd ground truth
load('ground_truth.mat');
p_ground=p;
%% read results 

path_joints = 'joint_pos.mat';
path_cart = 'cart_pos.mat';
path_tau = 'joint_tau';
path_index = 'index.mat';


path_joints = strcat(path,'/',path_joints);
path_cart =   strcat(path,'/',path_cart);
path_tau =   strcat(path,'/',path_tau);
path_index =  strcat(path,'/',path_index);

all_joints = load(path_joints,'-ascii');
all_cartesian = load(path_cart,'-ascii');
all_tau = load(path_tau,'-ascii');
index = load(path_index,'-ascii');

%% plot results
dim_tit=16;
dim_lab=14;
dim_leg=12;

tolSmall=15;
tolHigh=40;

figure
plot(index,all_joints);
grid on;
leg = legend('q1','q2','q3','q4','q5','q6');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','northeast');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('deg'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

figure
plot(index,all_tau);
grid on;
leg = legend('u1','u2','u3','u4','u5','u6');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','northeast');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('N*m'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

q_ground_to_plot = q_ground(index,:);

figure; 
for ijl = 1:6
    subplot(3,3,ijl)
    plot(index,all_joints(:,ijl),'b');
    hold on;
    plot(index,q_ground_to_plot(:,ijl),'r');
    grid on;
    name_q_exec = strcat('q',num2str(ijl));
    name_q_des = strcat('GTq',num2str(ijl));
    leg = legend(name_q_exec,name_q_des);
    set(leg,'FontSize',dim_leg,'Location','northeast');
    xlab=xlabel('t'); % x-axis label
    set(xlab,'FontSize',dim_lab,'Interpreter','latex');
    ylab=ylabel('deg'); % y-axis label
    set(ylab,'FontSize',dim_lab,'Interpreter','latex');
    %set(gca, 'Position', [0.1 0.1 0.3 0.85])
end

figure
plot(index,all_cartesian(:,1:3));
grid on;
legend('X','Y','Z');
set(leg,'FontSize',dim_leg,'Interpreter','latex','Location','best');
xlab=xlabel('t'); % x-axis label
set(xlab,'FontSize',dim_lab,'Interpreter','latex');
ylab=ylabel('meter'); % y-axis label
set(ylab,'FontSize',dim_lab,'Interpreter','latex');

figure
plot(index,all_cartesian(:,4:6));
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
p_ground = p_ground(index,1:3);
p_err = p_ground - all_cartesian(:,1:3);
p_mean_x = sum(p_err(:,1))/number_of_sample;
p_mean_y = sum(p_err(:,2))/number_of_sample;
p_mean_z = sum(p_err(:,3))/number_of_sample;

p_var_x =  var(p_err(:,1));
p_var_y =  var(p_err(:,2));
p_var_z =  var(p_err(:,3));

