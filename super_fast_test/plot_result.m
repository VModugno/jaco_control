clear variables
close all
clc

deg = pi/180;

% ground truth
plot_subchain1 = [6];
plot_target_link{1} = plot_subchain1;
% reference parameters
taskspace_dim = 3;
plot_type = {'cartesian_x'};
plot_control_type = {'tracking'};
plot_type_of_traj = {'func'};
plot_traj = {'circular'};
plot_time_law = {'linear'};


%parameters first chains
plot_geom_parameters{1,1} = [0.3 0*deg,90*deg,0,0.4,0,0.4]; 
plot_time_struct.ti = 0;
plot_time_struct.tf = 20;
plot_time_struct.step = 0.001;
plot_dim_of_task{1,1}={[1;1;1]};



%% reference
% if type_of_task = sampled i have to specify the Time to reach the
% end of the trajectories that is equal to the simulation time
plot_reference = References(plot_target_link,plot_type,plot_control_type,plot_traj,plot_geom_parameters,plot_time_law,plot_time_struct,plot_dim_of_task,plot_type_of_traj);
plot_reference.BuildTrajs();


p_tot=[];
pd_tot = [];

for t=plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf
 
	[p_cur,pd_cur]=plot_reference.GetTraj(1,1,t);
	p_tot = [p_tot,p_cur];
   pd_tot = [pd_tot,pd_cur];
end

%% read result 

path_joints = 'joint_pos.mat';
path_cart = 'cart_pos.mat';
path_index = 'index.mat';
allpath=which('plot_result.m');
path=fileparts(allpath);

path_joints = strcat(path,'/',path_joints);
path_cart =   strcat(path,'/',path_cart);
path_index =  strcat(path,'/',path_index);

all_joints = load(path_joints,'-ascii');
all_cartesian = load(path_cart,'-ascii');
index = load(path_index,'-ascii');

figure
plot(all_joints);
grid on;
legend('q1','q2','q3','q4','q5','q6');
xlabel('t') % x-axis label
ylabel('deg') % y-axis label
figure
plot(all_cartesian(:,1:3));
grid on;
legend('X','Y','Z');
xlabel('t') % x-axis label
ylabel('meter') % y-axis label
figure
plot(all_cartesian(:,4:6));
grid on;
legend('roll','pitch','yaw');
xlabel('t') % x-axis label
ylabel('rad') % y-axis label
figure; hold on;
plot3(p_tot(1,:),p_tot(2,:),p_tot(3,:),'b');
plot3(all_cartesian(:,1),all_cartesian(:,2),all_cartesian(:,3),'r');
grid on;
xlabel('X') % x-axis label
ylabel('Y') % y-axis label
zlabel('Z')
legend('ground truth','actual trajectory');


% mean and average between the desired trajectory and the executed one 
number_of_sample = size(all_cartesian,1);
p_tot = p_tot';
p_tot = p_tot(index,:);
p_err = p_tot - all_cartesian(:,1:3);
p_mean_x = sum(p_err(:,1))/number_of_sample;
p_mean_y = sum(p_err(:,2))/number_of_sample;
p_mean_z = sum(p_err(:,3))/number_of_sample;

p_var_x =  var(p_err(:,1));
p_var_y =  var(p_err(:,2));
p_var_z =  var(p_err(:,3));

