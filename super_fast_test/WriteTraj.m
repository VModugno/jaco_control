clear variables;
close all;
clc;

deg = pi/180;

namefile = {'cart_pos.txt','cart_vel.txt','start_joint_pos.txt','start_cart_pos.txt'};
write_traj = false;
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


%% plot scene
plot_bot = MdlJaco();
%plot_bot = MdlLBR4p();

p_tot=[];
pd_tot = [];

for t=plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf
 
	[p_cur,pd_cur]=plot_reference.GetTraj(1,1,t);
	p_tot = [p_tot,p_cur];
   pd_tot = [pd_tot,pd_cur];
end

orientation = [0 , pi , 0];
T = eye(4);
Rx = rotx(orientation(1));
Ry = roty(orientation(2));
Rz = rotz(orientation(3));
T(1:3,1:3) = Ry;
T(1:3,4) = p_tot(:,1);

joint_pose = plot_bot.ikunc(T);
start_cartesian_pos = [p_tot(:,1)' , orientation]; 


plot3(p_tot(1,1:end),p_tot(2,1:end),p_tot(3,1:end));
cur_joint = joint_pose;
all_joint = cur_joint;
index = 1;
for  t=plot_time_struct.ti:plot_time_struct.step:plot_time_struct.tf

   J =plot_bot.jacob0(cur_joint);
   J = J(1:3,1:end);
   qd = pinv(J)*pd_tot(:,index);
   cur_joint = cur_joint + (qd*plot_time_struct.step)';
   all_joint = [all_joint ; cur_joint];
   index = index + 1;
   
end

%% write on file 
if (write_traj)
   p_tot = p_tot';
   WriteFF(p_tot,taskspace_dim,namefile{1});
   pd_tot = pd_tot';
   WriteFF(pd_tot,taskspace_dim,namefile{2});
   WriteFF(joint_pose,6,namefile{3});
   WriteFF(start_cartesian_pos,6,namefile{4});
end

% test of the trajectory
fps = 200;
plot_bot.plot(all_joint,'fps',fps);
   

