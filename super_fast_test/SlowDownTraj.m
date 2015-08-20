%slow down trajectory by interpolation
function SlowDownTraj
clear all 
close all
clc

h = 0.001; % time step
current_final = 160; % duration in second before the interpolation
new_final_time = 320; % duration in second of the new trajectory

allpath=which('plot_result.m');
path=fileparts(allpath);
%% read data from file
p=[];
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
        p = [ p ;app];
    end
end

pd=[];
fid = fopen('cart_vel.txt');
tline = fgetl(fid);
while ischar(tline)
    tline = fgetl(fid); % in this way i cut out the first line
    if(tline == -1)
        disp('read velocity completed');
    else
        tlin = strsplit(tline);
        app=[];
        for i=1:size(tlin,2)
        app = [app,str2double(tlin{1,i})];
        end
        pd = [ pd ;app];
    end
end

%% interpolation
new_p=Interpolation(p,h,current_final,new_final_time);
new_pd=Interpolation(pd,h,current_final,new_final_time);

%%DEBUG
figure; hold on;
plot3(p(:,1),p(:,2),p(:,3),'b');
plot3(new_p(:,1),new_p(:,2),new_p(:,3),'r');
figure; hold on;
plot3(pd(:,1),pd(:,2),pd(:,3),'b');
plot3(new_pd(:,1),new_pd(:,2),new_pd(:,3),'r');

%% write ff back
WriteFF(new_p,3,strcat(path,'/','cart_pos.txt'));
WriteFF(new_pd,3,strcat(path,'/','cart_vel.txt'));

end



% i obtain the interpolated torque and in this way i can compute mean and
% variance
function new_vec=Interpolation(vec,step,current_final,new_final_time)
   
   mult_factor = new_final_time/current_final;
   slower_step = step*mult_factor;
   cur_time = 0:slower_step:new_final_time;
   interp_time = 0:step:new_final_time;
   new_vec = interp1(cur_time,vec,interp_time,'nearest');
 
    
end