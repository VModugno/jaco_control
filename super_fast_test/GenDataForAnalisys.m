% generate analisys file from .txt
clear all
close all
clc

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

%% save data

save('ground_truth.mat','p','pd');

