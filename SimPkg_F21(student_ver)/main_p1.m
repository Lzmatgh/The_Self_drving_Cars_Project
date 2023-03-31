clear all; close all; clc;
tic

% load U_Left.mat
% 
% ROB535_ControlProject_part1_input = ROB599_ControlsProject_part1_input;
% save('ROB535_ControlProject_part1_Team26.mat','ROB535_ControlProject_part1_input');

%%
load TestTrack.mat
load U1206_tune_velocity_std117.mat

[Y,T] = forwardIntegrateControlInput(U);

%% Results
info = getTrajectoryInfo(Y,U)
%%
figure
    hold all
    plot(TestTrack.bl(1,:),TestTrack.bl(2,:),'k')
    plot(TestTrack.br(1,:),TestTrack.br(2,:),'k')    
    plot(info.Y(:,1),info.Y(:,3),'r')
    
toc

%%
% plot(Y(:,1),Y(:,3),'g')
max(U(:,1))
max(U(:,2))
    
    

