clear all
clc
load TestTrack.mat
x=TestTrack.cline;
% d2=zeros(101,1);
% for i=1:100
%     d2(i,1)=(x(1,i+1)-x(1,i))^2+(x(2,i+1)-x(2,i))^2;
% end
syms dt
vc=10;
deltat=zeros(size(x,2),1);
T=zeros(size(x,2),1);
s=zeros(size(x,2),1);
for i=1:size(x,2)-1
    dotx=(x(1,i+1)-x(1,i))/dt;
    doty=(x(2,i+1)-x(2,i))/dt;
    eqn=(dotx^2+doty^2)^(0.5)==vc;
    delta=solve(eqn,dt);
    deltat(i,1)=abs(double(delta(1,:)));
    T(i+1,1)=T(i,1)+deltat(i,1);
    distance(i,1)=((x(1,i+1)-x(1,i))^2+(x(2,i+1)-x(2,i))^2)^(1/2);
    s(i+1,1)=s(i,1)+distance(i,1);
end

filename='TestTrack.mat';
save(filename,'deltat','T','s','-append')