clear all
clc

load ('TestTrack.mat');
%% Parameters
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;
%%
x0   =   287;
u0   =   5.0;
y0   =  -176;
v0   =   0.0;
psi0 =   2.0;
r0   =   0.0;
z0 = [x0, u0, y0, v0, psi0, r0]';
%%
bl = TestTrack.bl;       % Left Boundaries
br = TestTrack.br;       % Right Boundaries
cline = TestTrack.cline; % Center Line
theta = TestTrack.theta; % Center Line's Orientation
%%
%plot the track
plot(bl(1,:),bl(2,:),'r')
hold on
plot(br(1,:),br(2,:),'r')

%%
N=5000;
U=zeros(N,2);
Y=zeros(N,6);
T=zeros(1,N);
dt=0.01;
T=0:dt:dt*(N-1); %%check the length of T
%%
cline_plus1=[[0;0] cline];
cline247=[cline [0;0]];
ip=diag((cline_plus1-cline247)'*(cline_plus1-cline247));
ip=ip(2:end-1);
ip=ip.^(1/2);
max_distance=max(ip);


%%
Youtput=[];
Youtput(1,:)=z0;
cline=[cline cline(:,end) cline(:,end)];
% cline=[cline247 zeros(2,10)];
theta=[theta theta(end) theta(end) theta(end)];
T=[0 dt];
vdata=[];
for i=1:100000
    % solve Y using ode45, simulate model
%     options = odeset('MaxStep',0.01);
%     tspan=[T(i),T(i+1)];
    Uin=U(i,:);
%     Uin=U(i:i+1,:);
%     [~,Ytemp]=ode45(@(t,Y) dynamics_28(t,Y,Uin),[0 dt],Youtput(i,:),options); %check the arguments
%     [Ytemp,~]=forwardIntegrateControlInput(U(i,:),Youtput(i,:));
    %Solve for trajectory
    options = odeset('MaxStep',0.01);
    [t1,Ytemp]=ode45(@(t,x)bike(t,x,T,Uin),T,Youtput(i,:),options);
    Youtput(i+1,:)=Ytemp(end,:);
%     t1
    if Youtput(i+1,1)>=cline(1,246)
        break
    end
    % find the nearest point in cline
    x=Youtput(i+1,1);
    y=Youtput(i+1,3);
    xy=[x;y];
    innerproduct=diag((cline-xy)'*(cline-xy));
    [M,nearest_idx]=min(innerproduct);

    %compute current velocity
    dnear=(dot(Youtput(i+1,[1 3])-Youtput(i,[1 3]),Youtput(i+1,[1 3])-Youtput(i,[1 3])))^(1/2);
    v=dnear/dt;
    vdata(i,1)=v;

    % to get Kp
    dl=(vecnorm(xy-bl(:,nearest_idx)))^(1/2);
    dr=(vecnorm(xy-br(:,nearest_idx)))^(1/2);
    % dl/dr-1
    part=(dl/dr)/((dl/dr)+1)-1/2; % -left +right
    Kp=1+abs((dl/dr)/((dl/dr)+1)-1/2); 

%     U(i+1,1)=theta(nearest_idx+1)-Youtput(i,5);
    ca=1;
    d2=(cline(:,nearest_idx+1+ca)-cline(:,nearest_idx+ca))'*(cline(:,nearest_idx+1+ca)-cline(:,nearest_idx+ca));
    d=d2^(1/2);
    if d<3
        c1=5; c2=100; c3=10; c0=1;
    else
        c1=3; c2=6; c3=0; c0=1;
    end
%     c1=3; c2=6; c0=1;%weight constant
    U(i+1,1)=1*((c3*theta(nearest_idx+3)+c2*theta(nearest_idx+2)+c1*theta(nearest_idx+1)+theta(nearest_idx))/(c2+c1+c0)-Youtput(i,5))+0.5*part;
    if U(i+1,1)<-0.5
        U(i+1,1)=-0.5;
    elseif U(i+1,1)>0.5
        U(i+1,1)=0.5;
    else
        U(i+1,1)=U(i+1,1);
    end

%     change speed in advance
%     ca=1;
%     d2=(cline(:,nearest_idx+1+ca)-cline(:,nearest_idx+ca))'*(cline(:,nearest_idx+1+ca)-cline(:,nearest_idx+ca));
%     d=d2^(1/2);
    d=d+0.1; % in case d=0
    mFx=4200;
    if nearest_idx<3 | nearest_idx>200
        U(i+1,2)=mFx*(d/max_distance)*(50/v);
    elseif nearest_idx>95 && nearest_idx<150
        U(i+1,2)=500*(d/max_distance)*(10/v);
    else
        U(i+1,2)=1000*(d/max_distance)*(10/v);
    end

    if U(i+1,2)<-mFx
        U(i+1,2)=-mFx;
    elseif U(i+1,2)>mFx
        U(i+1,2)=mFx;
    else
        U(i+1,2)=U(i+1,2);
    end

    if abs(theta(nearest_idx+1)-theta(nearest_idx))>0.1
        U(i+1,2)=0;
    end
%     U(i+1,2)=600*(d/max_distance);
    i
%     nearest_idx
end
%%
% (dot(cline(246)-cline(245),cline(246)-cline(245)))^(1/2)
% cline(246)-cline(245)
% forwardIntegrateControlInput(U(1,:),Youtput(1,:))
%%
plot(Youtput(:,1),Youtput(:,3),'b');
figure(2)
plot(vdata)

%%


function [dx] = dynamics_28(t,x,Uin)
    %constants
    Nw=2;
    f=0.01;
    Iz=2667;
    a=1.35;
    b=1.45;
    By=0.27;
    Cy=1.2;
    Dy=0.7;
    Ey=-1.6;
    Shy=0;
    Svy=0;
    m=1400;
    g=9.806;


    delta_f = Uin(1);
    F_x = Uin(2); %% have changed the order to the same as the project
    
    %slip angle functions in degrees
    a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
    a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

    %Nonlinear Tire Dynamics
    phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
    phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

    F_zf=b/(a+b)*m*g;
    F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

    F_zr=a/(a+b)*m*g;
    F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

    F_total=sqrt((Nw*F_x)^2+(F_yr^2));
    F_max=0.7*m*g;

    if F_total>F_max

        F_x=F_max/F_total*F_x;

        F_yr=F_max/F_total*F_yr;
    end

    %vehicle dynamics
    dx = [x(2)*cos(x(5))-x(4)*sin(x(5));...
              (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
              x(2)*sin(x(5))+x(4)*cos(x(5));...
              (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
              x(6);...
              (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end




function dzdt=bike(t,x,T,U)
%constants
Nw=2;
f=0.01;
Iz=2667;
a=1.35;
b=1.45;
By=0.27;
Cy=1.2;
Dy=0.7;
Ey=-1.6;
Shy=0;
Svy=0;
m=1400;
g=9.806;


%generate input functions
% delta_f=interp1(T,U(:,1),t,'previous','extrap');
% F_x=interp1(T,U(:,2),t,'previous','extrap');
delta_f = U(1);
F_x = U(2);


%slip angle functions in degrees
a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));

%Nonlinear Tire Dynamics
phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));

F_zf=b/(a+b)*m*g;
F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;

F_zr=a/(a+b)*m*g;
F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;

F_total=sqrt((Nw*F_x)^2+(F_yr^2));
F_max=0.7*m*g;

if F_total>F_max
    
    F_x=F_max/F_total*F_x;
  
    F_yr=F_max/F_total*F_yr;
end

%vehicle dynamics
dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
          (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
          x(2)*sin(x(5))+x(4)*cos(x(5));...
          (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
          x(6);...
          (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
end