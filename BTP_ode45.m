clc; clear all;
%% Input - Initial State of Vehicles
n=input('Enter Number of UAVs: ');
xyO=zeros(n,3);
xyO_1=zeros(n,3);
disp('Enter initial global posture info (x, y, theta) for the N vehicles');
for i=1:n 
    for j=1:3
        if j==1
            w1=sprintf('Enter x (for vehicle %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for vehicle %d) : ',i);
        end
        if j==3
            w1=sprintf('Enter theta [in degree](for vehicle %d) : ',i);
        end
        xyO(i,j)=input(w1);
        xyO_1(i,j)=xyO(i,j);
    end
end
%% Input - Initial State of Targets
clc;
m=input('Enter Number of Targets: ');
xy=zeros(m,5);
xy_1=zeros(m,5);

disp('Enter initial position and moving speed and time until movement stops (x, y, v(x), v(y), t) for the m targets');
for i=1:m 
    for j=1:5
        if j==1
            w1=sprintf('Enter x (for Target %d) : ',i);
        end
        if j==2
            w1=sprintf('Enter y (for Target %d) : ',i);
        end
        if j==3
            w1=sprintf('Enter speed_x (for Target %d) : ',i);
        end
        if j==4
            w1=sprintf('Enter speed_y (for Target %d) : ',i);
        end
        if j==5
            w1=sprintf('Enter total time of movement (for Target %d) : ',i);
        end
        xy(i,j)=input(w1) ;
        xy_1(i,j)=xy(i,j);
    end
end
%% Radii value generator block
clc;
r=zeros(n,1);
for i=1:n
    r(i)=reqmod(i,m);
end
%% Controller Synthesis and Analysis
%% Design Parameters 
clc;
a=zeros(n,1);
b=zeros(n,1);
c=zeros(n,1);
del=zeros(n,1);
ref=zeros(n,1);
ref=1-ref;
kw=1;
nud=input('Enter number of vehicles that are not able to detect their target: ');

if nud==0
    if mod(n,2)==0
        epsilon=0.1;
    end
    if mod(n,2)==1
        epsilon=0;
    end
    a_star=-1+epsilon;
    d_v=ceil(n/2)-1;
    phi_v=2*d_v*pi/n;
    kv=abs((1-a_star*cos(phi_v))/(a_star*sin(phi_v)));
end

if nud~=0
    epsilon=0.1;
    a_star=1-epsilon;
    d_v=n-1;
    phi_v=2*d_v*pi/n;
    kv=abs((1-a_star*cos(phi_v))/(a_star*sin(phi_v)));
end

for i=1:nud
    undec=input('Enter the vehicle number that is not able to detect its target: ');
    ref(undec)=0;
end

for i=1:n
    
    % for undectected vehicles
    if ref(i)==0
        %s(i)=r(i+1);
        b(i)=0;
        r(i)=r(reqmod(i+1,n))/a_star;
        del(i)=1/r(i);
        a(i)=1;
        c(i)=0;
    end

    % for dectected vehicles
    if ref(i)==1
        b(i)=1;
        del(i)=1/r(i);
        a(i)=(r(i)*a_star)/r(reqmod(i+1,n));
        c(i)=1-a(i)-b(i);
    end   
end
%% Over Time Interval
dt=1;
xyO_1(:,3)=deg2rad(xyO_1(:,3));
clc;
f_x=zeros(1000,n);
f_y=zeros(1000,n);
xyO=xyO_1;
xy=xy_1;
p_val=1;
for time=1:1:1000
    if time>0
        f_x(p_val,:)=xyO(:,1);
        f_y(p_val,:)=xyO(:,2);
        p_val=p_val+1;
    end
    phi=zeros(n,1);
    RO=zeros(2,2,n);
    u_p=zeros(2,n);
    u_b=zeros(2,n);
    u_c=zeros(2,n);
    d=zeros(2,n);
    v_x=zeros(n,1);
    v_y=zeros(n,1);
    w=zeros(n,1);
    xyO_v=zeros(n,3);
    u_p_v=zeros(2,n);
    Op=zeros(2,1);
    
    %%%%% phi measurement%%%%%%%
    for i=1:n
        phi(i)=atan2((xyO(reqmod(i+1,n),2)-xy(reqmod(i+1,m),2)),(xyO(reqmod(i+1,n),1)-xy(reqmod(i+1,m),1)))-atan2((xyO(reqmod(i,n),2)-xy(reqmod(i,m),2)),(xyO(reqmod(i,n),1)-xy(reqmod(i,m),1)));
    end
    
    %%%%% Relative Measurements %%%%%%
    for i=1:n
        %RO(:,:,i)=[cos(deg2rad(xyO(i,3))),sin(deg2rad(xyO(i,3)));-1*sin(deg2rad(xyO(i,3))),cos(deg2rad(xyO(i,3)))];  
        RO(:,:,i)=[cos(xyO(i,3)) sin(xyO(i,3));-1*sin(xyO(i,3)) cos(xyO(i,3))];  
        u_p(:,i)=RO(:,:,i)*[xyO(reqmod(i+1,n),1)-xyO(i,1);xyO(reqmod(i+1,n),2)-xyO(i,2)];
        u_b(:,i)=RO(:,:,i)*[xy(reqmod(i,m),1)-xyO(i,1);xy(reqmod(i,m),2)-xyO(i,2)];
        u_c(:,i)=RO(:,:,i)*[xy(reqmod(i+1,m),1)-xyO(i,1);xy(reqmod(i+1,m),2)-xyO(i,2)];
        d(:,i)=-1*(a(i)+c(i))*RO(:,:,i)*[xy(reqmod(i+1,m),1)-xy(reqmod(i,m),1);xy(reqmod(i+1,m),2)-xy(reqmod(i,m),2)];
    end
%    disp(RO);;

    %%%%%% Virtual Vehicle %%%%%%%%
    for i=1:n
        xyO_v(i,:)=[(xyO(i,1)-xy(reqmod(i,m),1))/r(i),(xyO(i,2)-xy(reqmod(i,m),2))/r(i),xyO(i,3)];
    end
    
    for i=1:n
        u_p_v(:,i)=RO(:,:,i)*((a_star*xyO_v(reqmod(i+1,n),1:2)-xyO_v(i,1:2)).'); 
    end
   
    for i=1:n
       Op=[kv,0;0,kw]*u_p_v(:,i);
       
       v_x(i)=(r(i)*Op(1,1))*cos(xyO(i,3));
       v_y(i)=(r(i)*Op(1,1))*sin(xyO(i,3));

       %v_x(i)=(r(i)*Op(1,1))*cos(deg2rad(xyO(i,3)));
       %v_y(i)=(r(i)*Op(1,1))*sin(deg2rad(xyO(i,3)));
       
       w(i)=Op(2,1);
    end
    %disp(w);
%end
    for i=1:m
       if time<(xy(i,5)+1) 
           xy(i,1)=xy(i,1)+xy(i,3)*dt;
           xy(i,2)=xy(i,2)+xy(i,4)*dt;
       end
    end
    
    tspan=[0 1];
    for i=1:n
       x0=xyO(i,1);
       [t,x]=ode45(@(t,x) v_x(i)*t,tspan,x0);
       xyO(i,1)=x(end);
       y0=xyO(i,2);
       [t,y]=ode45(@(t,y) v_y(i)*t,tspan,y0);
       xyO(i,2)=y(end);
       theta0=xyO(i,3);
       [t,theta]=ode45(@(t,theta) w(i)*t,tspan,theta0);
       xyO(i,3)=theta(end);
    end
   
end
plot(xy(:,1),xy(:,2),'^'); 
hold on;
for i=1:n
    %figure;
    plot(f_x(:,i),f_y(:,i));
    plot(f_x(end,i),f_y(end,i),'o');
    grid on;
end
%% Functions
function z = reqmod(p,q)
    if mod(p,q)==0
        z=q;
    else
        z=mod(p,q);
    end
end
