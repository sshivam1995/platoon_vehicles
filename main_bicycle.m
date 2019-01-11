close all
dt=0.01; tf=40; t=0:dt:tf;
N=size(t,2); T=0.5; delta_T=0.002*T;
alpha1=30;
alpha2=30;
alpha3=20;
alpha4=10;

distance1_2=10;
distance2_3=10;
distance3_4=10;

delta=0.3;
brake_T=0;

tester=zeros(1,N);

Nbar=(N-1)*38/40;

%% Car parameters 
lr=1.738; lf=1.105; m=2050; Iz=3344; Calpha=[-57500;-92500];      
accln_sat=5;
angle_sat=pi/4;


%% Car 1
% Z is xdot,ydot,phidot, X, Y, phi
Z=zeros(6,N);
Z0=[25;0;0;0;0;1.13];
Z(:,1)=[0.1;0;0.1;0.1;0.0;0];
%Z(:,1)=Z0;

%% Car 2
% Z is xdot,ydot,phidot, X, Y, phi
Zbar1=zeros(6,N);
%Z0=[25;0;0;0;0;1.13];
Zbar1(:,1)=[0.1;0;0.1;-10;0.0;0];

%% Car 3
% Z is xdot,ydot,phidot, X, Y, phi
Zbar2=zeros(6,N);
%Z0=[25;0;0;0;0;1.13];
Zbar2(:,1)=[0.1;0;0.1;-20;0.0;0];


%% Car 4
% Z is xdot,ydot,phidot, X, Y, phi
Zbar3=zeros(6,N);
%Z0=[25;0;0;0;0;1.13];
Zbar3(:,1)=[0.1;0;0.1;-30;0.0;0];




%% Velocity profile for car 1 reference
%define reference r(1) is x_pos, r(2) is y_pos;
V=zeros(2,N);M=N-1;
% for i=1:M/4
%    V(1,i)=10*i*4/M; 
%    V(2,i)=20*i*4/M;
% end
% 
% for i=M/4+1:2*M/5
%    V(:,i)=V(:,M/4);
% end
% 
% for i=2*M/5+1:M/2 
%    V(1,i)= V(1,M/4)-(i-2*M/5)*0.6*V(1,M/4)*10/M;                    % 40% decrease in speed x on cornering
%    V(2,i)= V(2,M/4)-(i-2*M/5)*V(2,M/4)*10/M;                      % 100% decrease in speed y on cornering
% end    
%     
% V(:,(N+1)/2)= V(:,M/2);
% 
% for i=1:M/2
%     V(1,N-i+1)=V(1,i);
%     V(2,N-i+1)=-V(2,i);
% end    

for i=1:M/4
    V(:,i)=[20*i/(M/4)*cos((pi/2)*i/(M/4)); 20*i/(M/4)*sin((pi/2)*i/(M/4))];
end

for i=1:M/8
    V(:,i+M/4)=[20*sin((pi/6)*i/(M/8));20*cos((pi/6)*i/(M/8))];
end

for i=1:M/8
    V(:,i+3*M/8)=[V(1,3*M/8)-i*0.5*V(1,3*M/8)/(M/8); V(2,3*M/8)-i*V(2,3*M/8)/(M/8)];    %V(1,M/4)-(i-2*M/5)*0.6*V(1,M/4)*10/M
end

V(1,M/2+1)=V(1,M/2);
V(2,M/2+1)=0;

for i=1:M/2
     V(1,N-i+1)=V(1,i);
     V(2,N-i+1)=-V(2,i);
end 

norm_Vdot=zeros(1,N);

for i=1:M/4
    norm_Vdot(i)=20*4/tf;
end

for i=1:M/8
    norm_Vdot(i+M/4)=0;
end

for i=1:M/8
    V_net=norm(V(:,i+3*M/8));
    Vxdot=-V(1,3*M/8)*4/tf;
    Vydot=-V(2,3*M/8)*8/tf;
    norm_Vdot(i+3*M/8)=(V(1,i+3*M/8)*Vxdot+V(2,i+3*M/8)*Vydot)/V_net;
end    

for i=1:M/2
   norm_Vdot(M-i)=-norm_Vdot(i); 
end    

%% reference for car 1 from velocity

r0=[0;0];
r=zeros(2,N);

r(:,1)=r0;

for i=2:N
    r(:,i)=r(:,i-1)+V(:,i)*dt;
end

r_extend=zeros(2,N+200);

for i=1:200
    r_extend(:,i)=[-40+0.2*i;0];
end

for i=1:N
    r_extend(:,i+200)=r(:,i);
end    
%% Reference shifted in future time
ref=r; 
for i=1:N-T/dt
   ref(:,i)=r(:,i+T/dt); 
end    
for i=N-T/dt+1:N
    ref(:,i)=ref(:,N-T/dt);
end

ref_bar1=zeros(2,N);
ref_bar2=zeros(2,N);
ref_bar3=zeros(2,N);

ref_bar1=Zbar1(4:5,1);
ref_bar2=Zbar2(4:5,1);
ref_bar3=Zbar3(4:5,1);
%% Input for cars 
% u is delta_f (front wheel angle wrt body) and a (acceleration)
u=zeros(2,N);
u(:,1)=[0;0];
ubar1=zeros(2,N);
ubar1(:,1)=[0;0];

ubar2=zeros(2,N);
ubar2(:,1)=[0;0];

ubar3=zeros(2,N);
ubar3(:,1)=[0;0];


udot=zeros(2,N);
udotbar=zeros(2,N);
%% Track  vehicle motion
pos_tracker=zeros(2,N);
pos_trackerbar1=zeros(2,N);
pos_trackerbar2=zeros(2,N);
pos_trackerbar3=zeros(2,N);

pos_tracker(:,1)=Z(4:5,1);
pos_trackerbar1(:,1)=Zbar1(4:5,1);
pos_trackerbar2(:,1)=Zbar2(4:5,1);
pos_trackerbar3(:,1)=Zbar3(4:5,1);


% derivative of z at all time instances
zdot=zeros(6,N);
zbardot1=zeros(6,N);
zbardot2=zeros(6,N);
zbardot3=zeros(6,N);

g_u=zeros(2,N);
g_u_prime=zeros(2,2,N);
g_u_bar1=zeros(2,N);
g_u_prime_bar1=zeros(2,2,N);

g_u_bar2=zeros(2,N);
g_u_prime_bar2=zeros(2,2,N);

g_u_bar3=zeros(2,N);
g_u_prime_bar3=zeros(2,2,N);

%% Main loop from 0 to tf

for i=2:N 
    if mod(i,1000)==0
        i
    end
    
    zdot(:,i)=dzdt(Z(:,i-1),u(:,i-1),lr,lf,m,Iz,Calpha);
    zbardot1(:,i)=dzdt(Zbar1(:,i-1),ubar1(:,i-1),lr,lf,m,Iz,Calpha);
    zbardot2(:,i)=dzdt(Zbar2(:,i-1),ubar2(:,i-1),lr,lf,m,Iz,Calpha);
    zbardot3(:,i)=dzdt(Zbar3(:,i-1),ubar3(:,i-1),lr,lf,m,Iz,Calpha);
    
  
%     for k=1:6  
%         if abs(zdot(k,i))>1
%             zdot(k,i)=zdot(k,i)/abs(zdot(k,i)); 
%         end
%     end    
     
    Z(:,i)=Z(:,i-1)+zdot(:,i)*dt;  
    Zbar1(:,i)=Zbar1(:,i-1)+zbardot1(:,i)*dt;  
    Zbar2(:,i)=Zbar2(:,i-1)+zbardot2(:,i)*dt;  
    Zbar3(:,i)=Zbar3(:,i-1)+zbardot3(:,i)*dt;  
    
    pos_tracker(:,i)=[Z(4,i);Z(5,i)];
    pos_trackerbar1(:,i)=[Zbar1(4,i);Zbar1(5,i)];   
    pos_trackerbar2(:,i)=[Zbar2(4,i);Zbar2(5,i)];   
    pos_trackerbar3(:,i)=[Zbar3(4,i);Zbar3(5,i)];   
    
    %Future prediction
    [g_u(:,i),g_u_prime(:,:,i)]=g_rt(Z(:,i-1),u(:,i-1), T, delta_T, lr,lf,Calpha,m,Iz);
    [g_u_bar1(:,i),g_u_prime_bar1(:,:,i)]=g_rt(Zbar1(:,i-1),ubar1(:,i-1), T, delta_T, lr,lf,Calpha,m,Iz);
    [g_u_bar2(:,i),g_u_prime_bar2(:,:,i)]=g_rt(Zbar2(:,i-1),ubar2(:,i-1), T, delta_T, lr,lf,Calpha,m,Iz);
    [g_u_bar3(:,i),g_u_prime_bar3(:,:,i)]=g_rt(Zbar3(:,i-1),ubar3(:,i-1), T, delta_T, lr,lf,Calpha,m,Iz);
   
 %% reference for follower 1
 
    v_leader=(pos_tracker(:,i)-pos_tracker(:,i-1))/dt;  
    
    distance1_2=10+norm(v_leader*brake_T);
    
    if norm(v_leader)>0
        ref_bar1(:,i)=pos_tracker(:,i)+v_leader*T-distance1_2*v_leader/norm(v_leader);
    end

    [ref_bar1(:,i),dist,t_a]=distance2curve(r_extend',ref_bar1(:,i)');
    
    if norm(ref_bar1(:,i)-ref_bar1(:,i-1))>delta
       ref_bar1(:,i)=ref_bar1(:,i-1)+delta*(ref_bar1(:,i)-ref_bar1(:,i-1))/norm(ref_bar1(:,i)-ref_bar1(:,i-1)); 
    end    
    
    
%% reference for follower 2

    v_leader=(pos_trackerbar1(:,i)-pos_trackerbar1(:,i-1))/dt;
    
    distance2_3=10+norm(v_leader*brake_T);
    
    if norm(v_leader)>0
        ref_bar2(:,i)=pos_trackerbar1(:,i)+v_leader*T-distance2_3*v_leader/norm(v_leader);
    end
    
    [ref_bar2(:,i),dist,t_a]=distance2curve(r_extend',ref_bar2(:,i)');

    
    if norm(ref_bar2(:,i)-ref_bar2(:,i-1))>delta
       ref_bar2(:,i)=ref_bar2(:,i-1)+delta*(ref_bar2(:,i)-ref_bar2(:,i-1))/norm(ref_bar2(:,i)-ref_bar2(:,i-1)); 
    end    
    
%% reference for follower 3

    v_leader=(pos_trackerbar2(:,i)-pos_trackerbar2(:,i-1))/dt;   
    
    distance3_4=10+norm(v_leader*brake_T);
    
    if norm(v_leader)>0
        ref_bar3(:,i)=pos_trackerbar2(:,i)+v_leader*T-distance3_4*v_leader/norm(v_leader);
    end
    
    [ref_bar3(:,i),dist,t_a]=distance2curve(r_extend',ref_bar3(:,i)');
    
    if norm(ref_bar3(:,i)-ref_bar3(:,i-1))>delta
       ref_bar3(:,i)=ref_bar3(:,i-1)+delta*(ref_bar3(:,i)-ref_bar3(:,i-1))/norm(ref_bar3(:,i)-ref_bar3(:,i-1)); 
    end    
        
%% input computation

%     if norm((ref(:,i)-ref(:,i-1)))>0
%         ref_bar(:,i)=ref(:,i)-(ref(:,i)-ref(:,i-1))/norm((ref(:,i)-ref(:,i-1)))*distance;
%         tester(i)=1;
%     else
%         ref_bar(:,i)=ref_bar(:,i-1);
%         tester(i)=0;
%     end
    
    u(:,i)=u(:,i-1)+g_u_prime(:,:,i)\(ref(:,i)-g_u(:,i))*alpha1*dt; 
    ubar1(:,i)=ubar1(:,i-1)+g_u_prime_bar1(:,:,i)\(ref_bar1(:,i)-g_u_bar1(:,i))*alpha2*dt; 
    ubar2(:,i)=ubar2(:,i-1)+g_u_prime_bar2(:,:,i)\(ref_bar2(:,i)-g_u_bar2(:,i))*alpha3*dt; 
    ubar3(:,i)=ubar3(:,i-1)+g_u_prime_bar3(:,:,i)\(ref_bar3(:,i)-g_u_bar3(:,i))*alpha4*dt; 
    
    if i>N-T/(dt)
        %delta=u(:,N-T/(dt))/(T/(dt));
        %u(:,i)=u(:,i-1)-delta;
        u(:,i)=u(:,i-1);
        ubar1(:,i)=ubar1(:,i-1);
    end    
    
    
    %% Saturations
    
    if abs(u(1,i))>angle_sat
        u(1,i)=u(1,i)*angle_sat/abs(u(1,i));
    end    
    
    if abs(u(2,i))>accln_sat
        u(2,i)=u(2,i)*accln_sat/abs(u(2,i));
    end
    
    if abs(ubar1(1,i))>angle_sat
        ubar1(1,i)=ubar1(1,i)*angle_sat/abs(ubar1(1,i));
    end    
    
    if abs(ubar1(2,i))>accln_sat
        ubar1(2,i)=ubar1(2,i)*accln_sat/abs(ubar1(2,i));
    end
    
    if abs(ubar2(1,i))>angle_sat
        ubar2(1,i)=ubar2(1,i)*angle_sat/abs(ubar2(1,i));
    end    
    
    if abs(ubar2(2,i))>accln_sat
        ubar2(2,i)=ubar2(2,i)*accln_sat/abs(ubar2(2,i));
    end

    if abs(ubar3(1,i))>angle_sat
        ubar3(1,i)=ubar3(1,i)*angle_sat/abs(ubar3(1,i));
    end    
    
    if abs(ubar3(2,i))>accln_sat
        ubar3(2,i)=ubar3(2,i)*accln_sat/abs(ubar3(2,i));
    end
    
    
    udot(:,i-1)=(u(:,i)-u(:,i-1))/dt;
    udotbar(:,i-1)=(ubar1(:,i)-ubar1(:,i-1))/dt;
end    
  
g_u(:,1)=g_u(:,2);
g_u_bar1(:,1)=g_u_bar1(:,2);
g_u_bar2(:,1)=g_u_bar2(:,2);
g_u_bar3(:,1)=g_u_bar3(:,2);

%% r-g

figure (6)

for i=1:Nbar
    control_error1(i)=norm(ref(:,i)-g_u(:,i));
    control_error2(i)=norm(ref_bar1(:,i)-g_u_bar1(:,i));
    control_error3(i)=norm(ref_bar2(:,i)-g_u_bar2(:,i));
    control_error4(i)=norm(ref_bar3(:,i)-g_u_bar3(:,i));
end
plot(t(1:Nbar),control_error1,'LineWidth',1.5)
hold on
plot(t(1:Nbar),control_error2,'LineWidth',1.5)
plot(t(1:Nbar),control_error3,'LineWidth',1.5)
plot(t(1:Nbar),control_error4,'LineWidth',1.5)


  x1=xlabel('$Time~[s]$');
 y1=ylabel('Control Error $[m]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')
 
leg1=legend('Car 1','Car 2','Car 3','Car 4');
 set(leg1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Control error vs time')

hold off

pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('control_error','-dsvg','-r0')

%% inter-agent distance
figure (1)
for i=1:N
d12(i)=norm(pos_tracker(:,i)-pos_trackerbar1(:,i));
d23(i)=norm(pos_trackerbar1(:,i)-pos_trackerbar2(:,i));
d34(i)=norm(pos_trackerbar2(:,i)-pos_trackerbar3(:,i));
end
plot (t(1:Nbar),d12(1:Nbar),'LineWidth',1.5)
hold on 
plot (t(1:Nbar),d23(1:Nbar),'LineWidth',1.5)
plot (t(1:Nbar),d34(1:Nbar),'LineWidth',1.5)

 x1=xlabel('Time$~[s]$');
 y1=ylabel('Inter-Vehicle distance$~[m]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')

leg1=legend('$d_{12}$','$d_{23}$','$d_{34}$');
 set(leg1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Inter-agent distance vs time')

hold off

pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('d_ij','-dsvg','-r0')
% % Plot of accln    
% figure (1)
% plot(t,u(2,:))
% accl_r_t=zeros(1,N);
% vec_speed=vecnorm(V);
% hold on
% plot(t, ubar1(2,:))
% for i=1:M/4
%     accl_r_t(1,i)=20/(tf/4);
% end   
% 
% for i=M/4+1:3*M/8
%     accl_r_t(1,i)=0;
% end
% V_ini=V(:,3*M/8);
% for i=3*M/8+1:M/2
%     
%     accl_r_t(1,i)=-(V(1,i)*V_ini(1)/(2*tf/8)+V(2,i)*V_ini(2)/(tf/8))/vec_speed(1,i);
% end
% 
% accl_r_t(M/2+1)=0;
% 
% for i=1:M/2
%    accl_r_t(1,N+1-i)=-accl_r_t(1,i); 
% end    
% plot(t,accl_r_t)
% ylim([-6 6])
% title('longitudinal acceleration vs time');
% legend('actual accleration leader','actual acceleration follower', 'r(t) acceleration');

% % Plot of jerk
% figure (2)
% plot(t,udot(2,:))
% hold on 
% plot(t,udotbar(2,:))
% legend('jerk leader','jerk follower');
% title('Longitudinal jerk vs time');
% 
% % PLot of tracking error norm
% figure (3)
% error=pos_tracker-r;   
% 
% norm_error=vecnorm(error);
% 
% plot (t,norm_error);
% title('tracking error norm (leader) vs time');

%% Lateral error

figure (2)

[close_point1,lateral_error_norm1,arc]=distance2curve(r_extend',pos_tracker');
[close_point2,lateral_error_norm2,arc]=distance2curve(r_extend',pos_trackerbar1');
[close_point3,lateral_error_norm3,arc]=distance2curve(r_extend',pos_trackerbar2');
[close_point4,lateral_error_norm4,arc]=distance2curve(r_extend',pos_trackerbar3');

 

plot(t(1:Nbar),lateral_error_norm1(1:Nbar),'LineWidth',1.5)
hold on
plot(t(1:Nbar),lateral_error_norm2(1:Nbar),'LineWidth',1.5)
plot(t(1:Nbar),lateral_error_norm3(1:Nbar),'LineWidth',1.5)
plot(t(1:Nbar),lateral_error_norm4(1:Nbar),'LineWidth',1.5)

leg1=legend('Car 1','Car 2','Car 3','Car 4');
 set(leg1,'Interpreter','latex')
 
  x1=xlabel('Time$~[s]$');
 y1=ylabel('Lateral Error$~[m]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Lateral error vs time')

hold off

pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('Lat_error','-dsvg','-r0')



%% input

figure (4)

for i=1:Nbar
   a1(i)=u(2,i);
   a2(i)=ubar1(2,i);
   a3(i)=ubar2(2,i);
   a4(i)=ubar3(2,i);
end    

plot(t(1:Nbar),a1,'LineWidth',1.5);
hold on
plot(t(1:Nbar),a2,'LineWidth',1.5);
plot(t(1:Nbar),a3,'LineWidth',1.5);
plot(t(1:Nbar),a4,'LineWidth',1.5);
plot(t(1:Nbar),norm_Vdot(1:Nbar),'LineWidth',1.5);

  x1=xlabel('$Time~[s]$');
 y1=ylabel('$a_l~[m/s^2]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')
 
leg1=legend('Car 1','Car 2','Car 3','Car 4','ref accl.');
 set(leg1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Acceleration vs time')

hold off

pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('accln','-dsvg','-r0')
% 
% 

%% head error
figure(5)
head_des1=zeros(2,Nbar);
head_des2=zeros(2,Nbar);
head_des3=zeros(2,Nbar);
head_des4=zeros(2,Nbar);


for i=1:Nbar
    
    if mod(i,200)==0
        i
    end
    
    min1=1; min2=1; min3=1; min4=1;
    min_index1=1; min_index2=1; min_index3=1; min_index4=1;

    
    for j=1:size(r_extend,2)
        if norm(close_point1(i,:)'-r_extend(:,j))<min1
            min1=norm(close_point1(i,:)'-r_extend(:,j));
            min_index1=j;             
        end
        
        if norm(close_point2(i,:)'-r_extend(:,j))<min2
            min2=norm(close_point2(i,:)'-r_extend(:,j));
            min_index2=j;
        end
        
        if norm(close_point3(i,:)'-r_extend(:,j))<min3
            min3=norm(close_point3(i,:)'-r_extend(:,j));
            min_index3=j;

        end
        
        if norm(close_point4(i,:)'-r_extend(:,j))<min4
            min4=norm(close_point4(i,:)'-r_extend(:,j));
            min_index4=j;
            
        end
    end  
    
    head_des1(:,i)=r_extend(:,min_index1+1)-r_extend(:,min_index1);    
    head_des2(:,i)=r_extend(:,min_index2+1)-r_extend(:,min_index2);
    head_des3(:,i)=r_extend(:,min_index3+1)-r_extend(:,min_index3);
    head_des4(:,i)=r_extend(:,min_index4+1)-r_extend(:,min_index4);
    
    ang_des1(i)=mod(atan2(head_des1(2,i),head_des1(1,i)),2*pi);
    ang_des2(i)=mod(atan2(head_des2(2,i),head_des2(1,i)),2*pi);
    ang_des3(i)=mod(atan2(head_des3(2,i),head_des3(1,i)),2*pi);
    ang_des4(i)=mod(atan2(head_des4(2,i),head_des4(1,i)),2*pi);
    
    actual_head1(i)=Z(6,i);
    actual_head2(i)=Zbar1(6,i);
    actual_head3(i)=Zbar2(6,i);
    actual_head4(i)=Zbar3(6,i);
    
    error_head1(i)=mod(actual_head1(i)-ang_des1(i),2*pi);
    error_head2(i)=mod(actual_head2(i)-ang_des2(i),2*pi);
    error_head3(i)=mod(actual_head3(i)-ang_des3(i),2*pi);
    error_head4(i)=mod(actual_head4(i)-ang_des4(i),2*pi);
    
    if error_head1(i)>pi
            error_head1(i)=error_head1(i)-2*pi;
    end
    
    if error_head2(i)>pi
            error_head2(i)=error_head2(i)-2*pi;
    end
    
    if error_head3(i)>pi
            error_head3(i)=error_head3(i)-2*pi;
    end

    
    if error_head4(i)>pi
            error_head4(i)=error_head4(i)-2*pi;
    end
end
    
error_head1=error_head1*180/pi;
error_head2=error_head2*180/pi;
error_head3=error_head3*180/pi;
error_head4=error_head4*180/pi;

plot(t(1:Nbar),error_head1,'LineWidth',1.5);
hold on
plot(t(1:Nbar),error_head2,'LineWidth',1.5);
plot(t(1:Nbar),error_head3,'LineWidth',1.5);
plot(t(1:Nbar),error_head4,'LineWidth',1.5);


  x1=xlabel('$Time~[s]$');
 y1=ylabel('$\Delta\psi~[^\circ]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')
 
leg1=legend('Car 1','Car 2','Car 3','Car 4');
 set(leg1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Heading error')
hold off

pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('head','-dsvg','-r0')


%% Path 
figure (3)
plot (r(1,1:Nbar),r(2,1:Nbar),'LineWidth',1.5);
hold on
plot(pos_tracker(1,1:Nbar),pos_tracker(2,1:Nbar),'LineWidth',1.5);
plot(pos_trackerbar1(1,1:Nbar),pos_trackerbar1(2,1:Nbar),'LineWidth',1.5);
plot(pos_trackerbar2(1,1:Nbar),pos_trackerbar2(2,1:Nbar),'LineWidth',1.5);
plot(pos_trackerbar3(1,1:Nbar),pos_trackerbar3(2,1:Nbar),'LineWidth',1.5);

  x1=xlabel('$Z_1~[m]$');
 y1=ylabel('$Z_2~[m]$');
  set(x1,'Interpreter','latex')
 set(y1,'Interpreter','latex')


leg1=legend('$Reference$','Car 1','Car 2','Car 3','Car 4');
 set(leg1,'Interpreter','latex')
 
set(gcf, 'color', 'none');
set(gca, 'color', 'none');
title('Path tracked')


pbaspect([2.5 1 1])
fig.PaperUnits = 'inches';
print('Path','-dsvg','-r0')
%     
%   xlim([-40 250]) 
%   ylim([-20 250])
% 
% s = plot(pos_tracker(1,1),pos_tracker(2,1),'o','MarkerFaceColor','red');      % bot 1
% %q = plot(r(1,1),r(2,1),'o','MarkerFaceColor','blue');                          % ref
% rbar = plot(pos_trackerbar1(1,1),pos_trackerbar1(2,1),'o','MarkerFaceColor','green');
% vbar = plot(pos_trackerbar2(1,1),pos_trackerbar2(2,1),'o','MarkerFaceColor','yellow');
% wbar = plot(pos_trackerbar3(1,1),pos_trackerbar3(2,1),'o','MarkerFaceColor','blue');
% 
% for k = 2:N
%     s.XData = pos_tracker(1,k);
%     s.YData = pos_tracker(2,k);
%       
% %     q.XData = r(1,k);
% %     q.YData = r(2,k);
%     
%     rbar.XData = pos_trackerbar1(1,k);
%     rbar.YData = pos_trackerbar1(2,k);
%     
%     vbar.XData = pos_trackerbar2(1,k);
%     vbar.YData = pos_trackerbar2(2,k);
% 
%     wbar.XData = pos_trackerbar3(1,k);
%     wbar.YData = pos_trackerbar3(2,k);
%     
%     
%     drawnow
% end
% 
