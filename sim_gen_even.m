function sim_quadcopter
clc
clear all
close all


n1 = 8;
params.m = 0.1*n1;
params.g = 9.81;

params.Ixx = 0.01275*n1;
params.Iyy = 0.01275*n1;
params.Izz = 0.0255*n1;
params.l = 0.5;
params.K = 0.00981;
params.b = 0.05;
% params.Ixx = 4.856*1e-3;
% params.Iyy = 4.856*1e-3;
% % params.Izz = 8.801*1e-3;
% params.l = 1;
% params.K = 2.980*1e-6;
% params.b = 1.14*1e-7;
params.Ax = 0.25*0;
params.Ay = 0.25*0;
params.Az = 0.25*0;

%euler angles and rates
phi0 = 0;
theta0 = 0;
psi0 = 0;
phidot0 = 0;
thetadot0 = 0; 
psidot0 = 0;

%%%%%%%%%% play with speeds of each motor to get the quadcopter to roll,
%%%%%%%%%% yaw, pitch, and hover  %%%%%%%%%%
%%%%% Tz = K*(omega1^2+omega2^2+omega3^2+omega4^2+omega5^2+omega6^2+omega7^2+omega8^2); balances with mg
%%%%% tau_x = tau_phi = K*L/sqrt(2)*(omega2^2 + omega4^2 - omega6^2 - omega8^2) + K*L*(omega3^2-omega7^2);
%%%%% tau_y = tau_theta = K*L/sqrt(2)*(omega4^2 - omega2^2 + omega6^2 - omega8^2) + K*L*(omega5^2-omega1^2);
%%%%% tau_z = tau_psi = b*(omega1^2-omega2^2+omega3^2-omega4^2+omega5^2-omega6^2+omega7^2-omega8^2);
% omega = 0.757552; %0.757552
speed = sqrt(params.m*params.g/n1/params.K);
dspeed = (1-sqrt(0.79))*speed;
% dspeed = 0.4*speed;



% params.omega1 = speed;
% params.omega2 = speed;
% params.omega3 = speed;
% params.omega4 = speed;
% params.omega5 = speed;
% params.omega6 = speed;
% params.omega7 = speed;
% params.omega8 = speed;

%%% for lateral movement
% phi0 = 0.2;
% theta0 = 0.2;
t=1/sqrt(cos(phi0))/sqrt(cos(theta0));

params.omega = cell(1,n1);

for q = 1 : n1
%     params.omega{1, q} = t*speed;
%      params.omega{1, q} = speed;
    if mod(q,2)==1
        params.omega{1, q} = 1.1*speed;
    else
        params.omega{1, q} = speed-dspeed;
    end
end


%%params.omega1 = t*speed;
%%params.omega2 = t*speed;
%%params.omega3 = t*speed;
%%params.omega4 = t*speed;
%%params.omega5 = t*speed;
%%params.omega6 = t*speed;
%%params.omega7 = t*speed;
%%params.omega8 = t*speed;

%%% to hover and spin about z axis
%params.omega1 = speed-0.5*dspeed;
%params.omega2 = speed+0.5*dspeed;
%params.omega3 = speed-0.5*dspeed;
%params.omega4 = speed+0.5*dspeed;
%params.omega5 = speed-0.5*dspeed;
%params.omega6 = speed+0.5*dspeed;
%params.omega7 = speed-0.5*dspeed;
%params.omega8 = speed+0.5*dspeed;


%position and velocity
x0 = 0;
y0 = 0;
z0 = 0;
vx0 = 0;
vy0 = 0;
vz0 = 0;

tspan = linspace(0,1);
Z0=[x0 y0 z0 phi0 theta0 psi0 vx0 vy0 vz0 phidot0 thetadot0 psidot0]';
options=odeset('abstol',1e-12,'reltol',1e-12);

[T Z] = ode45(@eom,tspan,Z0,options,params);


figure(1)
plot(T,Z);
legend('x','y','z','$\phi$','$\theta$','$\psi$','$\dot{x}$','$\dot{y}$','$\dot{z}$','$\dot{\phi}$','$\dot{\theta}$','$\dot{\psi}$','Interpreter','latex','Fontsize',12)

figure(2)
positions = Z(:,1:3);
angles = Z(:,4:6);
animate(positions,angles,params.l)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function Zdot = eom(t,Z,params)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
n1 = 8;
m = params.m;
Ixx = params.Ixx;
Iyy = params.Iyy;
Izz = params.Izz;
g = params.g;
l = params.l;
K = params.K;
b = params.b;
Ax = params.Ax;
Ay = params.Ay;
Az = params.Az;

%%omega1 = params.omega1;
%%omega2 = params.omega2;
%%omega3 = params.omega3;
%%omega4 = params.omega4;
%%omega5 = params.omega5;
%%omega6 = params.omega6;
%%omega7 = params.omega7;
%%omega8 = params.omega8;

omegao = cell(1,n1);
for q = 1 : n1
    omegao{1, q} = params.omega{1, q};
end



x = Z(1); y = Z(2); z = Z(3);
phi = Z(4); theta = Z(5); psi = Z(6);
vx = Z(7); vy = Z(8); vz = Z(9);
phidot = Z(10); thetadot = Z(11); psidot = Z(12);


%%%%%%% copy pasted from matlab %%%%%%%%%
 
A(1,1)=m;
A(1,2)=0;
A(1,3)=0;
A(1,4)=0;
A(1,5)=0;
A(1,6)=0;
A(2,1)=0;
A(2,2)=m;
A(2,3)=0;
A(2,4)=0;
A(2,5)=0;
A(2,6)=0;
A(3,1)=0;
A(3,2)=0;
A(3,3)=m;
A(3,4)=0;
A(3,5)=0;
A(3,6)=0;
A(4,1)=0;
A(4,2)=0;
A(4,3)=0;
A(4,4)=Ixx;
A(4,5)=0;
A(4,6)=-Ixx*sin(theta);
A(5,1)=0;
A(5,2)=0;
A(5,3)=0;
A(5,4)=0;
A(5,5)=Iyy - Iyy*sin(phi)^2 + Izz*sin(phi)^2;
A(5,6)=cos(phi)*cos(theta)*sin(phi)*(Iyy - Izz);
A(6,1)=0;
A(6,2)=0;
A(6,3)=0;
A(6,4)=-Ixx*sin(theta);
A(6,5)=cos(phi)*cos(theta)*sin(phi)*(Iyy - Izz);
A(6,6)=Ixx*sin(theta)^2 + Izz*cos(phi)^2*cos(theta)^2 + Iyy*cos(theta)^2*sin(phi)^2;

temp = 0;
temp1 = 0;
temp2 = 0;
temp3 = 0;
for q = 1 : n1
temp = temp + omegao{1, q}^2;   
temp1 = temp1 + (omegao{1, q}^2)*sin(((q-1)*(2*pi))/n1);
temp2 = temp2 + (omegao{1, q}^2)*cos(((q-1)*(2*pi))/n1); 
temp3 = temp3 + (omegao{1, q}^2)*cos(((q-1)*pi));  
end

Tz = K*temp;
tau_x = K*l*temp1;
tau_y = -1*K*l*temp2;
tau_z = K*l*temp3;

%%Tz = K*(omega1^2+omega2^2+omega3^2+omega4^2+omega5^2+omega6^2+omega7^2+omega8^2);
%%tau_x = K*l/sqrt(2)*(omegao{1,2}^2 + omegao{1,4}^2 - omegao{1,6}^2 - omegao{1,8}^2) + K*l*(omegao{1,3}^2-omegao{1,7}^2);
% tau_y = K*l/sqrt(2)*(omegao{1,4}^2 - omegao{1,2}^2 + omegao{1,6}^2 - omegao{1,8}^2) + K*l*(omegao{1,5}^2-omegao{1,1}^2);
% tau_z = b*(omegao{1,1}^2-omegao{1,2}^2+omegao{1,3}^2-omegao{1,4}^2+omegao{1,5}^2-omegao{1,6}^2+omegao{1,7}^2-omegao{1,8}^2);
%%tau_y = K*l/sqrt(2)*(omega4^2 - omega2^2 + omega6^2 - omega8^2) + K*l*(omega5^2-omega1^2);
%%tau_z = b*(omega1^2-omega2^2+omega3^2-omega4^2+omega5^2-omega6^2+omega7^2-omega8^2);
 
B(1,1)=K*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*Tz/K - Ax*vx;
B(2,1)=- Ay*vy - K*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*Tz/K;
B(3,1)=cos(phi)*cos(theta)*Tz - g*m - Az*vz;
B(4,1)=(Izz*thetadot^2*sin(2*phi))/2 - (Iyy*thetadot^2*sin(2*phi))/2 + tau_x + Ixx*psidot*thetadot*cos(theta) - Iyy*psidot*thetadot*cos(theta) + Izz*psidot*thetadot*cos(theta) + Iyy*psidot^2*cos(phi)*cos(theta)^2*sin(phi) - Izz*psidot^2*cos(phi)*cos(theta)^2*sin(phi) + 2*Iyy*psidot*thetadot*cos(phi)^2*cos(theta) - 2*Izz*psidot*thetadot*cos(phi)^2*cos(theta);
B(5,1)=(Ixx*psidot^2*sin(2*theta))/2 + tau_y - Ixx*phidot*psidot*cos(theta) + Iyy*phidot*thetadot*sin(2*phi) - Izz*phidot*thetadot*sin(2*phi) - Izz*psidot^2*cos(phi)^2*cos(theta)*sin(theta) - Iyy*psidot^2*cos(theta)*sin(phi)^2*sin(theta) - Iyy*phidot*psidot*cos(phi)^2*cos(theta) + Izz*phidot*psidot*cos(phi)^2*cos(theta) + Iyy*phidot*psidot*cos(theta)*sin(phi)^2 - Izz*phidot*psidot*cos(theta)*sin(phi)^2;
B(6,1)= tau_z + Ixx*phidot*thetadot*cos(theta) + Iyy*phidot*thetadot*cos(theta) - Izz*phidot*thetadot*cos(theta) - Ixx*psidot*thetadot*sin(2*theta) + Iyy*psidot*thetadot*sin(2*theta) + Iyy*thetadot^2*cos(phi)*sin(phi)*sin(theta) - Izz*thetadot^2*cos(phi)*sin(phi)*sin(theta) - 2*Iyy*phidot*thetadot*cos(phi)^2*cos(theta) + 2*Izz*phidot*thetadot*cos(phi)^2*cos(theta) - 2*Iyy*phidot*psidot*cos(phi)*cos(theta)^2*sin(phi) + 2*Izz*phidot*psidot*cos(phi)*cos(theta)^2*sin(phi) - 2*Iyy*psidot*thetadot*cos(phi)^2*cos(theta)*sin(theta) + 2*Izz*psidot*thetadot*cos(phi)^2*cos(theta)*sin(theta);
 
X = A\B;

Zdot = [vx vy vz phidot thetadot psidot X(1) X(2) X(3) X(4) X(5) X(6)]'; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function animate(positions,angles,l)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n1 = 8;

axle = cell(1,n1);
for q = 1 : n1
    axle{1, q} = [(l/2)*cos(((q-1)*2*pi)/n1) (l/2)*sin(((q-1)*2*pi)/n1) 0];
end


%%axle_1 = [-l/2 0 0;   l/2 0 0];
%%axle_2 = [-l/2/sqrt(2) l/2/sqrt(2) 0; l/2/sqrt(2)  -l/2/sqrt(2) 0];
%%axle_3 = [0 l/2 0;   0 -l/2 0];
%%axle_4 = [l/2/sqrt(2) l/2/sqrt(2) 0; -l/2/sqrt(2) -l/2/sqrt(2) 0];
      
dmax = max([max(positions),l]);

r = 0.1*l; %radius of propellers
ang = linspace(0,2*pi);
x_circle = r*cos(ang);
y_circle = r*sin(ang);
z_circle = zeros(1,length(ang));
propeller = [x_circle',y_circle',z_circle'];


[p1,q1] = size(propeller);
%%[p2,q2] = size(axle_1);
[mm,nn] = size(angles);

for ii=1 : mm
    x = positions(ii,1);
    y = positions(ii,2);
    z = positions(ii,3);
    phi = angles(ii,1); 
    theta = angles(ii,2);
    psi = angles(ii,3);
    R = get_rotation(phi,theta,psi);

    new_axle = cell(1,n1);

    for i= 1 : n1
        
        r_body = axle{1,i}';
        r_world = R*r_body;
        new_axle{1,i} = r_world';
        
        new_axle{1,i} = [x y z] + new_axle{1,i};
         
        
    end
   
   
%     for i=1:p2
%         r_body = axle_1(i,:)';
%         r_world = R*r_body;
%         new_axle_1(i,:) = r_world';
%     end
%         new_axle_1 = [x y z] +new_axle_1;
%  
%     for i=1:p2
%         r_body = axle_2(i,:)';
%         r_world = R*r_body;
%         new_axle_2(i,:) = r_world';
%     end
%     new_axle_2 = [x y z] +new_axle_2;
%     
%     for i=1:p2
%         r_body = axle_3(i,:)';
%         r_world = R*r_body;
%         new_axle_3(i,:) = r_world';
%     end
%     new_axle_3 = [x y z] +new_axle_3;
%     
%     for i=1:p2
%         r_body = axle_4(i,:)';
%         r_world = R*r_body;
%         new_axle_4(i,:) = r_world';
%     end
%     new_axle_4 = [x y z] +new_axle_4;
   
     for i=1:p1
         r_body = propeller(i,:)';
         r_world = R*r_body;
         new_propeller(i,:) = r_world';
     end

    
%     for i=1:p1
%         r_body = propeller(i,:)';
%         r_world = R*r_body;
%         new_propeller(i,:) = r_world'; 
%     end
    
    new_propellero = cell(1,n1);
    
    
    for q = 1 : n1
    new_propellero{1,q} = new_axle{1,q} + new_propeller;    
    end
    

  

%     new_propeller1 = new_axle_1(1,:) + new_propeller;
%     new_propeller5 = new_axle_1(2,:) + new_propeller;
%     new_propeller2 = new_axle_2(1,:) + new_propeller;
%     new_propeller6 = new_axle_2(2,:) + new_propeller;
%     new_propeller3 = new_axle_3(1,:) + new_propeller;
%     new_propeller7 = new_axle_3(2,:) + new_propeller;
%     new_propeller4 = new_axle_4(1,:) + new_propeller;
%     new_propeller8 = new_axle_4(2,:) + new_propeller;
    


   %%disp( (new_propellero{1,3}(:,1)+new_propellero{1,1}(:,1) )/2 +" "+((new_propellero{1,3}(:,2)+new_propellero{1,1}(:,2))/2)+" "+(( new_propellero{1,3}(:,3)+new_propellero{1,1}(:,3) )/2)) +" "+((new_propeller{1,3}(:,2)+new_propeller{1,1}(:,2))/2)+" "+(( new_propeller{1,3}(:,3)+new_propeller{1,1}(:,3) )/2)
   center = cell(1,3);
   temp1=0;
   temp2=0;
   temp3=0;
   for q = 1 : n1 
       temp1= temp1 + new_axle{1,q}(:,1);
       temp2= temp2 + new_axle{1,q}(:,2);
       temp3= temp3 + new_axle{1,q}(:,3);
   end
   center{1,1}=temp1/n1;
   center{1,2}=temp2/n1;
   center{1,3}=temp3/n1;
   
   
   
%    if mod(q,2)==0
%     for q = 1 : n1/2 
%         line([new_axle{1,q}(:,1),new_axle{1,q+n1/2}(:,1)],[new_axle{1,q}(:,2),new_axle{1,q+n1/2}(:,2)],[new_axle{1,q}(:,3),new_axle{1,q+n1/2}(:,3)],'Linewidth',2);
%     end
%    else
    for q = 1 : n1 
            line( [new_axle{1,q}(:,1),center{1,1}] ,[new_axle{1,q}(:,2),center{1,2}], [new_axle{1,q}(:,3),center{1,3}] ,'Linewidth',2);    
    end
%     end

%      plot( [0,center{1,1}] ,[0,center{1,2}], [0,center{1,3}] ,'Linewidth',2, '--or' )
      for q = 1 : n1         
       %  line([new_axle{1,q}(:,1),0],[new_axle{1,q}(:,2),0],[new_axle{1,q}(:,3),0],'Linewidth',2);

        if mod(q,2)==0
        patch(new_propellero{1,q}(:,1),new_propellero{1,q}(:,2),new_propellero{1,q}(:,3),'b');
        else
       patch(new_propellero{1,q}(:,1),new_propellero{1,q}(:,2),new_propellero{1,q}(:,3),'r');
       
       end
         
     end
        


%      line(new_axle_1(:,1),new_axle_1(:,2),new_axle_1(:,3),'Linewidth',2); hold on;
%      line(new_axle_2(:,1),new_axle_2(:,2),new_axle_2(:,3),'Linewidth',2);
%      line(new_axle_3(:,1),new_axle_3(:,2),new_axle_3(:,3),'Linewidth',2);
%      line(new_axle_4(:,1),new_axle_4(:,2),new_axle_4(:,3),'Linewidth',2);
%     patch(new_propeller1(:,1),new_propeller1(:,2),new_propeller1(:,3),'r');
%     patch(new_propeller2(:,1),new_propeller2(:,2),new_propeller2(:,3),'b');
%     patch(new_propeller3(:,1),new_propeller3(:,2),new_propeller3(:,3),'r');
%     patch(new_propeller4(:,1),new_propeller4(:,2),new_propeller4(:,3),'b');
%     patch(new_propeller5(:,1),new_propeller5(:,2),new_propeller5(:,3),'r');
%     patch(new_propeller6(:,1),new_propeller6(:,2),new_propeller6(:,3),'b');
%     patch(new_propeller7(:,1),new_propeller7(:,2),new_propeller7(:,3),'r');
%     patch(new_propeller8(:,1),new_propeller8(:,2),new_propeller8(:,3),'b');
  
    
    
    
    axis(1.2*[-dmax dmax -dmax dmax -dmax dmax]);
    xlabel('x'); ylabel('y'); zlabel('z');
    %view(0,90)
    view(3)
    grid on
    pause(0.1)
    if (ii~=mm)
        clf
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function R = get_rotation(phi,theta,psi)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%% uses 3-2-1 euler angles
R_x = [1    0       0; ...
       0  cos(phi) -sin(phi); ...
       0  sin(phi) cos(phi)];
   
R_y = [cos(theta)  0   sin(theta); ...
       0           1         0; ...
      -sin(theta) 0   cos(theta)]; 
   
R_z = [cos(psi) -sin(psi)  0; ...
       sin(psi)  cos(psi)  0; ...
       0           0       1];  
R = R_z*R_y*R_x;
