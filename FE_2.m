% Final Exam Problem 2
clear;clc;close all;

%choose which performance index weight to vary, while keeping others at 1
%alpha=1, beta=2, q=3
varied_wt = 3;

switch varied_wt
    case 1
        a_wt = [0, 0.5, 1, 10, 100];
        b_wt = 1;
        weight = 1;
        x = 0:10;
        for i = 1:length(a_wt)
            plot(x,DP_problem2(a_wt(i),b_wt,weight),'linewidth',2);
            lgd{i} = ['alpha= ', num2str(a_wt(i))];
            legend(lgd);
            title('Optimal Velocity Profile - varying alpha');
            hold on;
        end

    case 2
        a_wt = 1;
        b_wt = [0, 0.5, 1, 10, 300];
        weight = 1;
        x = 0:10;
        for i = 1:length(b_wt)
            plot(x,DP_problem2(a_wt,b_wt(i),weight),'linewidth',2);
            lgd{i} = ['beta= ', num2str(b_wt(i))];
            legend(lgd);
            title('Optimal Velocity Profile - varying beta');
            hold on;
        end

    case 3
        a_wt = 1;
        b_wt = 1;
        weight = [0, 0.5, 1, 10, 300];
        x = 0:10;
        for i = 1:length(weight)
            plot(x,DP_problem2(a_wt,b_wt,weight(i)),'linewidth',2);
            lgd{i} = ['q= ', num2str(weight(i))];
            legend(lgd);
            title('Optimal Velocity Profile - varying q');
            hold on;
        end
end

ylim([0 5]);
grid on;
xlabel('Distance, x [m]');
ylabel('Velocity, Vopt [m/s]');

%-------------------------------------------------------------------------
function Vopt = DP_problem2(a,b,q)
% Initialize variables:
xmin = 0; xmax = 10; deltax = 1; % steps for distance (counter k)
vmin = 0; vmax = 5; deltav = .5; % steps for velocity (counter j / h)
amin = -3; amax = 3; deltaa = .5; % steps of acceleration (controlinput)
vinit = 0; vfinal = 0; % constraints on init and final velocity
lrg = 1e4; % large cost value
% Relevant equations, from kinematics:
% v(k+1) = v(k) + a(k)*deltat -- gives: deltat = (v(k+1)-v(k))/a(k)
% or: deltat = 2*deltax /(v(k+1)+v(k))
% v(k+1)^2 - v(k)^2 = 2a*deltax -- gives: a(k) = (v(k+1)^2-v(k)^2)/(2*deltax)
x = xmin:deltax:xmax; kmax = length(x);
v = vmin:deltav:vmax; hmax = length(v); jmax = hmax;
costmin = lrg*ones(hmax,kmax); costmin(1,kmax)=0;

m = 2;
A = 0.1;
Cd = 0.4;
rho = 1.204;
mu = 0.2;
g = 10;

for k=kmax-1:-1:1
    if k==1 % Velocity constraint on initial time step (vf=0)
    hmax = find(v==vinit);
    end
    for h=hmax:-1:1 % indexes possible velocities at (k-1)th timestep
        dt = zeros(jmax,1); accel=dt-dt; % initialize large time matrix
        PM_t = zeros(jmax,1);
        E = zeros(jmax,1);
        J = zeros(jmax,1);
        for j=jmax:-1:1 % indexes possible actions at kth timestep
            if v(h)==v(j) % Eliminate NaN errors
                accel(j) = 0; % Accel
                if v(h)~=0
                dt(j) = deltax/v(h); % Deltat
                elseif v(h)==0
                J(j)=lrg+costmin(j,k+1);
                break
                end
            else
                accel(j) = (v(j)^2-v(h)^2)/(2*deltax); % Accel
                dt(j) = (2*deltax)/(v(j)+v(h)); % Deltat
            end
            if accel(j) < amin || accel(j) > amax
                J(j) = lrg; % Prohibits accel values that violate the constraint
            else
                drag = 0.5*Cd*rho*A*(v(h)^2+v(j)^2)/2;
                E(j) = abs(m*accel(j))+drag+mu*m*g;
                PM_t(j) = 11+0.5*(v(h)^2+v(j)^2)-3*(v(h)+v(j));
                J(j)= a*E(j)+b*PM_t(j)+q*dt(j);
            end
            J(j) = J(j)+ costmin(j,k+1);
        end
            Jmin = min(J);
            [row,col] = find(J==Jmin);
            if length(row)>1
                row=row(1); col=col(1);
            end
            costmin(h,k) = J(row);
            umin(h,k) = accel(row);
    end
end
% Generate optimal costs, velocity, and accel control vectors
for k=1:kmax-1
    if k==1
        Vopt(k)=vinit;
        mininitcost = min(costmin(:,1));
        [row,col] = find(costmin==mininitcost);
        Uopt(k) = umin(row,col);
        
        Vopt(k+1) = sqrt(Uopt(k)*2*deltax);
        Costtogo(k) = mininitcost;
    else
        [row,col]=find(v==Vopt(k));
        Uopt(k) = umin(col,k);
        Vopt(k+1) = sqrt(Uopt(k)*2*deltax+Vopt(k)^2);
        Costtogo(k) = costmin(col,k);
    end
end
end
