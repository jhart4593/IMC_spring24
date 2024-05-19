% HW5 #2
clear;clc;close all;

%Constants
m = 2; %kg
A = 0.1; %m2
Cd = 0.4;
p = 1.204; %kg/m3
mu = 0.2;
g = 10; %m/s2

%Initial and final conditions, limits
xi = 0; % start pos
xf = 10; % end pos
vi = 0; % initial velocity
vf = 0; % end velocity
v_lims = 0:0.5:5; % velocity limits
vll = length(v_lims); % length of velocity range
a_lims = [-3,3]; % acceleration limits
q = [0 0.5 1 10 300];

for ll = 1:length(q)

    %Find optimal acceleration profile to minimize energy performance index
    opt_vals1 = zeros(5,xf); % optimal start vel, end vel, accel, time, perf index for each 1m segment
    opt_vals2 = zeros(5,xf); % optimal start vel, end vel, accel, time, perf index for each 1m segment
    
    % calculate v0 that minimizes performance index given ve final = 0. Start at last 
    % segment and work back.
    ve1 = 0;
    v0_vec1 = v_lims;
    
    for ii = xf:-1:1
        ve_vec1 = ve1*ones(1,vll);
        t1 = 2./(v0_vec1+ve_vec1);
        a1 = (ve_vec1-v0_vec1)./t1;
        E1 = m.*(abs(a1)+mu*g) + 1/4*Cd*p*g*A.*(v0_vec1.^2 + ve_vec1.^2);
        J1 = E1+q(ll).*t1;
    
        min_J = 15+q(ll)*5;
        for jj = 1:vll
            if (J1(jj)<min_J) && (a1(jj)>=a_lims(1)) && (a1(jj)<=a_lims(2)) 
                t1_opt = t1(jj);
                a1_opt = a1(jj);
                v0_opt1 = v0_vec1(jj);
                J1_opt = J1(jj);
                min_J = J1_opt;
            end
        end
    
        opt_vals1(:,ii) = [v0_opt1;ve1;a1_opt;t1_opt;J1_opt];
        ve1 = v0_opt1;
    end
    
    % calculate ve that minimizes performance index given v0 start = 0. Start at first 
    % segment and work forward.
    v02 = 0;
    ve_vec2 = v_lims;
    
    for mm = 1:xf
        v0_vec2 = v02*ones(1,vll);
        t2 = 2./(v0_vec2+ve_vec2);
        a2 = (ve_vec2-v0_vec2)./t2;
        E2 = m.*(abs(a2)+mu*g) + 1/4*Cd*p*g*A.*(v0_vec2.^2 + ve_vec2.^2);
        J2 = E2+q(ll).*t2;
    
        min_J = 15+q(ll)*5;
        for nn = 1:vll
            if (J2(nn)<min_J) && (a2(nn)>=a_lims(1)) && (a2(nn)<=a_lims(2)) 
                t2_opt = t2(nn);
                a2_opt = a2(nn);
                ve_opt2 = ve_vec2(nn);
                J2_opt = J2(nn);
                min_J = J2_opt;
            end
        end
    
        opt_vals2(:,mm) = [v02;ve_opt2;a2_opt;t2_opt;J2_opt];
        v02 = ve_opt2;
    end
    
    %find point at which optimal values of velocity from each method match up
    for kk = 1:(length(opt_vals1)-1)
        if opt_vals1(1,kk+1) == opt_vals2(2,kk)
            eq_segment = kk;
        end
    end
    
    %combined optimal values (v0,ve,a,t) and plot velocity
    opt_vals = [opt_vals2(:,1:eq_segment) opt_vals1(:,eq_segment+1:end)];
    
    if ll == 3
        plot(0:xf,[opt_vals(1,:) opt_vals(2,end)],':','LineWidth',3)
        hold on
    else
        plot(0:xf,[opt_vals(1,:) opt_vals(2,end)],'LineWidth',3)
        hold on
    end
end

grid on
xticks(0:10)
xlabel('Distance, x [m]')
ylabel('Velocity, Vopt [m/s]')
title('Optimal Velocity Profile - Minimizing Performance Index')
legend('q = 0','q = 0.5','q = 1','q = 10','q = 300')