clc
clear all

%% Parameters
params.n_states = 3;
params.stance_legs = cellstr(['rf'; 'rh'; 'lh']);
params.width = 0.5;
params.length = 1.0;
params.base_position = [0.0; 0.0]; % m
params.yaw_0 = 0.5; % rad
params.plot_margin = 1.5;
params.offsets.lf = [1.0; 0.0];
params.offsets.rf = [0.0; 0.0];
params.offsets.rh = [0.0; 0.0];
params.offsets.lh = [0.0; 0.0];

%% Setup
params.n_stance_leg = length(params.stance_legs);

% Nominal foot distances from base center
distances.lf = [ params.length;  params.width];
distances.rf = [ params.length; -params.width];
distances.rh = [-params.length; -params.width];
distances.lh = [-params.length;  params.width];
hips = fieldnames(distances);
n_hips = length(hips);

% Foot positions
R_0 = [cos(params.yaw_0) -sin(params.yaw_0);
       sin(params.yaw_0)  cos(params.yaw_0)];
foot_positions.lf = params.base_position + R_0 * (distances.lf + params.offsets.lf);
foot_positions.rf = params.base_position + R_0 * (distances.rf + params.offsets.rf);
foot_positions.rh = params.base_position + R_0 * (distances.rh + params.offsets.rh);
foot_positions.lh = params.base_position + R_0 * (distances.lh + params.offsets.lh);
feet = fieldnames(foot_positions);
n_feet = length(feet);

% Support polygon
support = zeros(params.n_stance_leg+1, 2);
for i = 1:params.n_stance_leg
    support(i, :) = foot_positions.(params.stance_legs{i})';
end
% Add extra point to close polygon
support(params.n_stance_leg+1,:) = foot_positions.(params.stance_legs{1})';

%% Problem definition
% min Ax - b, Gx <= h

% Objective
A = zeros(2 * n_feet, params.n_states);
b = zeros(2 * n_feet, 1);
R_star = [0 -1;
          1  0];
for i = 1:n_feet
    A(1+2*(i-1):2*i,:) = horzcat(eye(2), R_0*R_star*distances.(hips{i}));
    b(1+2*(i-1):2*i) = foot_positions.(feet{i}) - R_0*distances.(hips{i});
end

% Inequality constraints
% G = zeros(params.n_stance_leg, 2);
% h = zeros(params.n_stance_leg, 1);
% for i = 1:params.n_stance_leg
%     foot_1 = support(i,:)';
%     foot_2 = support(i+1,:)';
%     m = (foot_2(2) - foot_1(2)) / (foot_2(1) - foot_1(1));
%     G(i, :) = [-m 1]
%     h(i) = -m*foot_1(1) + foot_1(2)
% end
[G,h]=vert2con(support);
G = horzcat(G, zeros(size(G, 1), 1));
  
%% Formulation as QP
% min 1/2 x'Px + q'x + r

P = 2*A'*A;
q = -2*A'*b;
r = b'*b;

%% Solve
[x,fval,exitflag,output,lambda] = quadprog(P, q, G, h);
%[x,fval,exitflag,output,lambda] = linprog(f,A,b,[],[],lb);
x

yaw = params.yaw_0 + x(3);
T = [cos(yaw)  sin(yaw) 0 x(1);
     -sin(yaw)  cos(yaw)  0 x(2);
     0          0         1 0;
     0          0         0 1]

residual = A*x - b;

%% Compute hip positions
for i = 1:n_hips
    hip_positions.(hips{i}) = x(1:2) + R_0 * (eye(2) + R_star*x(3)) * distances.(hips{i});
end

%% Plot

% Setup
figure
hold on
x_min = params.base_position(1) - params.plot_margin;
x_max = params.base_position(1) + params.plot_margin;
y_min = params.base_position(2) - params.plot_margin;
y_max = params.base_position(2) + params.plot_margin;
axis([x_min x_max y_min y_max])
axis equal

% Support polygon
plot(support(:,1), support(:,2), ':k')

% Feet points
for i = 1:n_feet
    foot = foot_positions.(feet{i});
    plot(foot(1), foot(2), 'bo')
end

% Base center
plot(x(1), x(2), 'kx')

% Base outline
outline = zeros(n_hips+1, 2);
for i = 1:n_hips
    outline(i, :) = hip_positions.(hips{i})';
end
outline(n_hips+1,:) = hip_positions.(hips{1})';
plot(outline(:,1), outline(:,2), '-k')

% Leg & error
leg = zeros(2, 2);
error = 0;
for i = 1:n_feet
    leg(1, :) = hip_positions.(hips{i});
    leg(2, :) = foot_positions.(feet{i});
    error = error + sum((leg(1, :) - leg(2, :)).^2);
    plot(leg(:, 1), leg(:, 2), '-r')
end
sqrt(error) % ~= norm(residual)
