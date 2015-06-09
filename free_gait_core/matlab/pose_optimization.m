clc
clear all

%% Parameters
params.stance_legs = cellstr(['rf'; 'rh'; 'lh']);
params.width = 0.2;
params.length = 0.25;
params.base_position = [1.0; 3.0];
params.plot_margin = 0.4;
params.offsets.lf = [0.1; 0.0];
params.offsets.rf = [-0.04; 0.02];
params.offsets.rh = [-0.07; 0.03];
params.offsets.lh = [0; 0.01];

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
foot_positions.lf = params.base_position + distances.lf + params.offsets.lf;
foot_positions.rf = params.base_position + distances.rf + params.offsets.rf;
foot_positions.rh = params.base_position + distances.rh + params.offsets.rh;
foot_positions.lh = params.base_position + distances.lh + params.offsets.lh;
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
A = repmat(eye(2), n_feet, 1);
b = zeros(2 * n_feet, 1);
for i = 1:n_feet
    b(1+2*(i-1):2*i) = foot_positions.(feet{i}) - distances.(hips{i});
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
  
%% Formulation as QP
% min 1/2 x'Px + q'x + r

P = 2*A'*A;
q = -2*A'*b;
r = b'*b;

%% Solve
[x,fval,exitflag,output,lambda] = quadprog(P, q, G, h);
%[x,fval,exitflag,output,lambda] = linprog(f,A,b,[],[],lb);

%% Compute hip positions
for i = 1:n_hips
    hip_positions.(hips{i}) = x + distances.(hips{i});
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

% Leg
leg = zeros(2, 2);
for i = 1:n_feet
    leg(1, :) = hip_positions.(hips{i});
    leg(2, :) = foot_positions.(feet{i});
    plot(leg(:, 1), leg(:, 2), '-r')
end

