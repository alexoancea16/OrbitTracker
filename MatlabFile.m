clc; clear; close all;

% Initial conditions provided in the problem
r1_t0 = 10^6 * [-3.111567646661099; 2.420733547442338; -5.626803092595423];
r2_t0 = 10^6 * [-3.422732421327209; 2.662806902186572; -6.18948308151366];
d_r1_t0 = 10^3 * [4.953572247000772; -3.787243278806948; -4.362500902062312];
d_r2_t0 = 10^3 * [5.448929471700850; -4.165967606687643; -4.798750992268544];

%% Requirement 1

% Initial condition for epsilon
epsilon_t0 = 10^(-5);

%% Requirement 2

% Simulink model file
model = 'SimulinkFileModel.slx';

%% Requirement 3

% Writing the system as a first-order differential equation

%% Requirement 4

k = 2; % input constant
h = 1; % integration step size
T = 1800; % total simulation time
t = 0:1:T-1; % time vector
x0 = [r1_t0; d_r1_t0; r2_t0; d_r2_t0; epsilon_t0]; % initial state
x = zeros(length(x0), 1800); % initialize state vector
x(:, 1) = x0;

% Runge-Kutta 4th-order method implementation
for i = 1:T-1
    k1 = fct_rk(t(i), x(:, i), k);
    k2 = fct_rk(t(i) + (h/2), x(:, i) + (h/2)*k1, k);
    k3 = fct_rk(t(i) + (h/2), x(:, i) + (h/2)*k2, k);
    k4 = fct_rk(t(i) + h, x(:, i) + h*k2, k);
    x(:, i+1) = x(:, i) + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
end

%% Requirement 5 

% r1 and r2 positions over time
r1_tr = x(1:3, :); 
r2_tr = x(7:9, :);

% Plotting the satellite trajectories using the RK4 method
plot3(r1_tr(1, :), r1_tr(2, :), r1_tr(3, :), 'r', 'DisplayName', 'r_1 (RK4)');
hold on;
plot3(r2_tr(1, :), r2_tr(2, :), r2_tr(3, :), 'b', 'DisplayName', 'r_2 (RK4)');
xlabel('x [m]'); % Label for the X-axis
ylabel('y [m]'); % Label for the Y-axis
zlabel('z [m]'); % Label for the Z-axis
title('Satellite Trajectories using the RK4 Method'); % Plot title
legend('Location', 'best'); % Display legend
grid on;

format long;
out = sim(model); % Running the Simulink model
r1_sim = out.r1_sim.data; % Extracting r1 from Simulink results
r2_sim = out.r2_sim.data; % Extracting r2 from Simulink results

% Plotting the trajectories obtained from Simulink simulation
figure;
plot3(r1_sim(:,1), r1_sim(:,2), r1_sim(:,3), 'r--', 'DisplayName', 'r_1 (Simulink)');
hold on;
plot3(r2_sim(:,1), r2_sim(:,2), r2_sim(:,3), 'b--', 'DisplayName', 'r_2 (Simulink)');
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Satellite Trajectories using Simulink');
legend('Location', 'best');
grid on;

%% Requirement 6

% Output y and time from Simulink
y_sim = out.y.data;
t_sim = out.tout;

% Output y and time from RK method
y_rk = x(13,:);
t_rk = t;

% Interpolation for comparing y values
y_rk_interp = interp1(t_rk, y_rk, t_sim, 'linear');

% Compute the 2-norm of the error between Simulink and RK method
err = sqrt((y_sim - y_rk_interp).^2);

% Plot the integration error
figure;
plot(t_sim, err, 'r', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Error ||y_{sim} - y_{rk}||_2');
title('Integration Error between Simulink and Runge-Kutta');
grid on;

%% Requirement 7

epsilon = x(13,end); % Final epsilon value

% Example constant values for k
k_values = [0.1, 0.2, 0.5, 1, 2]; 

% Arrays to store epsilon* and u* values
epsilon_star_values = zeros(size(k_values));
u_star_values = zeros(size(k_values));

% Compute epsilon* and u* for different values of k
for i = 1:length(k_values)
    k = k_values(i); 
    epsilon_star_values(i) = x(13,i);
    u_star_values(i) = k * 1e-3;
end

% Plot epsilon* vs u*
figure;
plot(u_star_values, epsilon_star_values, 'o-', 'LineWidth', 1.5);
xlabel('u^* (Perturbation Acceleration)');
ylabel('\epsilon^* (Final Value at t_f)');
title('Dependence between \epsilon^* and u^* for Constant k Values');
grid on;

%% Requirement 8

% Determine polynomial approximation coefficients
coefficients = polyfit(u_star_values, epsilon_star_values, 2);

% Evaluate the polynomial based on the data
epsilon_approx = polyval(coefficients, u_star_values);

% Compute the approximation error
error = norm(epsilon_star_values - epsilon_approx);

% Plot experimental data and polynomial fit
figure;
plot(u_star_values, epsilon_star_values, 'bo', 'MarkerSize', 8, 'DisplayName', 'Experimental Data');
hold on;
u_fit = linspace(min(u_star_values), max(u_star_values), 100);
epsilon_fit = polyval(coefficients, u_fit);
plot(u_fit, epsilon_fit, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Polynomial Fit');
xlabel('u^* (Perturbation Acceleration)');
ylabel('\epsilon^* (Final Value at t_f)');
title('Approximation of \epsilon^* as a Function of u^*');
legend('show');
grid on;

disp('Polynomial approximation coefficients:');
disp(coefficients);
fprintf('Approximation error (2-norm): %.5e\n', error);

%% Requirement 9

% Number of simulations
num_simulations = 100;

% Generate alpha values for uncertainty
alpha_values = normrnd(0, 0.1, [1, num_simulations]);

epsilon_max = zeros(1, num_simulations); % To store maximum epsilon values

% Analysis threshold (10 km)
threshold = 10e3; 

epsilon = zeros(num_simulations,1800);
for i = 1:num_simulations
    r2_perturbed = (1 + alpha_values(i)) * r2_t0; % Perturbed initial position for r2
    x0 = [r1_t0; d_r1_t0; r2_perturbed; d_r2_t0; 0.05]; % Initial state vector
    x = zeros(length(x0), 1800);
    x(:, 1) = x0;
    for j = 1:T-1
        k1 = fct_rk(t(j), x(:, j), k);
        k2 = fct_rk(t(j) + (h/2), x(:, j) + (h/2)*k1, k);
        k3 = fct_rk(t(j) + (h/2), x(:, j) + (h/2)*k2, k);
        k4 = fct_rk(t(j) + h, x(:, j) + h*k2, k);
        x(:, j+1) = x(:, j) + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
    epsilon(i,:) = x(13, :); 
    epsilon_max(i) = max(epsilon(i,:)); 
end

% Probability that epsilon exceeds the threshold
probability = sum(epsilon_max > threshold) / num_simulations;

% Plot epsilon evolution for all simulations
figure;
hold on;
for i = 1:num_simulations
    plot(t, epsilon(i,:), 'b-', 'DisplayName', 'Last Simulation');
end
xlabel('Time [s]');
ylabel('\epsilon(t)');
title('Evolution of \epsilon(t) for Simulations with Uncertainty');
grid on;

fprintf('The probability that epsilon(t) exceeds the %.2f km threshold is %f%%\n', threshold / 1e3, probability * 100);

%% Requirement 10

% Number of simulations
num_simulations = 100;

epsilon_max = zeros(1, num_simulations); % Maximum epsilon values

% Analysis threshold (10 km)
threshold = 10e3; 

% Generate alpha values for additive uncertainty
alpha_values = 20000 * normrnd(0, 5, [3, num_simulations]);

for i = 1:num_simulations
    r2_perturbed = alpha_values(:, i) + r2_t0; % Perturbed initial position for r2
    x0 = [r1_t0; d_r1_t0; r2_perturbed; d_r2_t0; 0.05]; % Initial state vector
    x = zeros(length(x0), 1800);
    x(:, 1) = x0;
    k = 2; % Input constant
    h = 1; % Integration step
    T = 1800; % Total time
    t = 0:1:1800-1; % Time vector
    for j = 1:T-1
        k1 = fct_rk(t(j), x(:, j), k);
        k2 = fct_rk(t(j) + (h/2), x(:, j) + (h/2)*k1, k);
        k3 = fct_rk(t(j) + (h/2), x(:, j) + (h/2)*k2, k);
        k4 = fct_rk(t(j) + h, x(:, j) + h*k2, k);
        x(:, j+1) = x(:, j) + (h/6) * (k1 + 2*k2 + 2*k3 + k4);
    end
    epsilon(i,:) = x(13, :); 
    epsilon_max(i) = max(epsilon(i,:)); 
end

% Probability that epsilon exceeds the threshold
probability = sum(epsilon_max > threshold) / num_simulations;

% Plot epsilon evolution for all simulations
figure;
hold on;
for i = 1:num_simulations
    plot(t, epsilon(i,:), 'b-', 'DisplayName', 'Last Simulation');
end
xlabel('Time [s]');
ylabel('\epsilon(t)');
title('Evolution of \epsilon(t) for Simulations with Additive Uncertainty');
grid on;

fprintf('The probability that epsilon(t) exceeds the %.2f km threshold is %.2f%%\n', threshold / 1e3, probability * 100);

%% Requirement 11

Tmax = 200; % Maximum time for simulation
t = 0:0.01:Tmax; % Time vector
u = normrnd(0, 1, size(t)); % Random exogenous signal u(t)
mdl = 'SimulinkFileModel_ex.slx';
simin = timeseries(u,t); % Create timeseries for simulation input
load_system(mdl); 
y = out.y.data; % Extract simulation output

% Compute first and second discrete derivatives
dy = diff(y); 
d2y = diff(dy);

% Plot the distribution of the exogenous signal
figure;
subplot(2, 1, 1);
histogram(u, 'Normalization', 'pdf');
xlabel('u(t) Value');
ylabel('Relative Frequency');
title('Distribution of the Exogenous Signal u(t)'); 
grid on;

% Plot the distribution of the second discrete derivative of y(t)
subplot(2, 1, 2);
histogram(d2y, 'Normalization', 'pdf');
xlabel('Second Discrete Derivative of y(t)'); % Label for the X-axis
ylabel('Relative Frequency'); % Label for the Y-axis
title('Distribution of the Second Discrete Derivative of y(t)'); % Title for the plot
grid on;
ylim([0 1000]); % Adjust Y-axis limits

% Display statistics for u(t) and d2y
fprintf('Mean of u(t): %.4f\n', mean(u));
fprintf('Standard deviation of u(t): %.4f\n', std(u));
fprintf('Mean of d2y: %.4f\n', mean(d2y));
fprintf('Standard deviation of d2y: %.4f\n', std(d2y));
