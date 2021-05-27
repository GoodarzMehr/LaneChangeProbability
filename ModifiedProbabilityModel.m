%% Modified Probability Model

% This script estimates the probability that a vehicle can reach the
% right-most lane using one or multiple lane changes before reaching a
% goal state. To develop this model, several assumptions are made to simplify
% the problem. First, it is assumed that in each lane to the right of the ego
% vehicle, inter-vehicle headway times are I.I.D. random variables from a
% log-normal distribution with parameters mu(i) (i denoting the number of
% lane) and sigma(i). Further, it is assumed that all vehicles on lane i have
% velocity v(i), and that no vehicle other than the ego vehicle changes lanes.
% Finally, it is assumed that the minimum safe inter-vehicle distance required
% for the ego vehicle to change lanes is given by the formula dM(i) = dS +
% lv + delta * v(i), where dS and lv are constants. Also, it is assumed that
% it takes the ego vehicle tLC(i) seconds to move from lane i - 1 to lane i.

% Author: Goodarz Mehr
% Email: goodarzm@vt.edu
% First written: 8/6/2019
% Last revision: 12/16/2019
% Published Under GPL-2.0.

% Define parameters.

dE = 5000; % Maximum range for ego vehicle distance from the goal state (m).
dET = 1000; % Ego vehicle distance from the goal state for a sample point
% (m).
v = [37.9428; 35.6222; 33.1194; 30.3083]; % Average velocity of vehicles on
% each lane (m/s).
lv = 5; % Average length of a vehicle (m).
dS = 2; % Minimum safe distance constant (m).
delta = 1.6; % Lane velocity multiplier for minimum safe distance (s).
muT = [0; 0.8418; 0.9603; 0.9724]; % Headway time distribution parameter
% (mu) (s).
sigma = [0; 0.6089; 0.6833; 0.7662]; % Headway time distribution parameter
% (sigma) (s).
tLC = [0; 3; 3; 3]; % The average time it takes to change lanes (s).
NL = 4; % Maximum number of lanes.
dM = zeros(NL, 1); % Minimum safe distance array.
mu = zeros(NL, 1); % Headway distance distribution parameter (mu) array.

% Calculate dM and mu.

for i = 2 : 1 : NL
    dM(i) = dS + lv + delta * v(i);
    mu(i) = muT(i) + log(v(i));
end

% Calculate the probability of successfully reaching the sample point for
% different number of lanes.

PT = zeros(1, NL - 1);

for j = 1 : 1 : NL - 1
    PT(1, j) = LaneChangeProbCalc(dET, dM(1 : j + 1), v(1 : j + 1),...
        mu(1 : j + 1), sigma(1 : j + 1), tLC(1 : j + 1), j, 1);
end
    
% Calculate the probability of successfully reaching a goal state some
% distance away for different number of lanes.

N = 2 ^ (floor(log2(dE + 1)));
P = zeros(N + 1, NL - 1);

for j = 1 : 1 : NL - 1
    P(:, j) = LaneChangeProbCalc(dE, dM(1 : j + 1), v(1 : j + 1),...
        mu(1 : j + 1), sigma(1 : j + 1), tLC(1: j + 1), j, 2);
end

% Plot the results.

x = dE / N * (0 : 1 : N)';
 
figure;
plot(x, P(:, 1));
hold on;

for j = 2 : 1 : NL - 1
    plot(x, P(:, j));
end

% Change plot formatting to LaTeX.

xlabel('$d_{E}$ (m)', 'Interpreter', 'latex');
ylabel('P', 'Interpreter', 'latex');
legend({'Two lanes', 'Three lanes', 'Four lanes', 'Five lanes',...
    'Six lanes', 'Seven lanes'}, 'Location', 'southeast',...
    'Interpreter', 'latex');

ax = gca;
ax.TickLabelInterpreter = 'latex';

% Examine the effect of the velocity of the ego vehicle on the probability.

% vT = (20 : 0.1 : 50)';
% PvT = zeros(301, NL - 1);
% 
% for i = 1 : 1 : 301
% 
%     v(1) = vT(i);
%     
%     for j = 1 : 1 : NL - 1
%         PvT(i, j) = LaneChangeProbCalc(dET, dM(1 : j + 1), v(1 : j + 1),...
%             mu(1 : j + 1), sigma(1 : j + 1), j, 1);
%     end
% end

% plot the results.

% figure;
% plot(vT, PvT(:, 1));
% hold on;
% 
% for j = 2 : 1 : NL - 1
%     plot(vT, PvT(:, j));
% end

% Change plot formatting to LaTeX.

% xlabel('$v_{1}$ (m/s)', 'Interpreter', 'latex');
% ylabel('P', 'Interpreter', 'latex');
% legend({'Two lanes', 'Three lanes', 'Four lanes', 'Five lanes',...
%     'Six lanes', 'Seven lanes'}, 'Location', 'southeast',...
%     'Interpreter', 'latex');
% 
% ax = gca;
% ax.TickLabelInterpreter = 'latex';

% Examine the effect of the minimum safe distance velocity multiplier
% on the probability.

% deltaT = (0.5 : 0.1 : 4)';
% PdeltaT = zeros(36, NL - 1);
% 
% for i = 1 : 1 : 36
%     
%     delta = deltaT(i);
% 
%     for k = 2 : 1 : NL
%         dM(k) = dS + lv + delta * v(k);
%     end
%     for j = 1 : 1 : NL - 1
%         PdeltaT(i, j) = LaneChangeProbCalc(dET, dM(1 : j + 1),...
%             v(1 : j + 1), mu(1 : j + 1), sigma(1 : j + 1), j, 1);
%     end
% end

% plot the results.

% figure;
% plot(deltaT, PdeltaT(:, 1));
% hold on;
% 
% for j = 2 : 1 : NL - 1
%     
%     plot(deltaT, PdeltaT(:, j));
%     
% end

% Change plot formatting to LaTeX.

% xlabel('$\delta$ (s)', 'Interpreter', 'latex');
% ylabel('P', 'Interpreter', 'latex');
% legend({'Two lanes', 'Three lanes', 'Four lanes', 'Five lanes',...
%     'Six lanes', 'Seven lanes'}, 'Location', 'northeast',...
%     'Interpreter', 'latex');
% 
% ax = gca;
% ax.TickLabelInterpreter = 'latex';

% Examine the effect of traffic density on the probability.

% muTT = rand(10000, NL - 1) .* 2;
% sigmaT = rand(10000, NL - 1) + 0.2;
% rhoT = 3600 ./ (exp(muTT + (sigmaT .^ 2) ./ 2));
% CrhoT = cumsum(rhoT, 2);
% PrhoT = zeros(10000, NL - 1);
% 
% parfor i = 1 : 1 : 10000
%     
%     muT = [0; muTT(i, :)'];
%     sigma = [0; sigmaT(i, :)'];
%     mu = zeros(NL, 1);
%     mu(2 : NL) = muT(2 : NL) + log(v(2 : NL));
%     temp = zeros(1, NL - 1);
%     
%     for j = 1 : 1 : NL - 1
%         temp(j) = LaneChangeProbCalc(dET, dM(1 : j + 1), v(1 : j + 1),...
%             mu(1 : j + 1), sigma(1 : j + 1), j, 1);
%     end
%     
%     PrhoT(i, :) = temp;
%     
% end

% plot the results.

% figure;
% scatter(CrhoT(:, 1), PrhoT(:, 1), '.');
% hold on;
% 
% for j = 2 : 1 : NL - 1
%     CrhoT(:, j) = CrhoT(:, j) ./ j;
%     scatter(CrhoT(:, j), PrhoT(:, j), '.');
% end

% Change plot formatting to LaTeX.

% xlabel('Average traffic density (vehicle/(lane * hr))', 'Interpreter',...
%     'latex');
% ylabel('P', 'Interpreter', 'latex');
% legend({'Two lanes', 'Three lanes', 'Four lanes', 'Five lanes',...
%     'Six lanes', 'Seven lanes'}, 'Location', 'northeast',...
%     'Interpreter', 'latex');
% 
% ax = gca;
% ax.TickLabelInterpreter = 'latex';

function P = LaneChangeProbCalc(dE, dM, v, mu, sigma, tLC, n, m)

    %% Lane Change Probability Calculator function

    % This function calculates the probability of conducting one or
    % more successful lane changes before reaching a goal state some
    % distance dE away.
    %
    % Inputs:
    %
    %   dE := distance to the goal state.
    %   dM := array of minimum safe inter-vehicle distance.
    %   v := array of average lane velocities.
    %   mu := array of headway time distribution parameters (mu).
    %   sigma := array of headway time distribution parameters (sigma).
    %   tLC := average time it takes to change lanes (s).
    %   n := number of lane changes.
    %   m := mode. Mode 1 calculates the probability for a single point.
    %       Mode 2 calculates the probability for the entire range of
    %       distance from 0 to dE.
    %
    % Outputs:
    %
    %   P := probability of successfully reaching the goal state.
    
    % Author: Goodarz Mehr
    % Email: goodarzm@vt.edu
    % First written: 8/6/2019
    % Last revision: 12/16/2019
    % Published Under GPL-2.0.
    
    if n == 1
    
        % First consider the case where n = 1, that is, only one lane
        % change is required. For this case, the probability is calculated
        % from interpolation of probability data for a unit interval based
        % on three parameters: mu, sigma, and d.
        
        % Define parameters.
        
        N = 0;
        dMM = dM(2);
        v1 = v(1);
        v2 = v(2);
        mu = mu(2);
        sigma = sigma(2);
        tLC = tLC(2);

        if m == 2
            N = 2 ^ (floor(log2(dE + 1)));
            dE = dE / N * (0 : 1 : N)';
        end
        
        dE = dE - tLC * v1;
        ZI = size(dE(dE(:, 1) < 0), 1);

        if v1 >= v2
            dR = dE * (1 - v2 / v1);
        else
            dR = dE * (v2 / v1 - 1);
        end

        dSR = dR + dMM; % Relative search distance.

        % Normalize parameters for a unit search interval.

        mu = mu - log(dSR);
        d = dMM ./ dSR;

        % Find the index of the triplet just-below (element-wise) of the
        % current values to use for interpolation.

        i = max(min(floor((mu + 5) / 0.05) + 1, 120), 1);
        j = max(min(floor(sigma / 0.05) + 1, 40), 1);
        k = max(min(floor(d / 0.01) + 1, 100), 1);

        % Calculate the normalized distance from the triplet just below for
        % each parameter.

        x = (mu - ((i - 1) * 0.05 - 5)) / 0.05;
        y = (sigma - ((j - 1) * 0.05)) / 0.05;
        z = (d - ((k - 1) * 0.01)) / 0.01;

        % Load the probability dataset for the unit interval.

        load('DMSAug.mat', 'PN');

        % Interpolate based on the first parameter.

        j = j * ones(N + 1, 1);
        y = y * ones(N + 1, 1);

        C00 = (1 - x) .* PN(sub2ind(size(PN), i, j, k)) +...
            x .* PN(sub2ind(size(PN), i + 1, j, k));
        C01 = (1 - x) .* PN(sub2ind(size(PN), i, j, k + 1)) +...
            x .* PN(sub2ind(size(PN), i + 1, j, k + 1));
        C10 = (1 - x) .* PN(sub2ind(size(PN), i, j + 1, k)) +...
            x .* PN(sub2ind(size(PN), i + 1, j + 1, k));
        C11 = (1 - x) .* PN(sub2ind(size(PN), i, j + 1, k + 1)) +...
            x .* PN(sub2ind(size(PN), i + 1, j + 1, k + 1));

        % Interpolate based on the second parameter.

        C0 = (1 - y) .* C00 + y .* C10;
        C1 = (1 - y) .* C01 + y .* C11;

        % Interpolate based on the third parameter to obtain the
        % probability.

        P = (1 - z) .* C0 + z .* C1;
        P(1 : ZI) = 0;
        
    else
        
        % For the case where n > 1, that is, when multiple lane changes
        % are required, the probability is obtained recursively using a
        % convolution integral. The derivative in the formula can be moved
        % outside the integral.

        N = 2 ^ (floor(log2(dE + 1)));

        P1 = [LaneChangeProbCalc(dE, [0; dM(n + 1)], v(n : n + 1),...
            [0; mu(n + 1)], [0; sigma(n + 1)], [0; tLC(n + 1)], 1, 2);...
            zeros(N, 1)];
        P2 = [LaneChangeProbCalc(dE, dM(1 : n), v(1 : n), mu(1 : n),...
            sigma(1 : n), tLC(1 : n), n - 1, 2); zeros(N, 1)];
        
        % Convolution is performed by taking the FFT of each function,
        % element-wise multiplication of the results, and then taking the
        % inverse FFT to obtain the results. This is much faster than
        % calculating the convolution directly.

        PC = ifft(fft(P1) .* fft(P2));
        PD = PC(1 : N + 1);
        
        % Calculate the derivate that was moved outside the integral.

        if m == 1
            P = PD(N + 1) - PD(N);
        else

            P = zeros(N + 1, 1);

            for i = 1 : 1 : N + 1
                if i == 1
                    P(i) = PD(i + 1) - PD(i);
                elseif i == N + 1
                    P(i) = PD(i) - PD(i - 1);
                else
                    P(i) = (PD(i + 1) - PD(i - 1)) / 2;
                end
            end
        end
    end
end
