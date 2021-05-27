%% Probability Model Validation

% This script numerically validates the results of the probability model.
% To do this, at first several hundred samples are taken from a log-normal
% distribution with values matching the traffic data and our model. Then,
% the cumulative sum of those values is formed (essentially stacking them
% on top of one another) so they represent vehicle locations along the road.
% Next, a starting point is selected at random and we examine whether there
% is a gap larger than the minimum safe distance between the vehicles in the
% interval specified by the relative search distance. Repeating this process
% thousands of times allows us to numerically determine the probability and
% validate the probability model. The process is repeated for both cases
% with two and three lanes to check both the interpolator code and the
% convolution integral code.

% Author: Goodarz Mehr
% Email: goodarzm@vt.edu
% First written: 8/20/2019
% Last revision: 12/16/2019
% Published Under GPL-2.0.

% Define parameters.

dET = 3200; % Ego vehicle distance from the goal state for a test point (m).
v = [34; 33; 32]; % Velocity of vehicles in each lane (m/s).
lv = 5; % Length of a vehicle (m).
dS = 2; % Minimum safe distance constant (m).
delta = 2; % Lane velocity multiplier for minimum safe distance (s).
muT = [0; 1.15; 1.1]; % Headway time distribution parameter (mu) (s).
sigma = [0; 0.65; 0.6]; % Headway time distribution parameter (sigma) (s).
dM = zeros(3, 1); % Minimum safe distance array.
mu = zeros(3, 1); % Headway distance distribution parameter (mu) array.

% Calculate dM and mu.

for i = 2 : 1 : 3
    dM(i) = dS + lv + delta * v(i);
    mu(i) = muT(i) + log(v(i));
end

% First, let's look at the case for two lanes.

% Create a log-normal distribution.

P1N = zeros(161, 1);
PLN1 = makedist('Lognormal', 'mu', mu(2), 'sigma', sigma(2));

parfor k = 1 : 161
    
    % Define parameters.
    
    dET = (k - 1) * 20;

    if v(1) >= v(2)
        dR = dET * (1 - v(2) / v(1));
    else
        dR = dET * (v(2) / v(1) - 1);
    end

    dSR = dR + dM(2); % Relative search distance.
    sumF = 0;
    
    % Sample random values from the log-normal distribution and form the
    % cumulative sum.

    D1 = cumsum(random(PLN1, 400, 100000), 1);
    
    % Randomly assign the search start and finish points.

    Start = rand(1, 100000) .* D1(300, 100000);
    Finish = Start + dSR;

    for j = 1 : 1 : 100000

        % Look at the inter-vehicle distances in the interval defined by
        % start and finish.
        
        D1cut = D1(:, j);
        D1cut = [Start(j); D1cut(Start(j) <= D1cut &...
            D1cut <= Finish(j)); Finish(j)];
        D1diff = diff(D1cut);
        
        % Check if there is a gap larger than the minimum safe distance.

        if max(D1diff) >= dM(2)
            sumF = sumF + 1;
        end
    end
    
    % Calculate the probability.
    
    P1N(k) = sumF / 100000;
    
end

% Now, let's consider three lanes.

% Create two log-normal distributions, one for each lane.

P23N = zeros(161, 1);
PLN2 = makedist('Lognormal', 'mu', mu(2), 'sigma', sigma(2));
PLN3 = makedist('Lognormal', 'mu', mu(3), 'sigma', sigma(3));

parfor k = 1 : 1 : 161
    
    % Define parameters.
    
    dET = (k - 1) * 20;

    if v(1) >= v(2)
        dR = dET * (1 - v(2) / v(1));
    else
        dR = dET * (v(2) / v(1) - 1);
    end

    dSR = dR + dM(2); % Relative search distance.
    sumF = 0;
    
    % Sample random values from the log-normal distribution for each lane
    % and form the cumulative sum.

    D2 = cumsum(random(PLN2, 400, 100000), 1);
    D3 = cumsum(random(PLN3, 400, 100000), 1);

    % Randomly assign the search start and finish points.
    
    Start = rand(1, 100000) .* D2(240, 100000);
    Finish = Start + dSR;

    for j = 1 : 1 : 100000

        % Look at the inter-vehicle distances in the second lane in the
        % interval defined by start and finish.
        
        D2cut = D2(:, j);
        D2cut = [Start(j); D2cut(Start(j) <= D2cut &...
            D2cut <= Finish(j)); Finish(j)];
        D2diff = diff(D2cut);
        
        % Check if there is a gap larger than the minimum safe distance. If
        % so, move on to analyzing the third lane.

        if max(D2diff) >= dM(2)

            % Find where the first such gap in the second lane is relative
            % to the start point.
            
            D2GapIdxArray = find(D2diff >= dM(2));
            D2FirstGapIdx = D2GapIdxArray(1);
            D2FirstGapRelDist = D2cut(D2FirstGapIdx) - Start(j);
            
            % Calculate the absolute distance traveled based on the
            % relative distance from the start point in the second lane.
            
            if v(1) >= v(2)
                D2FirstGapAbsDist = D2FirstGapRelDist / (1 - v(2) / v(1));
            else
                D2FirstGapAbsDist = D2FirstGapRelDist / (v(2) / v(1) - 1);
            end
            
            % Calculate the relative distance traveled in the third lane
            % based on the absolute distance traveled.
            
            if v(1) >= v(3)
                D3RelPassDist = D2FirstGapAbsDist * (1 - v(3) / v(1));
            else
                D3RelPassDist = D2FirstGapAbsDist * (v(3) / v(1) - 1);
            end
            
            % Calculate the remaining distance before reaching dE in the
            % second lane relative to the third lane.
            
            if v(2) >= v(3)
                D3RelRemainDist = (dET - D2FirstGapAbsDist) *...
                    (1 - v(3) / v(2));
            else
                D3RelRemainDist = (dET - D2FirstGapAbsDist) *...
                    (v(3) / v(2) - 1);
            end
            
            % Look at the inter-vehicle distances in the third lane in the
            % interval defined above.
            
            D3cut = D3(:, j);
            D3cut = [Start(j) + D3RelPassDist + (dM(2) - dM(3)) / 2;...
                D3cut(Start(j) + D3RelPassDist +...
                (dM(2) - dM(3)) / 2 <= D3cut & D3cut <= Start(j) +...
                D3RelPassDist + D3RelRemainDist + (dM(2) + dM(3)) / 2);...
                Start(j) + D3RelPassDist + D3RelRemainDist + (dM(2) +...
                dM(3)) / 2];
            D3diff = diff(D3cut);
            
            % Check if there is a gap larger than the minimum safe
            % distance.
            
            if max(D3diff) >= dM(3)
                sumF = sumF + 1;
            end
        end
    end
    
    % Calculate the probability.

    P23N(k) = sumF / 100000;
    
end
