%% probability Model Numerical Database

% This script calculates a database of probability values for the unit
% interval transformed problem for different values of mu, sigma and d. To
% achieve this, for each combination of values of mu (between -5 and 1 with
% steps of 0.05), sigma (between 0 and 2 with steps of 0.05), and d (between
% 0 and 1 with steps of 0.01), vehicle positions in the destination lane are
% simulatied by first sampling hundreds of values from a log-normal
% distribution with parameters mu and sigma, then forming the cumulative sums
% of these values to resemble vehicle locations, then selecting a unit
% interval at random, and finally checking to see if there is a gap larger
% than d in that unit interval. This process is repeated for thousands of
% times for each combination of mu, sigma, and d, and the probability is
% calculated based on the results.
    
% Author: Goodarz Mehr
% Email: goodarzm@vt.edu
% First written: 8/4/2019
% Last revision: 12/16/2019
% Published Under GPL-2.0.

% Define parameters.

mu = -2; % Parameter used for defining log-normal distribution.
sigma = 0.4; % Parameter used for defining log-normal distribution.
d = 0.2; % Gap size.
N = 1000; % Number of simulations.

% Loop over different combinations of mu, sigma, and d.

PN = zeros(121, 40, 100);

for l = 1 : 1 : 121
    
    mu = -5 + (l - 1) * 0.05;
    
    for o = 1 : 1 : 40
        
        sigma = o * 0.05;
        PLN = makedist('Lognormal', 'mu', mu, 'sigma', sigma);

        parfor k = 1 : 100

            d = k * 0.01;
            sumF = 0;
            
            % Sample random values from the log-normal distribution and
            % form the cumulative sum.

            D = cumsum(random(PLN, ceil(400 /...
                (0.001 * (l - 1) ^ 2 + 1)), N), 1);
            
            % Randomly assign the search start point.

            Start = rand(1, N) .* D(floor(200 /...
                (0.001 * (l - 1) ^ 2 + 1)), N);
            Finish = Start + 1;

            for j = 1 : 1 : N

                % Look at the inter-vehicle distances in the unit interval
                % defined by start.
                
                Dcut = D(:, j);
                Dcut = [Start(j); Dcut(Start(j) < Dcut &...
                    Dcut <= Finish(j)); Finish(j)];
                Ddiff = diff(Dcut);
                
                % Check if there is a gap larger than the minimum safe
                % distance.

                if max(Ddiff) >= d
                    sumF = sumF + 1;
                end
            end
            
            % Calculate the probability.
            
            PN(l, o, k) = sumF / N;
            
        end
    end
end

% Save the database.

save('PN.mat', 'PN');
