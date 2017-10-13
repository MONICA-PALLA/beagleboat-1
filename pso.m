clc;
clear;
close all;

%% Problem definition

CostFunction = @(x) Sphere(x);   % The goal in this case is to converge to origin (Global Best)
                                 % The boat should converge to the
                                 % destination

nVar = 2;          % X is a vector in 5 dimensional space in my case (IMU, GPS, Temp, Obs dist) of each boat or 2D
                    % No. of unknow (Desicion) Variables 

VarSize = [1 nVar];  % Matrix Size of Desicion variables of an individual particle

VarMin = -10;      % Lower bound of desicion variables in my case varies with different search space of different
                   % parameters for example communication range with wifi,
                   % GPS with the fence or boundary 
                   % Obstacle distance with boundary to 0 (worst case)
                   % depends on maximum safe distance 
                   % the question is how to make it parameter specific
VarMax = 10;       % Upper bound of desicion variables 



%% Parameters of PSO

MaxIt = 10;            % Maximum no. of iterations (I might have to run it till the goal is reached)

nPop = 4;              % Population Size (Swarm Size) (In my case it the no. of boats)

% Inertia Coefficient
w = 1;

%wdamp = 0.99            % Damping Ratio of Inertia Coefficient
c1 = 2;                 % Personal Acceleration Coefficient  (Need to research out from references) 
c2 = 2;                 % Global Acceleration Coefficient 

%% Initialization    

empty_particle.Position = [];       %% Structure definition of each particle
empty_particle.Velocity = [];       %% Orientation can be included
empty_particle.Cost = [];           %% Distance from the Obstacle can also be included
empty_particle.Best.Position = [];  %% Two fields for Best
empty_particle.Best.Cost = [];

% Create Population Array
particle = repmat(empty_particle, nPop, 1);

% Initialize Global best % this can't be defined till the members of the
% population are initialized.
%GlobalBest.Cost = inf;  % worst case value so inf if it is minimization problem and -inf for Maximization

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%GlobalBest.Cost = 0;
%GlobalBest.Position =  [0,0];

GlobalBest.Cost = Inf;

% because of improper execution of if statement in for loop

% Initialize Population members
for i=1:nPop 

    % Generate Random Solution 
    particle(i).Position = unifrnd(VarMin, VarMax, VarSize); % uniform random variable varying from Min to Max 
    % and Size of VarSize, creats the random position in the search space 
                                                         
    % Initialize Velocity
    particle(i).Velocity = zeros(VarSize);
    
    % Update the Personal Best
    particle(i).Best.Position = particle(i).Position;
    particle(i).Best.Cost = particle(i).Cost;
        
    % Evaluation
    particle(i).Cost = CostFunction(particle(i).Position);                            
    % In my case the cost can be distance from the goal, distance form the obstacle, neighbour or e.t.c.,
    
    % Update Global Best   
    if particle(i).Best.Cost < GlobalBest.Cost
       GlobalBest = particle(i).Best;
    end
    % In my case I need to define the goal if I'm not going to update

end

% Array to hold Best cost Value on Each iteration  
BestCosts = zeros(MaxIt,1);



%% Main Loop of PSo

for it=1:MaxIt
    
    for i=1:nPop
        
        % Update Velocity
        particle(i).Velocity = w*particle(i).Velocity ...
            + c1*rand(VarSize).*(particle(i).Best.Position - particle(i).Position) 
            + c2*rand(VarSize).*(GlobalBest.Position - particle(i).Position);
        
        % Update Position
        particle(i).Position = particle(i).Position + particle(i).Velocity;
        
        % Cost
        particle(i).Cost = CostFunction(particle(i).Position);
        
        
        % Update personal best
        if particle(i).Cost < particle(i).Best.Cost
            
            particle(i).Best.Position = particle(i).Postion;
            particle(i).Best.Cost = particle(i).Cost;
            
            % the code for updating the Global best as well must be written
            % here, not required in this case
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest = particle(i).Best;
            end
            %plot(particle(i).Position); 
        end
        
    end
    
    % Store the Best Cost Value
     BestCosts(it)=GlobalBest.Cost;
   
    % Display Iteration Information
    disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCosts(it))]);
    
   % w = w * wdamp;
    
    
end

%% 

%figure;
%plot(BestCosts, 'LineWidth', 2);
%%semilogy(BestCosts, 'LineWidth', 2);
%xlabel('Iteration');
%ylabel('Best Cost');
