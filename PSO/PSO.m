clear all
close all
clc

% Define the details of the objective function
nVar = 10;
ub = [10 10 10 10 10 10 10 10 10 10]; 
lb = [-10 -10 -10 -10 -10 -10 -10 -10 -10 -10 ];
fobj = @ObjectiveFunction;

% PSO's parameters 
noP = 300;
maxIteration = 500;
wMax = 0.9;
wMin = 0.2;
c1 = 2;
c2 = 2;

% The PSO algorithm 

% Initialization of particles 
for k = 1 : noP 
    Swarm.Particles(k).X = (ub - lb) .* rand(1,nVar) + lb;
    Swarm.Particles(k).V = zeros(1,nVar);
    Swarm.Particles(k).PBEST.X = zeros(1, nVar);
    Swarm.Particles(k).PBEST.O = inf; % for minimization 
end

Swarm.GBEST.X = zeros(1,nVar);
Swarm.GBEST.O = inf;

% main loop 
for t = 1 : maxIteration 
    
    for k = 1 : noP
        currentX = Swarm.Particles(k).X;
        currentO = fobj ( currentX );
        
        if currentO <  Swarm.Particles(k).PBEST.O  % Update PBEST
            Swarm.Particles(k).PBEST.X = currentX;
            Swarm.Particles(k).PBEST.O = currentO;
        end
        
        if  currentO < Swarm.GBEST.O   % Update GBEST
            Swarm.GBEST.X = currentX;
            Swarm.GBEST.O = currentO;
        end     
    end
    
    w = wMax - t .* ( (wMax - wMin) / maxIteration);
    
    for k = 1 : noP
        Swarm.Particles(k).V = w .* Swarm.Particles(k).V + c1 .* rand(1, nVar) .* ( Swarm.Particles(k).PBEST.X - Swarm.Particles(k).X) ...
                                                                                     + c2 .* rand(1,nVar) .*  (Swarm.GBEST.X - Swarm.Particles(k).X) ;     
        Swarm.Particles(k).X  = Swarm.Particles(k).X + Swarm.Particles(k).V;                                                                 
                                                                                 
    end
    disp(Swarm.GBEST.O)
    
    cgCurve(t) = Swarm.GBEST.O;
    
end

semilogy(cgCurve)
    







    
    
    
    
    
    
    
    
    
    

