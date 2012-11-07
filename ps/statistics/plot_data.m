clear;

% read from files
fitness_file_CS = fopen('fitnessCS.txt');
fitness_file_FA = fopen('fitnessFA.txt');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% plotting fitness for CS %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = 0;
count = 1;
while count ~= 0
    t = t + 1;
    i = 1;
    fitness = 0;
    [val count] = fscanf(fitness_file_CS,'%f', 1);
    while count ~= 0 && val ~= -1
        fitness(i) = val;
        i = i + 1;
        [val count] = fscanf(fitness_file_CS,'%f', 1);
    end
    sort(fitness);
    best_fitness(t) = fitness(end);
    average_fitness(t) = mean(fitness);
    worst_fitness(t) = fitness(1);
end
figure(1);
hold on;
plot( 1 : t , best_fitness, 'g-');
plot( 1 : t , average_fitness, 'b-');
plot( 1 : t , worst_fitness, 'r-');
title('fitness evolution in time for Candidate Solutions');
xlabel('time');
ylabel('fitness');
leg1 = legend('best case','average','worst case');
set(leg1,'Location','NorthWest');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%% plotting fitness for FA %%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = 0;
count = 1;
while count ~= 0
    t = t + 1;
    i = 1;
    fitness = 0;
    [val count] = fscanf(fitness_file_FA,'%f', 1);
    while count ~= 0 && val ~= -1
        fitness(i) = val;
        i = i + 1;
        [val count] = fscanf(fitness_file_FA,'%f', 1);
    end
    sort(fitness);
    best_fitness(t) = fitness(end);
    average_fitness(t) = mean(fitness);
    worst_fitness(t) = fitness(1);
end
figure(2);
hold on;
plot( 1 : t , best_fitness, 'g-');
plot( 1 : t , average_fitness, 'b-');
plot( 1 : t , worst_fitness, 'r-');
title('fitness evolution in time for Fate Agents');
xlabel('time');
ylabel('fitness');
leg1 = legend('best case','average','worst case');
set(leg1,'Location','NorthWest');



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%% plotting number of CS and FA %%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

fclose(fitness_file_CS);
fclose(fitness_file_FA);
fitness_file_CS = fopen('fitnessCS.txt');
fitness_file_FA = fopen('fitnessFA.txt');

t = 0;
count = 1;
while count ~= 0
    t = t + 1;
    %count CS
    num_CS = 0;
    [val count] = fscanf(fitness_file_CS,'%f', 1); 
    while count ~= 0 && val ~= -1
        num_CS = num_CS + 1;
        [val count] = fscanf(fitness_file_CS,'%f', 1);
    end
    num_CS_time(t) = num_CS;
    %count FA
    num_FA = 0;
    [val count] = fscanf(fitness_file_FA,'%f', 1); 
    while count ~= 0 && val ~= -1
        num_FA = num_FA + 1;
        [val count] = fscanf(fitness_file_FA,'%f', 1);
    end
    num_FA_time(t) = num_FA;
end
figure(3);
hold on;
plot( 1 : t , num_CS_time, 'b-');
plot( 1 : t , num_FA_time, 'r-');
title('number of CS and FA evolution in time');
xlabel('time');
ylabel('number of agents');
leg1 = legend('candidate colutions','fate agents');
set(leg1,'Location','NorthEast');