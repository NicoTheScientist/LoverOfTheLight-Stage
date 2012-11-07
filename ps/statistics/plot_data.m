clear;

% read from files
fitness_file_CS = fopen('fitnessCS.txt');
fitness_file_FA = fopen('fitnessFA.txt');

% plotting fitness for CS
figure(1);
t = 1;
[val count] = fscanf(fitness_file_CS,'%f', 1);
disp([val,' aaaa ',count]);
while count ~= 0
    i = 1;
    fitness = 0;
    while val ~= -1 && count ~= 0
        fitness(i) = val;
        i = i + 1;
        [val count] = fscanf(fitness_file_CS,'%f', 1);
        disp([val,' aaa ',count]);
    end
    average_fitness(t) = mean(fitness);
    t = t + 1;
end
plot( 1 : num_eval , average_fitness, 'b-');
hold on;
title('average fitness evolution in time for CS');
xlabel('time');
ylabel('fitness');

% % plotting average fitness for FA
% figure(2);
% for t = 1 : num_eval
%     [val count] = fscanf(fitness_file_FA,'%f', 1);
%     i = 1;
%     fitness = 0;
%     while val ~= -1 && count ~= 0
%         fitness(i) = val;
%         i = i + 1;
%         [val count] = fscanf(fitness_file_FA,'%f', 1);
%     end
%     average_fitness(t) = mean(fitness);
% end
% plot( 1 : num_eval , average_fitness, 'r-');
% hold on;
% title('average fitness evolution in time for FA');
% xlabel('time');
% ylabel('fitness');
% 
% % plotting average fitness for CS
% fclose(fitness_file_CS);
% fclose(fitness_file_FA);
% fitness_file_CS = fopen('fitness_CS.txt');
% fitness_file_FA = fopen('fitness_FA.txt');
% figure(3);
% for t = 1 : num_eval
%     
%     % count CS
%     [val count] = fscanf(fitness_file_CS,'%f', 1);
%     num_CS = 0;
%     while val ~= -1 && count ~= 0
%         num_CS = num_CS + 1;
%         [val count] = fscanf(fitness_file_CS,'%f', 1);
%     end
%     num_CS_time(t) = num_CS;
%     
%     % count FA
%     [val count] = fscanf(fitness_file_FA,'%f', 1);
%     num_FA = 0;
%     while val ~= -1 && count ~= 0
%         num_FA = num_FA + 1;
%         [val count] = fscanf(fitness_file_FA,'%f', 1);
%     end
%     num_FA_time(t) = num_FA;
% end
% plot( 1 : num_eval , num_CS_time, 'b-');
% hold on;
% plot( 1 : num_eval , num_FA_time, 'r-');
% title('number of CS and FA evolution in time');
% xlabel('time');
% ylabel('number of agents');