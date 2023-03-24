%% Preamble
close all;
clear all;
clc;

%% Simulation setup
% Time
simulationTime_total = 170;           
stepSize_time = 0.05;               

% Initial states and controls
voltage_left  = 6;                 
voltage_right = 6;                  

state_initial = zeros(1,24);        
                                    
% Environment
canvasSize_horizontal = 10.01;
canvasSize_vertical   = 10.01;
stepSize_canvas       = 0.01;


fiscontrol1 = readfis('HA_C1_C2_C3');
fiscontrol2 = readfis('OA_C1_C2_C3_C4_C5');
fiscontrol3 = readfis('HA_C4-C5');
fiscontrol4 = readfis('OA_C5');
fiscontrol5 = readfis('HA_C5');


tolerance = 0.05;

checkpoints = [1.5 4.5; -1 2; -4 -1; -1 -4; 4 0; 0 0];

checkpoint = [0 0];

counter = 1;






%% Create Environment
obstacleMatrix = zeros(canvasSize_horizontal / stepSize_canvas, canvasSize_vertical / stepSize_canvas);

% Generate walls

% outside walls
[wall_1, obstacleMatrix] = WallGeneration( -5, 5, 5, 5, 'h', obstacleMatrix);
[wall_2, obstacleMatrix] = WallGeneration( -5, -5, -5, 5, 'h', obstacleMatrix);
[wall_3, obstacleMatrix] = WallGeneration( -5, -5, -5, 5, 'v', obstacleMatrix);
[wall_4, obstacleMatrix] = WallGeneration( 5, 5, -5, 5, 'v', obstacleMatrix);
% Obstacles
[wall_5, obstacleMatrix] = WallGeneration( -2.5, 2.5, 2.5, 2.5, 'h', obstacleMatrix);
[wall_6, obstacleMatrix] = WallGeneration( -3, -3, -3, 0.5, 'v', obstacleMatrix);
[wall_7, obstacleMatrix] = WallGeneration( 1, 1, 0, 0, 'h', obstacleMatrix);
[wall_8, obstacleMatrix] = WallGeneration( 1, 1, -2, 0, 'v', obstacleMatrix);



%% Main simulation
% Initialize simulation 
timeSteps_total = simulationTime_total/stepSize_time;
state = state_initial;
time = 0;

% Run simulation
for timeStep = 1:timeSteps_total

    sensorOut = Sensor(state(timeStep,19), state(timeStep,20), state(timeStep,24), obstacleMatrix);

    currentLocation = [state(timeStep,19), state(timeStep,20)];

    checkpoint = checkpoints(counter,:);

    [booleanAtCheckpoint, newHeadingAngle] = ComputeHeadingAngle(currentLocation, checkpoint, tolerance);


    if booleanAtCheckpoint == 1
       counter = counter + 1;
     end

    if counter == 7
      break
    end


HAerror = wrapToPi(state(timeStep,24)-newHeadingAngle);


if counter == 4

       HAoutput = evalfis(fiscontrol3, HAerror);
   voltage_left_1 = HAoutput(:,1);
   voltage_right_1 = HAoutput(:,2);

   WALLoutput = evalfis(fiscontrol2, sensorOut);
   voltage_left_2 = WALLoutput(:,1);
   voltage_right_2 = WALLoutput(:,2);

   voltage_left = voltage_left_1 + voltage_left_2;
   voltage_right = voltage_right_1 + voltage_right_2;

   voltages = [voltage_left; voltage_left; voltage_right; voltage_right];


else if counter == 5
         HAoutput = evalfis(fiscontrol3, HAerror);
   voltage_left_1 = HAoutput(:,1);
   voltage_right_1 = HAoutput(:,2);

   WALLoutput = evalfis(fiscontrol2, sensorOut);
   voltage_left_2 = WALLoutput(:,1);
   voltage_right_2 = WALLoutput(:,2);

   voltage_left = voltage_left_1 + voltage_left_2;
   voltage_right = voltage_right_1 + voltage_right_2;

   voltages = [voltage_left; voltage_left; voltage_right; voltage_right];


else if counter == 6
         HAoutput = evalfis(fiscontrol5, HAerror);
   voltage_left_1 = HAoutput(:,1);
   voltage_right_1 = HAoutput(:,2);

   WALLoutput = evalfis(fiscontrol4, sensorOut);
   voltage_left_2 = WALLoutput(:,1);
   voltage_right_2 = WALLoutput(:,2);

   voltage_left = voltage_left_1 + voltage_left_2;
   voltage_right = voltage_right_1 + voltage_right_2;

   voltages = [voltage_left; voltage_left; voltage_right; voltage_right];

else
   HAoutput = evalfis(fiscontrol1, HAerror);
   voltage_left_1 = HAoutput(:,1);
   voltage_right_1 = HAoutput(:,2);

   WALLoutput = evalfis(fiscontrol2, sensorOut);
   voltage_left_2 = WALLoutput(:,1);
   voltage_right_2 = WALLoutput(:,2);

   voltage_left = voltage_left_1 + voltage_left_2;
   voltage_right = voltage_right_1 + voltage_right_2;

   voltages = [voltage_left; voltage_left; voltage_right; voltage_right];


end
end
end 

    
    % Run model *** DO NOT CHANGE
    [state_derivative(timeStep,:), state(timeStep,:)] = DynamicalModel(voltages, state(timeStep,:), stepSize_time);   
    
    % Euler intergration *** DO NOT CHANGE
    state(timeStep + 1,:) = state(timeStep,:) + (state_derivative(timeStep,:) * stepSize_time); 
    time(timeStep + 1)    = timeStep * stepSize_time;

    % Plot robot on canvas  
    figure(1); clf; hold on; grid on; axis([-5.5,5.5,-5.5,5.5]);
    DrawRobot(0.2, state(timeStep,20), state(timeStep,19), state(timeStep,24), 'b');
plot(wall_1(:,1), wall_1(:,2),'k-');
plot(wall_2(:,1), wall_2(:,2),'k-');
plot(wall_3(:,1), wall_3(:,2),'k-');
plot(wall_4(:,1), wall_4(:,2),'k-');
plot(wall_5(:,1), wall_5(:,2),'k-');
plot(wall_6(:,1), wall_6(:,2),'k-');
plot(wall_7(:,1), wall_7(:,2),'k-');
plot(wall_8(:,1), wall_8(:,2),'k-');
plot(checkpoints(:,2), checkpoints(:,1), 'k--d', 'LineWidth',1.5)
    xlabel('y, m'); ylabel('x, m');
    
end

%% Plot results


figure(2); hold on; grid on; axis([-5.5,5.5,-5.5,5.5]);
plot(state(:,20), state(:,19),'LineWidth',1.5);
xlabel('y-position (m)', 'FontSize',16);
ylabel('x-position (m)', 'FontSize',16);
plot(wall_1(:,1), wall_1(:,2),'k-','LineWidth',1.5);
plot(wall_2(:,1), wall_2(:,2),'k-','LineWidth',1.5);
plot(wall_3(:,1), wall_3(:,2),'k-','LineWidth',1.5);
plot(wall_4(:,1), wall_4(:,2),'k-','LineWidth',1.5);
plot(wall_5(:,1), wall_5(:,2),'k-','LineWidth',1.5);
plot(wall_6(:,1), wall_6(:,2),'k-','LineWidth',1.5);
plot(wall_7(:,1), wall_7(:,2),'k-','LineWidth',1.5);
plot(wall_8(:,1), wall_8(:,2),'k-','LineWidth',1.5);
set(gca,'FontSize',20);
plot(checkpoints(:,2), checkpoints(:,1), 'k--d', 'LineWidth',1.5)
title('x-y position','FontSize',16);
yticks([-5 -4 -3 -2 -1 0 1 2 3 4 5]);
xticks([-5 -4 -3 -2 -1 0 1 2 3 4 5]);



