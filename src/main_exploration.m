%% %%%%%%%%%%%%%%%
% CONFIGURATIONS %
%%%%%%%%%%%%%%%%%%

for batch = 1: 3
    for obj = 1:4       
        for trial = 1:10
            configuration;
            dbstop if error;

            clc;

            batchExperiments_all = {'00','01','10','11'};
            objectNames = {'finger_tip', 'apc_red_bowl', 'apc_2x2', 'bunny'};
            batchExperiments = batchExperiments_all(batch);
            global objectName
            objectName = objectNames{1, obj};

            global ros_on
            ros_on = true; % true: MuJoCo+MATLAB, communicate with ros
            % ros_on = false; % MATLAB only

            global ifMovie; ifMovie = false; % If visualize hand exploration process as movie or not

            global ifRecord; ifRecord = false; % record avi file

            global maxExplorationSteps; maxExplorationSteps = 200;

            global testObjectiveFunction; testObjectiveFunction = true; % if testing the new objective function

            global reassignTargetsForFingers; reassignTargetsForFingers = false;

            global explorationChance; explorationChance = 2; % probability of taking a fully random exploration step

            global jointWeightMatrix;
            jointWeightMatrix = [1.5, 1.1, 1.2, 1.3]; % weighting on the joint damping terms

            global slackWeightMatrix;
            slackWeightMatrix = [1, 2, 2, 2]; % weighting on the slack variables of different finger targets
            slackWeightMatrix = slackWeightMatrix / sum(slackWeightMatrix) * 4;

            global projectNextContacts
            projectNextContacts = true; % for MATLAB only simulation, project reference targets on the object surface to make sure that the target can be achieved in matlab.
            % projectNextContacts = false; % for Simulation with MuJoCo. Direcly use projected reference without projecting to the object surface. MuJoCo will simulated the real contact points.
            
            listObjects = {'bunny','apc_red_bowl','apc_1','apc_2','apc_redcup'};
            saveFrameStep = 5; % index of frames to be saved as figures, 1:N:end

            % objectName = 'apc_red_bowl';
            % objectName = 'apc_1';
            % objectName = 'apc_2';
            % objectName = 'apc_2x2';
            % objectName = 'apc_redcup';
            % objectName = 'bunny';
            % objectName = 'finger_tip';
            % rng('default')

            global scalingFactor % Scale object
            switch objectName
                case 'bunny'
                    scalingFactor = 2.0;
                case 'apc_red_bowl'
                    scalingFactor = 2.0;
                case 'apc_1'
                    scalingFactor = 2.5;
                case 'apc_2'
                    scalingFactor = 1.0;
                case 'apc_redcup'
                    scalingFactor = 2.0;
                case 'finger_tip'
                    scalingFactor = 0.5;
                case 'apc_2x2'
                    scalingFactor = 1.0;

                otherwise
                    error('Unknown object name.');
            end

            global experimentType
            experimentType = 'full'; % full simulation, incl. hand and algorithm. generate hand pose at each step
            % experimentType = 'gpr'; % No real hand involved. Just ignore the kinematic solution of the hand and test GPR models.
            % experimentType = 'mixed'; % only solve for hand kinematic configurations at key steps (steps that match `saveFrameStep`, to save the images)

            global ifGenSymEqCon ifGenSymIneqCon ifGenSymObjfun
            ifGenSymEqCon = true; % if re-generate symbolic equality constraints
            ifGenSymIneqCon = true; % if regenerate symbolic inequality constraints
            ifGenSymObjfun = true; % if regenerate symbolic objective function

            ifGenSymExpressions = ifGenSymEqCon || ifGenSymIneqCon || ifGenSymObjfun; % regenerate all symbolic expressions

            global explorationStep % step size
            explorationStep = 50; % 20; % 10mm,  30 

            global d_epsilon
            d_epsilon = explorationStep/10; % 5mm, remove the points inside the epsilon closure, to avoid exploring the neighbor points

            global greyColor
            greyColor = 0.7*ones(1,3);

            %%% 1st digit: optimizeHandPose, 2nd digit: optimizeFingers
            % batchExperiments = {'00','01','10','11'};
            % batchExperiments = {'01'};

            LogicalStr = {'false','true'};
            global paperExperimentType % designed experiments for paper
            global optimizeHandPose % +/- hand pose regulation (consider hand quaternion)
            global optimizeFingers % +/- finger optimization (in objective function)

            %%% Variables to save computational cost
            global cost_fun; cost_fun = struct();
            global t_fmincon; t_fmincon = []; % time spent on solving fmincon
            % global t_variables; t_variables = []; % time spent on initializing variables
            % global t_symbolic; t_symbolic = theta[]; % time spent on creating variables
            % global t_analysis; t_analysis = []; % time spent on analyzing results

            %%%%%%%%%%%%%%%%%
            % Initilize ROS %
            %%%%%%%%%%%%%%%%%
            if ros_on
                rosshutdown
                setenv('ROS_MASTER_URI','http://xiao:11311/') % please run roscore before this file
                rosinit
                global pose_joints_pub tips_normal_pub contacts_sub if_contacts_feedback start_end_flag_pub
                pose_joints_pub = rospublisher("/pose_joints","std_msgs/Float64MultiArray");
                tips_normal_pub = rospublisher("/tips_normal","std_msgs/Float64MultiArray");
                start_end_flag_pub = rospublisher("/start_end_flag","std_msgs/Float64MultiArray");
                contacts_sub = rossubscriber("/contacts","std_msgs/Float64MultiArray", "BufferSize",1);
                if_contacts_feedback = true;
                pause(3); % Wait to ensure publisher is registered
            end

            for iE = 1:length(batchExperiments)
                paperExperimentType = batchExperiments{iE};
                optimizeHandPose = str2num(paperExperimentType(1));
                optimizeFingers = str2num(paperExperimentType(2));

                fprintf('***** Experiment Type: %s, Hand pose opt.: %s, Fingers opt.: %s *****\n', paperExperimentType, LogicalStr{optimizeHandPose+1}, LogicalStr{optimizeFingers+1});

                run_problem;

                % Analyze objective function terms
    %             if ~isempty(cost_fun)
    %                 nSteps = cost_fun.nSteps;
    % 
    %                 % Joint damping terms
    %                 if isfield(cost_fun,'joints')
    %                     plot_value_and_time(cost_fun.joints.val, cost_fun.joints.t, 'ObjFun term: damping of joint changes');
    %                 end
    % 
    %                 % Objective function term: slack damping terms
    %                 if isfield(cost_fun,'slack')
    %                     plot_value_and_time(cost_fun.slack.val, cost_fun.slack.t, 'ObjFun term: damping of slack variables');
    %                 end
    % 
    %                 % Cost term: isotropy
    %     %             if isfobjectNameield(cost_fun,'isotropy')
    %                     plot_value_and_time(cost_fun.isotropy.val, cost_fun.isotropy.t, 'ObjFun term: isotropy value');
    %                 end
    % 
    %                 % Objective funtion term: omega
    %                 if isfield(cost_fun,'omega')
    %                     plot_value_and_time(cost_fun.omega.val, cost_fun.omega.t, 'ObjFun term: omega value');
    %                 end
    % 
    %                 % Objective function term: variance gradient
    %                 if isfield(cost_fun,'variance') && testObjectiveFunction
    %                     plot_value_and_time(cost_fun.variance.val, cost_fun.variance.t, 'ObjFun term: gradient of variance');
    %                 end
    %             end
    % 
    %             % Analysis of computational time for solving fmincon at each exploration step
    %             if ~isempty(t_fmincon)
    %                 figure, hold on;
    %                 plot(t_fmincon);
    %                 grid on;
    %                 xlabel('Exploration Step');
    %                 ylabel('Time Spent (s)');
    %                 title(sprintf('Computational Time of Each Step: %d +/- %d', mean(t_fmincon), std(t_fmincon)));
    %                 % fprintf('\nSolving fmincon time: %d +/- %d\n', mean(t_fmincon), std(t_fmincon));
    %                 hold off;
    %             end
    % 
    %             % Plot the comparison of computational time
    %             compare_computational_time(cost_fun);
    % 
    %             % Save results to mat files
    %             save(sprintf('ET-%s_OH-%d_OF-%d_performance.mat',paperExperimentType,optimizeHandPose,optimizeFingers),'cost_fun','t_fmincon');


            %%%% send a message to end MuJoCo running, then start a new trial
                if ros_on
                    ros_msg = rosmessage(start_end_flag_pub);
                    ros_msg.Data = [0,1]';
                    send(start_end_flag_pub, ros_msg);
                end
            end
        
        end
    end
end

% function plot_value_and_time(val, t, fig_title)
%     figure, hold on;
%     subplot(2,1,1);
%     plot(val);
%     grid on;
%     xlabel('Exploration Step');
%     ylabel('Objective Term Value');
%     title('Value');
% 
%     subplot(2,1,2);
%     plot(t);
%     grid on;
%     xlabel('Exploration Step');
%     ylabel('Computational Time (s)');
%     title('Time');
% 
%     sgtitle(fig_title);
%     hold off;
% end