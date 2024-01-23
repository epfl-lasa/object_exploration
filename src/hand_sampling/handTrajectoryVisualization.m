% This function visualizes the transformation from the oldHand to the
% newHand by linearly interpolating the joint angles, positions, and quaternions.

function [M] = handTrajectoryVisualization(oldHand, newHand, timeSteps, objectCloud, sampledCloud, vecN, vecT, this_contacts, next_contacts)
    % ptClouds: (N,3)
    % vecN: (4,3), surface normal direction
    
    global plotConfig scalingFactor greyColor fingerColors
    if isempty(scalingFactor)
        scalingFactor = 1.0;
    end
    
    if isempty(greyColor)
        greyColor = 0.7*ones(1,3);
    end
    
    q_old = oldHand.q;
    T_old = oldHand.T;
    position_old = T_old(1:3,4);
    quat_old = quaternion(rotm2quat(T_old(1:3,1:3)));

    q_new = newHand.q;
    T_new = newHand.T;
    position_new = T_new(1:3,4);
    quat_new = quaternion(rotm2quat(T_new(1:3,1:3)));

    if nargout > 0
        loops = timeSteps;
        M(loops) = struct('cdata',[],'colormap',[]);
    end
    
    % figure, hold on;
    for i = 1:timeSteps
        clf;
        q_tmp = q_old + i*(q_new - q_old)/timeSteps;
        
        position_tmp = position_old + i*(position_new - position_old)/timeSteps;
        quat_tmp = slerp(quat_old, quat_new, i/timeSteps);
        T_tmp = [quat2rotm(quat_tmp), position_tmp; 0, 0, 0, 1];
        
        H = updateHandTransformationForVisualization(oldHand, T_tmp, q_tmp);
        mySGplotHand(H);
        hold on;
        
        %% Visualize object (point cloud)
        if nargin >= 4
            % scatter3(objectCloud(:,1), objectCloud(:,2), objectCloud(:,3), 10*scalingFactor, 'o', greyColor);
            scatter3(objectCloud(:,1), objectCloud(:,2), objectCloud(:,3), 10*scalingFactor, 'MarkerFaceColor', greyColor, 'MarkerFaceAlpha', 0.2, 'MarkerEdgeColor', 'none', 'LineWidth', 0.5);
        end

        %% Visualize sampled points
        if nargin >= 5
            % scatter3(sampledCloud(:,1), sampledCloud(:,2), sampledCloud(:,3), 25*scalingFactor, [0 0.4470 0.7410], 'filled');
            scatter3(sampledCloud(1:4:end,1), sampledCloud(1:4:end,2), sampledCloud(1:4:end,3), 25*scalingFactor, 'r', 'filled');
            scatter3(sampledCloud(2:4:end,1), sampledCloud(2:4:end,2), sampledCloud(2:4:end,3), 25*scalingFactor, 'g', 'filled');
            scatter3(sampledCloud(3:4:end,1), sampledCloud(3:4:end,2), sampledCloud(3:4:end,3), 25*scalingFactor, 'b', 'filled');
            scatter3(sampledCloud(4:4:end,1), sampledCloud(4:4:end,2), sampledCloud(4:4:end,3), 25*scalingFactor, 'y', 'filled');
        end
        
        %{
        %% Visualize contact normal direction at the latest sampled points
        if nargin >= 6
            quiver3(sampledCloud(end-3,1), sampledCloud(end-3,2), sampledCloud(end-3,3), vecN(1,1), vecN(1,2), vecN(1,3));
            quiver3(sampledCloud(end-2,1), sampledCloud(end-2,2), sampledCloud(end-2,3), vecN(2,1), vecN(2,2), vecN(2,3));
            quiver3(sampledCloud(end-1,1), sampledCloud(end-1,2), sampledCloud(end-1,3), vecN(3,1), vecN(3,2), vecN(3,3));
            quiver3(sampledCloud(end,  1), sampledCloud(end,  2), sampledCloud(end,  3), vecN(4,1), vecN(4,2), vecN(4,3));
        end
        %}

        %% Visualize the tangential direction at the previous sampled points
        if nargin >= 8
            for iF = 1:oldHand.n
                iContact = this_contacts.positions(:,iF); % (3,1)
                iVecT = transpose(vecT(iF,:)); % (3,1)

                %{
                iVecEnd = iContact + iVecT*100;
                plot3([iContact(1) iVecEnd(1)],[iContact(2) iVecEnd(2)],[iContact(3) iVecEnd(3)],strcat(fingerColors{iF},'-'),'LineWidth',5);
                %}
                quiver3(iContact(1),iContact(2),iContact(3),iVecT(1),iVecT(2),iVecT(3),'Color',fingerColors{iF},'LineWidth',5);
            end
        end
    
        %% Visualize exploration targets for next step, same code as in 'run_problems.m'
        if nargin >= 9
            next_positions = next_contacts.positions;
            scatter3(next_positions(1,1),next_positions(2,1),next_positions(3,1),100*scalingFactor,'MarkerEdgeColor','r','Marker','o','LineWidth',3);
            scatter3(next_positions(1,2),next_positions(2,2),next_positions(3,2),100*scalingFactor,'MarkerEdgeColor','g','Marker','o','LineWidth',3);
            scatter3(next_positions(1,3),next_positions(2,3),next_positions(3,3),100*scalingFactor,'MarkerEdgeColor','b','Marker','o','LineWidth',3);
            scatter3(next_positions(1,4),next_positions(2,4),next_positions(3,4),100*scalingFactor,'MarkerEdgeColor','y','Marker','o','LineWidth',3);
        end

        % xlim(plotConfig.X);
        % ylim(plotConfig.Y);
        % zlim(plotConfig.Z);

        if ~isempty(plotConfig.View)
            view(plotConfig.View);
        end
        
        axis off;
        drawnow;
        if nargout > 0
            M(i) = getframe;
        end
    end
end