function compare_computational_time(cost_fun)
    
    % Number of cost terms
    listOfCosts = fieldnames(cost_fun);
    N = length(listOfCosts)-1; % number of costs, excl: nSteps
    
    figure, hold on;
    idx = 0;
    ymin = 1e10;
    ymax = 1e-10;
    AX = zeros(1,N);
    for i = 1:N+1
        costName = listOfCosts{i};
        if strcmp(costName, 'nSteps')
            continue;
        else
            idx = idx + 1;
            costField = cost_fun.(costName);
            t = costField.t;
            idx_outlier = isoutlier(t);
            ax = subplot(1,N,idx);
            AX(idx) = ax;
            hold on;
            boxplot(t(~idx_outlier));
            grid on;
            title(costName,'FontSize',12);
            YLim = ylim;
            if ymin > YLim(1)
                ymin = YLim(1);
            end
            if ymax < YLim(2)
                ymax = YLim(2);
            end
        end
    end
    
    linkaxes(AX, 'xy');
    % sgtitle('Time Spent on Each Objective Term (Excl. Outliers)');
end