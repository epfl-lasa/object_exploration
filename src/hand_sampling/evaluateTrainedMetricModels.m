function evaluateTrainedMetricModels(Model, testScriptName, jointAngles, ValueMap, rMap, extra_test, iF)
    % dataStruct: struct with samples
    % Model: trained GMM model
    % test_extra: boolean, if test extra samples
    % iF: index of finger
    
    configuration;
    
    %% GPR
    predicts = resubPredict(Model); % Needed for subplot 2
    assert(length(predicts)==length(jointAngles));
    
    nSamples = length(jointAngles);
    
    if extra_test % Needed for subplot 3
        %% Sample new points (need IK to sample spatial points)
        hand_file = load(fullfile(path_hand_model, 'AllegroHandLeft_model.mat'));
        hand = hand_file.hand;
        symFK = hand.F{iF}.Link{end}.symbolic.HT_next(1:3,4);
        Q = sym(strcat('q',num2str(iF),'%d'), [1,4]);
        
        nTest = min(500,nSamples);
        
        q = optimvar('q',[1,4],'Type','continuous','LowerBound',min(jointAngles,[],1),'UpperBound',max(jointAngles,[],1));
        LB = q.LowerBound;
        UB = q.UpperBound;
        
        test_X = zeros(nTest, 4);
        test_y = zeros(nTest, 1);
        test_positions = zeros(nTest, 3);

        for i = 1:nTest
            % fprintf('%d-%d\n', nTest, i);
            q_test = LB + (UB-LB).*rand(1,4);

            test_positions(i,:) = double(subs(symFK, Q, q_test));
            test_X(i,:) = q_test;
            
            switch testScriptName
                case 'oMT'
                    test_y(i) = omegaValueThumb(q_test);
                case 'oMF'
                    test_y(i) = omegaValueFinger(q_test);
                case 'iMT'
                    test_y(i) = isotropyValueThumb(q_test);
                case 'iMF'
                    test_y(i) = isotropyValueFinger(q_test);
                case 'model'
                    test_y(i) = predict(Model,q_test);
            end
        end
        
        
        %% Testing response at sample points (very symbolic script is correct)
        %{
        test_X = jointAngles; % (N,4)
        test_y = zeros(nSamples,1); % (N,1)
        test_positions = rMap;
        for i = 1:nSamples
            q = test_X(i,:);
            %% Evaluate testing points 
            switch modelType
                case 'omega'
                    if iF == 1
                        test_y(i) = omegaValueThumb(q);
                    else
                        test_y(i) = omegaValueFinger(q);
                    end
                case 'isotropy'
                    if iF == 1
                        test_y(i) = isotropyValueThumb(q);
                    else
                        test_y(i) = isotropyValueFinger(q);
                    end
            end
            % test_y(i) = predict(Model,q); % ground truth to verify
        end
        %}
        
    end
    
    %% Evaluation by plotting
    sz = 20;
    figure, hold on;
    subplot(1,3,1);
    scatter3(rMap(:,1),rMap(:,2),rMap(:,3),sz,ValueMap,'filled');
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Original samples');
    
    subplot(1,3,2);
    scatter3(rMap(:,1),rMap(:,2),rMap(:,3),sz,predicts,'filled'); % Not necessarily have correspondance between `predicts` and `rMap`
    axis equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Model prediction');
    
    if extra_test
        subplot(1,3,3);
        scatter3(test_positions(:,1),test_positions(:,2),test_positions(:,3),sz,test_y,'filled');
        axis equal;
        grid on;
        xlabel('X');
        ylabel('Y');
        zlabel('Z');
        title('Model prediction (extra test)');
    end
    
    sgtitle('Comparison of original samples and model prediction');
    hold off;
end

%% GMM [deprecated]
%{
predicts = zeros(size(Map));
for i = 1:nSamples
    predicts(i) = predictGMModel(Model, jointAngles(i,:));
    % fprintf('Error: %d\n', predicts(i) - Map(i));
end
%}