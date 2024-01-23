function hand = mySGplotHand(hand, transp, if_transform)

    if nargin < 3
        if_transform = false; % When hand.T is not eye(4), it may be necessary to first calculate the transform of the hand
    end
    if nargin < 2
        transp = 0.8; % transparancy of plotting [0 for no color]
    end
    if nargin < 1
        hand = mySGparadigmatic;
    end

    if(~mySGisHand(hand))
        error 'hand argument is not a valid hand-structure' 
    end

    if if_transform
        if ~isequal(hand.T, eye(4))
            % Update hand model if this hand base is transformed
            hand = updateHandTransformationForVisualization(hand, hand.T, hand.q);
        end
    end
    
    nf = hand.n; % number of fingers
    
    for j = 1:nf % for each finger. Order: Index, Thumb, Middle, Ring, Little
        F = hand.F{j};
        
        % plot the finger base
        % plot3(F.base(1,4),F.base(2,4),F.base(3,4),'ro'); % base position in World
        if(j == 1)
            hold on;
        end

        % plot the joints and the end tip
        referenceJoint = F.base;
        for i = 2:F.n+1
            localTransf = mySGDHMatrix(F.DHpars(i-1,:));
            
            refOld = referenceJoint(1:3,4);
            referenceJoint = referenceJoint*localTransf;
            
            p1 = [refOld(1),refOld(2),refOld(3)]';
            p2 = [referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4)]';
            
            % plot link (finger digit)
            if hand.type == "AllegroHandLeft" || hand.type == "AllegroHandRight"
                isFingerTip = i == F.n+1; % Finger tip: last link, white rubber
                mySGplotLink_allegro_customized(p1,p2,transp,isFingerTip);
            else % for human paradigmatic hand model
                mySGplotLink(p1,p2,5,transp);
            end
            
            % plot the joint location
            %{
            if i < F.n+1
               h = plot3(referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4),'ro');
               set(h,'MarkerSize',8);
               set(h,'LineWidth',3);
            end
            %}
        end
    end
    
    % plot hand palm
    if hand.type == "AllegroHandLeft" || hand.type == "AllegroHandRight"
        mySGplotPalm_allegro_customized(hand);
    else
        mySGplotPalm(hand);
    end
    
    axis 'equal';
    grid on;

    % view([-150, 30]);
    
    xlabel('X','FontSize',12);
    ylabel('Y','FontSize',12);
    zlabel('Z','FontSize',12);
    % title(hand.type);
    
    hold on;

end