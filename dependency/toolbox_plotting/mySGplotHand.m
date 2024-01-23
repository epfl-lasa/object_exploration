function myCustomizedSGplotHand(hand, factv, transp, plot_contacts)
% Customized function to plot hand. Modified based on the function 'SGplotHand'
% 
% Input arguments:
%     * hand: the hand structure to plot
%     * factv: list of boolean variables to indicate the activation of fingers. If active is not empty, only ploy active fingers
%     * transp: numerical parameter that represents surface transparency
%     * plott_contacts: if plot contact points (and normal vectors) on the model

    if nargin < 4
        plot_contacts = true;
    end
    if(nargin < 3)
        transp = 1;
    end
    if nargin < 2
        factv = ones(1,hand.n);
    end
    if nargin < 1 % Test mode, newly added: set default hand
        hand = SGparadigmatic;
        % hand = SGmodularHand;
    elseif(~SGisHand(hand))
        error 'hand argument is not a valid hand-structure' 
    end

    for j=1:hand.n % for each finger. Order: Index, Thumb, Middle, Ring, Little
        if ~factv(j) % if this finger is not activated (0), skip
            continue;
        end

        % [1/3] plot the finger base
        plot3(hand.F{j}.base(1,4),hand.F{j}.base(2,4),hand.F{j}.base(3,4),'r*'); % base position in World
        hold on;

        % plot the joints and the end tip
        referenceJoint = hand.F{j}.base;
        for i = 2:hand.F{j}.n+1
            localTransf = mySGDHMatrix(hand.F{j}.DHpars(i-1,:));
            refOld = referenceJoint(1:3,4);
            referenceJoint = referenceJoint*localTransf;
            
            % [2/3] plot the link
            p1 = [refOld(1),refOld(2),refOld(3)]';
            p2 = [referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4)]';
            if hand.type=="AllegroHandLeft" || hand.type=="AllegroHandRight" 
                if i<hand.F{j}.n+1
                    SGplotLink_allegro(p1,p2,[28/2 28/2],transp,[0.30 0.30 0.30]);
                else           
                    SGplotLink_allegro(p1,p2,[28/2 28/2],transp,[0.9 0.9 0.9]);
                end
            else
                SGplotLink(p1,p2,5,transp);
            end
            % plot the joint location
            if i < hand.F{j}.n+1
                h = plot3(referenceJoint(1,4),referenceJoint(2,4),referenceJoint(3,4),'ro');          
                set(h,'MarkerSize',8);
                set(h,'LineWidth',3);
            end
        end

        %%%%% [Extra] plot available contact points and coordinate frames
        if plot_contacts
            %%% Step 1/2: plot coordinate frames on current fingertip
            p_tip = referenceJoint(1:3,4); % end-effector position
            R_tip = referenceJoint(1:3,1:3); % rotation matrix
            x_tip = R_tip(:,1);
            y_tip = R_tip(:,2);
            z_tip = R_tip(:,3);
            plotAvlContacts(p_tip, {x_tip,y_tip,z_tip});
            %%% Step 2/2: plot contact points on the side of finger digits and their normal directions
            if isfield(hand.F{j},'avlConSet') % available contact set
                for i = 1:length(hand.F{j}.avlConSet)
                    if isempty(hand.F{j}.avlConSet{i})
                        continue;
                    else
                        plotAvlContacts(hand.F{j}.avlConSet{i}.P, hand.F{j}.avlConSet{i}.normVec);
                    end
                end
            else
                warning('Finger avlConSet not exist.');
            end
        end
        
    end
    % [3/3] plot palm
    if hand.type=="AllegroHandLeft" || hand.type=="AllegroHandRight" 
        SGplotPalm_allegro(hand);
    else
        SGplotPalm(hand);
    end

    axis 'equal';
    grid on;
    xlabel('x');
    ylabel('y');
    zlabel('z');
end