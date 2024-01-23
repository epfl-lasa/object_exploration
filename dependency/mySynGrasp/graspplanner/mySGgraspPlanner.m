%    SGgraspPlanner - Synthesis of a grasp for a given couple hand-object
%
%    The function allows to define the best grasp given an object, the 
%    number of pregrasp positions of the hand and the quality metric to be 
%    used for grasp evaluation.    
%
%    Usage: [hand_c,object,BEST_INDEX] = SGgraspPlanner(hand,obj,N,qmtype)
%
%    Arguments:
%    hand = the hand structure on which the contact points lie
%    obj = the object structure to be grasped
%    N = the number of random pregrasp positions of the hand 
%    qmtype = the quality metric to be used for grasp evaluation
%
%    Returns:
%    hand_c = the best hand configuration
%    object = the grasped object
%    BEST_INDEX = the best quality index obtained
%    
%    See also: SGgenerateCloud, SGcloseHand, SGevaluateOffset
%
%    Copyright (c) 2012 M. Malvezzi, G. Gioioso, G. Salvietti, D.
%    Prattichizzo, A. Bicchi
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  All rights reserved.



function [hand_c,object,BEST_INDEX] = mySGgraspPlanner(hand,obj,N,qmtype)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
switch hand.type
    case 'Paradigmatic'
        active=[0 1 1 0 0 1 1 1 0 1 1 1 0 1 1 1 0 1 1 1]';
%         active=[0 1 1 0 0 1 1 1 0 1 1 1 0 0 0 0 0 0 0 0]';
        q_init = zeros(1,20);
        q_init(1) = pi/3*2;
        q_init(17) = -pi/12;
        hand = SGmoveHand(hand,q_init);
        D = 5; % gauge in `SGgenerateCloud`
    case '3Fingered'
        active = [1 1 0 1 1 0 1 1]';
        D = 25;
    case 'DLR'
        active = [0 1 1 0 0 1 1 1 0 1 1 1 0 1 1 1 0 1 1 1 ]';
        D = 10;
    case 'Modular'
        active = [1 1 1 1 1 1 1 1 1]';
        D = 20;
    case 'AllegroHandLeft'
        active = [0,1,1,1, ... %index
            0,1,1,1, ... %middle
            0,1,1,1, ... %last
            1,1,1,1]; %thumb
        D = 5;
    case 'AllegroHandRight'
        active = [0,1,1,1, ... %index
            0,1,1,1, ... %middle
            0,1,1,1, ... %last
            1,1,1,1]; %thumb
        D = 5;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


hand_init = hand;

P = mySGgenerateCloud(obj,D,N); % Creates an array of Homogeneous transformation matrices for hand pose, P is a 4*4*N matrix Sigma-World R.F.

offset = mySGevaluateOffset(hand); % compute the initial positions of the hands
    
Plan = [];

for i = 1:size(P,3)
        
    for j = 1:hand.n
        T = hand_init.F{1,j}.base; 
        hand.F{1,j}.base = P(:,:,i)*T*mySGtransl(offset);
    end
        
    [hand_c,object] = mySGcloseHand(hand,obj,active,0.1); % Close hand to perform a grasp
 
    switch qmtype
        case 'mev' % manipulability ellipsoid volume
            if(~isempty(object.G))
                Quality = mySGmanipEllipsoidVolume(object.G,hand_c.J);
            else
                Quality = Inf;    
            end
        case 'gii' % grasp isotropy index
            if(~isempty(object.G))
                Quality = mySGgraspIsotropyIndex(object.G);
            else
                Quality = Inf;    
            end
        case 'msvg' % minimum singular value of the matrix G
            if(~isempty(object.G))
                Quality = mySGminSVG(object.G);
            else
                Quality = Inf;    
            end
        case 'dtsc' % distance from the singular configuration
            if(~isempty(object.G))
                Quality = mySGdistSingularConfiguration(object.G,hand_c.J);
            else
                Quality = Inf;    
            end
        case 'uot' % uniformity measure
            if(~isempty(object.G))
                Quality = mySGunifTransf(object.G,hand_c.J);
            else
                Quality = Inf;    
            end
        otherwise
            error 'bad quality measure type'
    end
    Plan.Object{i} = object;
    Plan.Hands{i} = hand_c;
    Plan.QIndex{i} = Quality; 
    
%% comment the following part if no figures of the trials are necessary
%     figure(i)
%     subplot(1,2,1)
%     grid on
%     mySGplotHand(hand)
%     hold on
%     mySGplotSolid(obj)
%     figure(i)
%     subplot(1,2,2)
%     grid on
%     mySGplotHand(hand_c);
%     hold on
%     mySGplotSolid(obj)

end

Splan = mySort(Plan);
BEST_INDEX = Splan.QIndex{size(P,3)};
BEST_HAND = Splan.Hands{size(P,3)};
BEST_OBJECT = Splan.Object{size(P,3)};
hand_c = BEST_HAND;
object = BEST_OBJECT;

end