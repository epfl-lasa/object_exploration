% This function updates the transformation matrix T for the hand and
% returns a new hand.
% This is only for VISUALIZATION, since finger kinematics is not updated.

function H = updateHandTransformationForVisualization(hand, T, q)
    global baseMatrices

    H = hand; % This is the new hand
    H.T = T;
    H.q = q;
    
    for iF = 1:H.n
        H.F{iF}.base = T * baseMatrices.fingers(:,:,iF);
    end
    
    H.P.palm_base = T * baseMatrices.palm;
    H.P.basepoints_h =  T * baseMatrices.palm_basepoints_h;
    
    H.P.cvx_vtx_h = T * baseMatrices.palm_cvx_vtx_h;
    
    H = mySGmoveHand(H);
end