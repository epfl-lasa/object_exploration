%    mySGgraspStiffness - Evaluate the grasp stiffness matrix
%
%    Usage: K = mySGgraspStiffness(hand, object)
%
%    Arguments:
%    hand, object: structures that contain hand and object grasp properties
%
%    Returns: 
%    K: a 6x6 matrix representing grasp stiffness matrix
%
%    References
%    M. Malvezzi,D. Prattichizzo, "Evaluation of Grasp Stiffness in 
%    Underactuated Compliant Hands", Proceedings, IEEE International 
%    Conference on Robotics and Automation, Karlsruhe, Germany, 2014. 
%
%    See also:  mySGkinManipulability, mySGforceManipulability,
%    mySGquasistaticHsolution
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
%  All rights reserved.



function K = mySGgraspStiffness(hand,object)

	Kc = object.Kc;
	Kq = hand.Kq;
	Kz = hand.Kz;
	J = hand.J;
	G = object.G; 
	S = hand.S;

	Kceq = inv(inv(Kc) + J*inv(Kq)*J' + J*S*inv(Kz)*S'*J');
	K = G*Kceq*G';
end