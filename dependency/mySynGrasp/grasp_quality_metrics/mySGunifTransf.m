%   SGunifTransf - grasp quality measures
%
%   Quality measures associated with the hand configuration
%   ->Uniformity of transformation
%
%   This quality measure function considers hand configuration.
%   The transformation between the velocity domain in the finger joints and 
%   the velocity domain of the object is "uniform" when the contribution 
%   of each joint velocity is the same in all the components of  
%   object velocity.
%
%   The quality measure is given by:
%   Q=SmaxHO/SminHO
%   with SmaxHO and SminHO being the maximum and minimum singular values of
%   HO.
%   The quality of a grasp will be better when the grasp configuration 
%   gives Q as close as possible to 1.
%
%    Usage: [Ut] = SGunifTransf(G,J)
%
%    Arguments:
%    G = grasp matrix
%    J = hand jacobian matrix   
%
%    Returns:
%    Ut = Uniformity Of Transformation measure
%
%    See also: SGDistanceToSingularConfigurations,
%    SGVolumeOfTheManipulabilityEllisoid .
%
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
%  All rights reserved.


 
function [Ut] = mySGunifTransf(G,J) 

	Gc = G'*inv(G*G');
	HO = Gc'*J;
	SmaxHO = sqrt(eigs(HO*(HO'),1,'lm'));
	SminHO = sqrt(eigs(HO*(HO'),1,'sm'));
	Ut = SmaxHO/SminHO;

end