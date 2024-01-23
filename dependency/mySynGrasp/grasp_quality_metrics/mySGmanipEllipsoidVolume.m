%   SGmanipEllisoidVolume - evaluates the hand manipulability ellipsoid
%
%   This function evaluates a quality measures associated with the hand 
%   configuration and specifically the volume of the manipulability ellipsoid
%
%   This group of quality measures includes those that consider
%   the hand configuration to estimate the grasp quality.
%   In order to consider all the singular values of HO, the
%   volume of the manipulability ellipsoid is proposed as quality measure.
%   Let S1HO,S2HO, ...,SrHO be the singular values of HO. 
%   The grasp quality (i.e. the volume of the manipulability ellipsoid) is;
%   Q=kb*sqrt(det(HO*(HO'')))=kb*(S1HO*S2HO*....*SrHO)
%   where kb is a constant.
%   The quality is therefore proportional to the product of all the singular
%   values, and maximizing the determinant of H*H'
%   maximizes the volume of the ellipsoid.
%
%    Usage: [VE] = SGManipEllipsoidVolume(G,J)
%
%    Arguments:
%    G = grasp matrix
%    J = hand jacobian matrix   
%
%    Returns:
%    VE = Volume Of The Manipulability Ellisoid
%
%    See also: SGuniTransf, SGminSVG.
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
%  All rights reserved.


function [VE] = mySGmanipEllipsoidVolume(G,J)

	Gc = G'*inv(G*G');
	HO = Gc'*J;
	VSHO = sqrt(eig(HO*(HO')));
	VE = VSHO(1)*VSHO(2)*VSHO(3)*VSHO(4)*VSHO(5)*VSHO(6);
	VE = sqrt(det(HO*(HO')));

end