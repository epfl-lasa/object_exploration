%   SGminSVG - evaluate the minimum singular value of matrix G
%    
%   This function evaluates the minimum singular value of matrix G, this
%   parameter is adopted in the literature to evaluate grasp quality. 
%
%   A full-rank grasp matrix G has d singular values given by the 
%   positive square roots of the eigenvalues of GG' . 
%   When a grasp is in a singular configuration (i.e. when at least one
%   degree of freedom is lost due to hand configuration),
%   at least one of the singular values goes to zero. 
%   The smallest singular value of the grasp matrix G, SminG, 
%   is a quality measure that indicates how far is the grasp configuration 
%   from falling into a singular configuration, this is:
%   Q =SminG.
%   The largest SminG , the better the grasp. 
%   At the same time the largest the SminG the largest the minimum transmission
%   gain from the forces f at the contact points to the net wrench w 
%   on the object, which is also used as grasp optimization criterion.
%   
%    Usage: [SminG] = MinimumSingularValueOfG (G)
%
%    Arguments:
%    G = grasp matrix
%
%    Returns:
%    SminG = Minimum singular value of G
%    See also: SGuniTransf, SGmanipEllipsoidVolume.
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
%  All rights reserved.


 
function [SminG] = mySGminSVG(G)

	SminG = sqrt(eigs(G*(G'),1,'sm'));

end