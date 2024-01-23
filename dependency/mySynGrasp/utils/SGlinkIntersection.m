%  This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D. Prattichizzo,
%  All rights reserved.
% 
%  Redistribution and use with or without
%  modification, are permitted provided that the following conditions are met:
%      * Redistributions of source code must retain the above copyright
%        notice, this list of conditions and the following disclaimer.
%      * Redistributions in binary form must reproduce the above copyright
%        notice, this list of conditions and the following disclaimer in the
%        documentation and/or other materials provided with the distribution.
%      * Neither the name of the <organization> nor the
%        names of its contributors may be used to endorse or promote products
%        derived from this software without specific prior written permission.
% 
%  THIS SOFTWARE IS PROVIDED BY M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo, ``AS IS'' AND ANY
%  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
%  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
%  DISCLAIMED. IN NO EVENT SHALL M. Malvezzi, G. Gioioso, G. Salvietti, D.
%  Prattichizzo BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
%  EXEMPLARY, OR CONSEQUENTIAL DAMAGES
%  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
%  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
%  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
%  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

function [alpha, p_ct] = SGlinkIntersection(seg,object,epsilon)
    % alpha: intersection ratio
    % p_ct: point of contact

if(~SGisSeg(seg))
   error 'Argument seg should be a seg-structure' 
end

if(~isfield(seg,'radius'))
    seg.radius = 5;
end

if (nargin<3)
    epsilon=1e-4;
    sprintf('default value of epsilon set to %d',epsilon);
end
switch object.type
    case 'cube'
        % geometric structure
        geo_str = SGcube(object.Htr,object.dim(1)+3,object.dim(2)+3,object.dim(3)+3);
        alpha = SGintSegCube(seg,geo_str);
        p_ct = NaN; % NotImplemented
    case 'cyl'
        geo_str = SGcylinder(object.Htr,object.h,object.radius,object.res);
        alpha = SGintSegCyl(seg,geo_str,epsilon);
        p_ct = NaN; % NotImplemented
    case 'sph'
        geo_str = SGsphere(object.Htr,object.radius,object.res);
        [alpha, p_ct] = SGintSegSph(seg,geo_str,epsilon);
    otherwise
        error 'bad input arguments'
end
end