% function that evaluates the contact between a segment and a sphere,
% returning the corresponding alpha parameter as the normalized position:
% alpha = 0 means the first point, alpha = 1 means the second, if there
% aren't any contact points, alpha = NaN.
% epsilon could be absolute or radius-relative (% of radius)
%
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

function [alpha, p_ct] = SGintSegSph(seg,sph,epsilon)

p0 = seg.p0;
p1 = seg.p1;

N = 100; % number to interpolate
t = repmat(linspace(0,1,N), 3,1); % expand to three dimensions of the point
p_list = (1-t).*p0 + t.*p1; % (3,N)

d = pdist2(p_list',sph.center','euclidean');
[d_min, idx_min] = min(d);

if d_min-(sph.radius+seg.r)<=epsilon
    alpha = idx_min/N; % give the ration of the alpha, which is, the 'alp' coordinate of addContact
    p_ct = p_list(:,idx_min); % point of contact, in the central line of the link cylinder
    
    vec_norm = seg.radius * normalize(sph.center-p_ct,'norm'); % project to surface, the real contact point
    p_ct = p_ct + vec_norm;
    
    if length(idx_min) > 1
        warning('Multiple contact points detected: potential interposition of geometric shapes.');
    end
    return;
else
    alpha = NaN;
    p_ct = NaN;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Following are the Original SG Implementation %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% the solving equation is A*alpha^2 + B*alpha + C = 0; where:

if norm(seg.p1 - sph.center)<=sph.radius
    alpha = 1;
else
    x1 = seg.p1(1);
    y1 = seg.p1(2);
    z1 = seg.p1(3);
    x0 = seg.p0(1);
    y0 = seg.p0(2);
    z0 = seg.p0(3);
    xc = sph.center(1);
    yc = sph.center(2);
    zc = sph.center(3);
    R  = sph.radius;
    
    A = ((x1 - x0)^2 + (y1 - y0)^2 + (z1 - z0)^2);    
    B = 2*((x1 - x0)*(x0 - xc) + (y1 - y0)*(y0 - yc) + (z1 - z0)*(z0 - zc));    
    C = (x0 - xc)^2 + (y0 - yc)^2 + (z0 - zc)^2 - R^2;    
    Delta = B^2 - 4*A*C;
    
    % contact evaluation:
    
    if (Delta < 0.0)
        %no intersection
        alpha = NaN;
        return
    end
    if (Delta >= 0.0 && norm(Delta) <= epsilon)
        % approximated one point intersection
        alpha = (-B/(2*A));
        if (alpha < 0 || abs(alpha)>1)
            alpha = NaN;
            return
        end
    elseif (Delta > 0.0 && norm(Delta) > epsilon)
        % two points intersection
        t0 = (-B - sqrt(Delta))/(2*A);
        if (t0 > 0.0 && abs(t0)<1)
            alpha = t0;
        else
            alpha = (-B + sqrt(Delta))/(2*A);
            if (alpha < 0 || abs(alpha)>1)
                alpha = NaN;
                return
            end
        end
    end
end
end