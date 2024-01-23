%    SGplotLink - Plot a link of the hand structure
%
%    The function plots a clindrical link of the hand structure
%
%    Usage: h = SGplotLink(p1,p2,r)
%
%    Arguments:
%    p1 = the intial point of the link
%    p2 = the final point of the link
%    r = the radius of the link
%    transp = parameter representing surface transparency
%
%    See also: SGmakehand, SGmakeFinger
%
%    This file is part of SynGrasp (Synergy Grasping Toolbox).
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D.
%     Prattichizzo,
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


function [geo_mesh] = mySGplotLink(p1,p2,r,transp)

if nargout > 0
    if_plot = false; % If returned argument is expected, then do not plot the link. only for generating meshgrid.
else
    if_plot = true;
end

if nargin < 4
    transp = 1;
end

if nargin < 3
    r = 5; % radius of the link
end
    
    % link length
    l = norm(p2-p1)-r; % r = the radius of the link % minues r: plot cylinder in between, but use semispheres on both ends
    vers = (p2-p1)/norm(p1-p2);
    p3 = p1+vers*(l);

    % cilinder of radius r
    [xc,yc,zc]= cylinder(r,20);

    zc = l*zc;

    alphas = atan2(vers(2),vers(1));
    beta = atan2(sqrt(vers(1)^2+vers(2)^2),vers(3));

    R1 = SGroty(beta);
    R2 = SGrotz(alphas);
    R = R2*R1;

    % Pre-allocation
    xrot = zeros(2,21);
    yrot = zeros(2,21);
    zrot = zeros(2,21);

    for i = 1:2 
        for j = 1:21
            p = [xc(i,j), yc(i,j) zc(i,j)]';
            prt = R*p+p1;
            xrot(i,j) = prt(1);
            yrot(i,j) = prt(2);
            zrot(i,j) = prt(3);
        end
    end

    % Pre-allocation
    xst = zeros(21,21);
    yst = zeros(21,21);
    zst = zeros(21,21);

    % fcl = 0.75*ones(1,3); % face color
    % fcl = 'none';
    fcl = [224, 172, 105]/255;
    ecl = [141, 85, 36]/255; % 0.5*ones(1,3); % edge color

    % Step 1: surf the cylinder
    if if_plot
        h = surf(xrot,yrot,zrot,'FaceColor',fcl,'FaceAlpha',transp,'EdgeColor',ecl);
        alpha(h,transp);
    end
    
    [xs,ys,zs] = sphere(20);
    for i = 1:21  
        for j = 1:21
            p = [xs(i,j), ys(i,j) zs(i,j)]';
            pst = r*p+p3;
            xst(i,j) = pst(1);
            yst(i,j) = pst(2);
            zst(i,j) = pst(3);
        end
    end

    % Step 2: surf the semisphere on the fingertips
    if if_plot
        h = surf(xst,yst,zst,'FaceColor',fcl,'FaceAlpha',transp,'EdgeColor',ecl);
        alpha(h,transp);
    end
    
    if nargout > 0
        geo_mesh.cylinder.x = xrot; % (2,N) mtx, xrot(1,:) are x-coordinates of the top-circle of the cylinder, xrot(2,:) are x-coordinates of the bottom-circle of the cylinder
        geo_mesh.cylinder.y = yrot;
        geo_mesh.cylinder.z = zrot;
        geo_mesh.semisphere.x = xst;
        geo_mesh.semisphere.y = yst;
        geo_mesh.semisphere.z = zst;
    end
end