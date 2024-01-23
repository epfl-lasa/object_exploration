%    SGplotPalm - Plot the palm of a hand model
%
%    The function plots the palm of the hand model given as argument
%
%    Usage: SGplotPalm
%
%    Arguments:
%    hand = the hand whose palm the user wants to plot
%   
%    See also: SGplotHand
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

function mySGplotPalm(hand,transp,radius,nps)

if nargin < 4
    nps = 5; % nps is the resolution of sphere
end
if nargin < 3
    radius = 5;
end
if nargin < 2
    transp = 1;
end

F = hand.F;
n = hand.n;

basepoints = zeros(3,n+1);
for i = 1:n
    basepoints(:,i)= F{i}.base(1:3,4);
end

if abs(F{2}.base(1,4)-F{n}.base(1,4)) > abs(F{2}.base(2,4)-F{n}.base(2,4))
    puntodx = [F{n}.base(1,4)
        F{1}.base(2,4)
        F{n}.base(3,4)] ;
else
    puntodx = [F{1}.base(1,4)
        F{n}.base(2,4)
        F{n}.base(3,4)] ;
end
basepoints(:,n+1) = puntodx;

% Notice that this is different from 'makePalm' function.
bp = basepoints';
co = mean(bp);

for i = 1:size(bp,1)
    ncp(i,:) = (co - bp(i,:))/norm(co - bp(i,:));
end

X = bp;

% fcl = 0.75*ones(1,3); % face color
% ecl = 'k'; % 0.5*ones(1,3); % edge color

fcl = [224, 172, 105]/255; % Human skin color
ecl = [141, 85, 36]/255;

% fcl = 'none';
% ecl = 'none';

for i = 1:size(bp,1)
    [filletx,fillety,filletz] = sphere(nps);
    spherecenter = bp(i,:) + radius*ncp(i,:);
    for j = 1:nps+1
        for k = 1:nps+1
            fillett = radius*[filletx(j,k) fillety(j,k) filletz(j,k)]+spherecenter;
            X = [X;fillett];
        end
    end
end

k = convhulln(X);
trisurf(k,X(:,1),X(:,2),X(:,3),'FaceColor',fcl,'FaceAlpha',transp,'EdgeColor',ecl);
hold on;