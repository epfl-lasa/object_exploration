%function that plots a cube in the space given the cube as a structure
%calculated by the function SGcube
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

function SGplotCube(cube)

hold on
grid on
axis 'equal'
xlabel('x')
ylabel('y')
zlabel('z')
for i = 1:6
    fill3(cube.faces.ver{i}(1,:),cube.faces.ver{i}(2,:),cube.faces.ver{i}(3,:),'r')
end
% fill3(cube.faces.f1(1,:),cube.faces.f1(2,:),cube.faces.f1(3,:),'y')
% fill3(cube.faces.f2(1,:),cube.faces.f2(2,:),cube.faces.f2(3,:),'m')
% fill3(cube.faces.f3(1,:),cube.faces.f3(2,:),cube.faces.f3(3,:),'c')
% fill3(cube.faces.f4(1,:),cube.faces.f4(2,:),cube.faces.f4(3,:),'r')
% fill3(cube.faces.f5(1,:),cube.faces.f5(2,:),cube.faces.f5(3,:),'g')
% fill3(cube.faces.f6(1,:),cube.faces.f6(2,:),cube.faces.f6(3,:),'b')
end