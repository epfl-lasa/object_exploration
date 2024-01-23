%    SGslide - computes the displacement of hand joints and wrist position
% due to the dqr assigned. It takes care of the compliance at wrist, joint
% and contact level
%
%    Usage: [displacements] = SGslide(hand,Kc,dqr)
%
%    Arguments:
%    hand = the hand structure
%    object = Kc contact stiffness matrix
%    dqr = reference joint displacement
%
%    Returns: 
%    displacements = joint and wrist displacement 
%
%    References
%
%    See also:  SGquasistatic, SGquasistaticMaps
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

function [displacements] = mySGslide(hand,Kc,dqr)

ncp = size(hand.cp,2);
Kqext = [hand.Kq, zeros(hand.m, 6); zeros(6, hand.m) hand.Kw];

A = [eye(3*ncp), -hand.Jext, zeros(3*ncp, hand.m +6), zeros(3*ncp);
zeros(hand.m +6, 3*ncp), zeros(hand.m +6), eye(hand.m + 6) -hand.Jext';
zeros(hand.m +6, 3*ncp), Kqext, eye(hand.m +6), zeros(hand.m +6, 3*ncp);
-Kc, zeros(3*ncp, hand.m +6), zeros(3*ncp, hand.m +6), eye(3*ncp)];

b = [zeros(3*ncp,1); zeros(hand.m+6, 1); Kqext * dqr'; zeros(3*ncp,1)];

displacements = A\b;
 
end