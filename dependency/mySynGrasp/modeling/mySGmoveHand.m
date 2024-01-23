%    mySGmoveHand - Move a hand
%
%    The function move the finger joint imposing a displacement q. Returns
%    the hand in the new configuration.
%
%    Usage: hand = mySGmoveHand(hand,q)
%
%    Arguments:
%    hand = the hand structure to move
%    q = the joint variables of the new hand configuration
%    plot = this argument allows the user to plot the resulting movement of
%    the hand with the following semantic:
%       'initfinal' = the initial and final positions of the hand are
%                     plotted
%       'final' = only the final position of the hand is plotted
%
%    Returns:
%    hand = the hand in the new configuration
%
%    See also: mySGmakehand, mySGmakeFinger
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


function newHand = mySGmoveHand(hand,q)
    if nargin < 2
        q = hand.q; % if no extra q is given, use the current q to update the hand
    end
    
    if strcmp(q,'home')
        q = hand.q;
        q([2,3,4,6,7,8,10,11,12,14,15,16]) = 0;
    end
    
    if(size(q,1) ~= size(hand.q,1))
        error('mySG: Invalid dimensions of the input arguments.');
    end
    newHand = hand;
    
    q(~hand.qactv) = hand.q(~hand.qactv); % For inactive joint, keep current values
    
	k = 1;
	
    for i = 1:hand.n
        nq = hand.F{i}.n;
        newHand.F{i} = moveFinger(hand.F{i}, q(k:k+nq-1));
        k = k + nq;
    end
    
	newHand.q = q(:);
	newHand.ftips = mySGfingertips(newHand);
end