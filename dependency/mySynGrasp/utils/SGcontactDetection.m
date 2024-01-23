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


function [cp_mat, ct_pnts] = SGcontactDetection(hand,object,i)
    % i is the index that indicates the finger the algorithm is moving

    finger = hand.F{i};

    cp_mat = []; % (N_contact, 2), each row contains contact point information [j, alpha], where j is the link number, alpha is the ratio of this contact
    
    ct_pnts = []; % (3,N), save contact points

    % for j = 1:finger.n-1
    for j = 1:finger.nlink-1
        alpha = NaN;
        if(norm(finger.joints_pos(:,j+1)-finger.joints_pos(:,j)) > 0) % link length > 0, real link (in contrast to virtual link)

            % link_seg = SGsegment(finger.joints_pos(:,j),finger.joints_pos(:,j+1)); % [deprecated]
            
            link_seg = struct('p0', finger.Link{j}.HT_this(1:3,4),...
                'p1', finger.Link{j}.HT_next(1:3,4),...
                'r', finger.Link{j}.radius,...
                'type','seg'); % use finger link to construct a 'seg' type structure
            
            [alpha, p_ct] = SGlinkIntersection(link_seg,object); % p_ct only implemented for spheres
        end

        if ~isnan(alpha)    
           cp_mat = [cp_mat; j alpha];
           ct_pnts = [ct_pnts, p_ct(:)];
        end
    end
end