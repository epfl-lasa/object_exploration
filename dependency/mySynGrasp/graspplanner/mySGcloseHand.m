%    SGCloseHand - Synthesis of a grasp for a given couple hand-object
%
%    The function is used insede SGGraspPlanner to move the hand.
%
%    Arguments:
%    hand = the hand structure on which the contact points lie
%    obj = the object structure to be grasped
%    activeJoints = the active joints to move (1 to move)
%    increment = the quantity the joints have to move
%
%    Returns:
%    new_hand = the new position of the hand
%    object = the grasped object
%    
%    See also: SGgenerateCloud, SGgraspPlanner, SGevaluateOffset
%
%
%  Copyright (c) 2013, M. Malvezzi, G. Gioioso, G. Salvietti, D. Prattichizzo,
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

function [newHand,object,ct_points] = mySGcloseHand(hand,obj,active_joints,increment)

if nargin<4
    increment = 0.1;
end

final_pose = 0;

if (isscalar(increment))
    step = increment * ones(length(hand.q),1); % all joints have the same increment value (step size)
else
    step = increment; % increment is a vector, different values for different joints
end

count = 0;
max_iter = 1000;
ct_points = []; % save set of contact points

while (final_pose == 0 && count <= max_iter)
    for i = 1:hand.n % for each finger
        index = find(hand.qin == i); % index of joints belong to finger `i`
        for j = 1:length(index) % for each joint of finger `i`
            k = index(j); % joint order in this finger
            
            if(active_joints(k) == 1) % this joint is active
                q_new = hand.q;
                q_new(k) = q_new(k) + step(k);
                
                % Check new joint angle within joint motion range
                % if(q_new(k) >= hand.limit(k,1) && q_new(k) <= hand.limit(k,2))
                % limit(:,1): lower range, limit(:,2): upper range
                
                % if(q_new(k) >= hand.F{i}.lb(k) && q_new(k) <= hand.F{i}.ub(k))
                if (increment(k)>0 && q_new(k)<=hand.F{i}.ub(j)) || (increment(k)<0 && q_new(k)>=hand.F{i}.lb(j))
                    
                    hand = mySGmoveHand(hand,q_new);
                    
                    [cp_mat, tmp_ct] = mySGcontactDetection(hand,obj,i); % contact detection
                    % ct_points: contact points
                    
                    if ~isempty(cp_mat)
                        max_link = max(cp_mat(:,1));
                        for h = 1:max_link
                            active_joints(index(h)) = 0;
                        end
                        for c = 1:size(cp_mat,1)
                            hand = mySGaddContact(hand,1,i,cp_mat(c,1),cp_mat(c,2));
                        end
                        ct_points = [ct_points, tmp_ct];
                    end
                else
                    % Joint reaches limit, stop moving, no longer active
                    active_joints(k) = 0;
                end
            end
        end
    end
    
    if (active_joints == zeros(length(hand.q),1))
        final_pose = 1;
    end
    
    count = count +1;
    
end

if size(hand.cp,2)>0
    [hand,object] = mySGcontact(hand,obj);
else 
    object = obj;
    object.base = [eye(3) object.center; zeros(1,3) 1];
    object.Kc = [];
    object.H = [];
    object.Gtilde = [];
    object.G = [];
end
newHand = hand;
end