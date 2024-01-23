%    SGmapping_SC - Computes the scale matrix used in SGmappingExample
%    
%    Usage: Sc = SGmapping_SC(k)
%
%    Arguments:
%    k = scaling factor
%
%    Returns:
%    Sc = scale matrix used in SGmappingExample
%
%    References:
%    1. G. Gioioso, G. Salvietti, M. Malvezzi, D. Prattichizzo. Mapping 
%    Synergies from Human to Robotic Hands with Dissimilar Kinematics: an 
%    Approach in the Object Domain. IEEE Trans. on Robotics, 2013.
%    2. G. Salvietti, L. Meli, G. Gioioso, M. Malvezzi, D. Prattichizzo. 
%    Multi-Contact Bilateral Telemanipulation with Kinematic Asymmetries. 
%    IEEE/ASME Transaction on Mechatronics, 2017.
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

function Sc = mySGmapping_SC(k)

    I = eye(3,3);
    Sc = zeros(7,7);
    
    Sc = [k*I zeros(3,3) [0,0,0]'; zeros(3,3) I [0,0,0]'; [0,0,0] [0,0,0] 1];

end