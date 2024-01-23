%    SGevaluateOffset - Used by SGgraspPlanner
%
%    The function allows to compute the initial positions of the hands in
%    the grasp planning.
%
%    Usage: offset = SGevaluateOffset(hand)
%
%    Arguments:
%    hand = the hand structure on which the contact points lie
%
%    Returns:
%    offeset = the offeset need to correctly place the hand
%
%    See also: SGgraspPlanner



function offset = mySGevaluateOffset(hand)

offset = [0 0 0]';

for i = 1:hand.n
	offset = offset + hand.F{i}.base(1:3,1:3)*hand.F{i}.base(1:3,4);
end

offset = offset / hand.n;

end