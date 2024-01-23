function offset = mySGevaluateOffset(hand,active)
% Customized function. The function allows to compute the initial positions of the hands in the grasp planning.
% Currently work only for human hand "mySGparadigmatic". Parameters need to be generalized.
	if nargin < 2
	    active_finger = [1,1,1,1,1]; % all fingers are active
	else
	    active_finger = logical(sum(reshape(active,4,5),1)); % 20 dofs in total
	end
	offset = [0 0 0]';

	n = sum(active_finger~=0);
	idx = find(active_finger~=0);

	for i = 1:n
	    offset = offset + hand.F{idx(i)}.base(1:3,1:3)*hand.F{idx(i)}.base(1:3,4);
	end

	offset = offset / n;
end