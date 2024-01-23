%% This function samples the reachability map of the hand (currently the fingertip).

function isoMap = sampleReachabilityDistanceMap(rMap, alphaValue)

    if nargin < 2
        alphaValue = Inf; % Convex hull
    end

	if ~isequal(size(rMap,2),3)
		rMap = transpose(rMap);
	end
	nSamples = length(rMap);
	isoMap = zeros(nSamples,1); % isotropic property of distance

	%% Determine boundary by constructing alphaShape
	shp = alphaShape(rMap,alphaValue);
	[tri, xyz] = boundaryFacets(shp); % xyz is the matrix containing all boundary vertices (K,3)

	%% Generate spatial points to fill the inside of alphaShape [todo]
	for idx = 1:nSamples
		p = rMap(idx,:);
        if ismember(p,xyz,'row') % p is on the boundary surface
            isoMap(idx) = 0;
        else
            % disp(idx);
            distList = pdist2(p,xyz,'euclidean');
            try
                isoMap(idx) = min(distList(:))./max(distList(:)); % [0,1]
                % disp(isoMap(idx));
            catch
                isoMap(idx) = NaN;
            end
        end
	end
	
	%% Visualization
    %{
	figure, hold on;
	sz = 20;
    scatter3(rMap(:,1),rMap(:,2),rMap(:,3),sz,isoMap(:),'filled');
    colorbar;
    axid equal;
    grid on;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    title('Isotropy of Reachability Map');
    hold off;
    %}
end