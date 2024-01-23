function result = homoTransform(name, instant, T)
% Transform the 'instant' in CF1 to CF2 via T. p in CF1. Then the representation of p
% in CF2, p', is calculated as: p' = T*p.

switch name
    case 'hand'
        result = instant;
        for j=1:result.n % Iterate over fingers
            result.F{1,j}.base = T * instant.F{1,j}.base;
        end

        result.homoCF = T * instant.homoCF;
    case 'object' % Currently only for sphere object
        result = instant;
        result.Htr = T * instant.Htr;
        result.center = result.Htr(1:3,4);
end

end