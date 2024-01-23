% Calculate the center of the reachability map, expressed as alpha shape
function center = alphaShapeCenter(shp)
    center = mean(shp.Points);
end