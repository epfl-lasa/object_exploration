function h = plot_config(h)
    global plot_cfg
    if isempty(plot_cfg)
    else
        h.FaceLighting = plot_cfg.FaceLighting;
        h.AmbientStrength = plot_cfg.AmbientStrength;
        h.DiffuseStrength = plot_cfg.DiffuseStrength;
        h.SpecularStrength = plot_cfg.SpecularStrength;
        h.SpecularExponent = plot_cfg.SpecularExponent;
        h.BackFaceLighting = plot_cfg.BackFaceLighting;
    end
end