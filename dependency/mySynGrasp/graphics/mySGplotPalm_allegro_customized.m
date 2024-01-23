% [todo] This function works only for hand.T = eye(4), need to extend.

function mySGplotPalm_allegro_customized(hand,transp)

    if nargin < 2
        transp = 1.0;
    end
    
    metal_color = [0.2, 0.2, 0.2]; % Almost black
    rubber_color = [1.0, 1.0, 1.0]; % Almost white
    
    cvx_vtx_h = hand.P.cvx_vtx_h; % (4,N)
    
    X = transpose(cvx_vtx_h(1:3,:)); % (N,3)
    k = convhulln(X);

    h = trisurf(k,X(:,1),X(:,2),X(:,3),'EdgeColor','none','FaceAlpha',transp,'FaceColor',metal_color);
    h = plot_config(h);

    hold on;
end