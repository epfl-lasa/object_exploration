% This function visualizes Allegro hand in the same color scheme as the real robotic hand.
% It has been imported from the package 'hand_dexterity'.
% dependency:
%   - plot_config

function mySGplotLink_allegro_customized(p1,p2,transp,isFingerTip)

    if nargin < 4
        isFingerTip = false;
    end
    
    if nargin < 3
        transp = 1.0;
    end
    
    metal_color = [0.2, 0.2, 0.2]; % Almost black
    rubber_color = [0.9, 0.9, 0.9]; % Almost white
    
%     metal_color = [0.5, 0.5, 0.5];
%     rubber_color = [0.9856, 0.7372, 0.2537];
    
    rc = 14; % radius of Allegro link
    rt = 14; % radius of the semi-sphere on link tails

    %% Plot link cylinder. If this is fingertip, plot the link into two parts: the black metal part and the white rubber part.
    if isFingerTip % For Allegro hand fingertip, use two colors to plot
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Step 2-1: plot the first half %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % This part of the cylinder is the link of the finger.
        pm = (p1+p2)/2;
        
        l = norm(pm-p1);
        vers = (pm-p1)/norm(p1-pm);
        [xc,yc,zc]= cylinder(rc,20);
        zc = l*zc;

        alphas = atan2(vers(2),vers(1));
        beta = atan2(sqrt(vers(1)^2+vers(2)^2),vers(3));

        R1 = SGroty(beta);
        R2 = SGrotz(alphas);
        R = R2*R1;

        xrot = zeros(2,21);
        yrot = xrot;
        zrot = xrot;

        for i = 1:2 
            for j = 1:21
                p = [xc(i,j), yc(i,j) zc(i,j)]';
                prt = R*p + p1;
                xrot(i,j) = prt(1);
                yrot(i,j) = prt(2);
                zrot(i,j) = prt(3);
            end
        end
        h1 = surf(xrot,yrot,zrot,'EdgeColor','none','FaceAlpha',transp,'FaceColor',metal_color);
        h1 = plot_config(h1);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Step 2-2: plot the second half %
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % This part of the cylinder is part of the rubber tip.
        l = norm(p2-pm)-rc;
        vers = (p2-pm)/norm(pm-p2);
        p3 = pm+vers*(l);

        [xc,yc,zc]= cylinder(rc,20);

        zc = l*zc;

        alphas = atan2(vers(2),vers(1));
        beta = atan2(sqrt(vers(1)^2+vers(2)^2),vers(3));

        R1 = SGroty(beta);
        R2 = SGrotz(alphas);
        R = R2*R1;

        xrot = zeros(2,21);
        yrot = xrot;
        zrot = xrot;

        for i = 1:2 
            for j = 1:21
                p = [xc(i,j), yc(i,j) zc(i,j)]';
                prt = R*p + pm;
                xrot(i,j) = prt(1);
                yrot(i,j) = prt(2);
                zrot(i,j) = prt(3);
            end
        end
        xst = zeros(21,21);
        yst = xst;
        zst = xst;

        h2 = surf(xrot,yrot,zrot,'EdgeColor','none','FaceAlpha',transp,'FaceColor',rubber_color);
        h2 = plot_config(h2);
        
    else
       % Normal finger link
        l = norm(p2-p1)-rc;
        vers = (p2-p1)/norm(p1-p2);
        p3 = p1+vers*(l);

        [xc,yc,zc]= cylinder(rc,20);

        zc = l*zc;

        alphas = atan2(vers(2),vers(1));
        beta = atan2(sqrt(vers(1)^2+vers(2)^2),vers(3));

        R1 = SGroty(beta);
        R2 = SGrotz(alphas);
        R = R2*R1;

        xrot = zeros(2,21);
        yrot = xrot;
        zrot = xrot;

        for i = 1:2 
            for j = 1:21
                p = [xc(i,j), yc(i,j) zc(i,j)]';
                prt = R*p + p1;
                xrot(i,j) = prt(1);
                yrot(i,j) = prt(2);
                zrot(i,j) = prt(3);
            end
        end

        xst = zeros(21,21);
        yst = xst;
        zst = xst;
        
        hc = surf(xrot,yrot,zrot,'EdgeColor','none','FaceAlpha',transp,'FaceColor',metal_color);
        hc = plot_config(hc);
    end

    %% plot spheres on the link tail
    [xs,ys,zs] = sphere(20);
    for i = 1:21  
        for j = 1:21
            p = [xs(i,j), ys(i,j) zs(i,j)]';
            pst = rt*p+p3;
            xst(i,j) = pst(1);
            yst(i,j) = pst(2);
            zst(i,j) = pst(3);
        end
    end
    
    if isFingerTip
        color = rubber_color;
    else
        color = metal_color;
    end
    
    hs = surf(xst,yst,zst,'EdgeColor','none','FaceAlpha',transp,'FaceColor',color);
    plot_config(hs);
    
    if isFingerTip
        material('dull');
    else
        material('metal');
    end
end