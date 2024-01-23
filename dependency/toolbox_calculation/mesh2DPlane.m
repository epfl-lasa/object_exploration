function [Pm,k,k_ch] = mesh2DPlane(P)
% Create meshgrid on 2d plane in 3d space (degenerated). First, transform X
% to a 2d plane using 'absor', then generate meshgrid on X-Y plane, and
% then transferred back to the P reference frame.
% 
% P: (N,3)
% Source: the plane of P
% Target: the projected plane, only X-Y

% try
%     K = convhull(X(:,1),X(:,2),X(:,3),'Simplify',true);
% catch % degenerated
%     K = [];
% end

N = size(P,1);

p1 = P(1,:); % use three non-colinear points
p2 = P(2,:);
p3 = P(3,:); k = 3;

% filter out colinear points
while (dot(p1(:)-p2(:),p1(:)-p3(:))/(norm(p1(:)-p2(:))*norm(p1(:)-p3(:)))==1) && (k<size(P,1))
    k = k+1;
    p3 = P(k,:);
end

S = orth([p1(:)-p2(:), p1(:)-p3(:)]); % source plane
T = [1 0 0; 0 1 0].'; % Target plane (X-Y)

reg = absor(S,T,'doTrans',false);
HT = reg.M; % see function 'absor'

Psrc = P.'; % (3,N)
Psrc_h = cat(1,Psrc,ones(1,N)); % (4,N) homogeneous coordinates
Ptgt_h = HT*Psrc_h;
Ptgt = Ptgt_h(1:3,:); % only on (X-Y) plane

Pm_tgt = polygrid(Ptgt(1,:), Ptgt(2,:), 1); % (N,2), meshgrid data in target domain (only X,Y)

Pm_tgt = cat(2,Pm_tgt,ones(size(Pm_tgt,1),1)*mean(Ptgt(3,:))); % (N,3)
Pm_tgt_h = cat(2,Pm_tgt,ones(size(Pm_tgt,1),1)); % (N,4)

Pm_src_h = HT\(Pm_tgt_h.'); % (4,N), transfer back, using inv(HT)
Pm = Pm_src_h(1:3,:); % (3,N)
Pm = transpose(Pm);

if nargout > 1
    k = boundary(Ptgt(:,1),Ptgt(:,2)); % this k should be a vector, since the plane is degenerated
end
if nargout > 2
    k_ch = convhull(Ptgt(:,1),Ptgt(:,2));
end

end