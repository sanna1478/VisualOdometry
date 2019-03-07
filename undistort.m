% Tilte: Main function for initial VO 
% Author: Shreyash Annapureddy
% Date: 18/02/2016

%% Camera Undistortion Function
function correctedImage = undistort(distortedImage,camParam)

% It is assumed that a pin-hole model is being used

% All the inital camera parameters are for a logitech webcam owned by Shrey
f_x = camParam.fx; % Focal length in the x (727.65579)
f_y = camParam.fy; % Focal length in the y (727.95799)
o_x = camParam.ox; % Optical centre in the x axis (330.50635)
o_y = camParam.oy; % Optical centre in the y axis (167.87330)
k1  = camParam.k1; % First tangential distortion value (0.03547)
k2  = camParam.k2; % Second tangential distortion value (-0.34556)
k3  = camParam.k3; % Third tangential distortion value (0.0000)
p1  = camParam.p1; % First radial distortion value (0.0058)
p2  = camParam.p2; % Second radial distortion value (0.00316)
F   = camParam.F ; % Focal length of camera in mm (1.2)

mm_x = F/f_x; % conversion factor between pixels to millimeters
mm_y = F/f_y; % conversion factor between pixels to millimeters

intrinsicMatrix = [f_x, 0  , o_x;...
                   0  , f_y, o_y;...
                   0  , 0  , 1];  % Intrinsic params of camera
                             
correctedImage = zeros(size(distortedImage)); % images space same size as original image
[rows,cols] = find(~isnan(correctedImage));
rays = intrinsicMatrix\[cols rows ones(length(rows),1)]';
% Mapping the distortions forward from a psuedo-corrected image
r2 = rays(1,:).^2 + rays(2,:).^2;
x = rays(1,:);
y = rays(2,:);

x = x.*(1+k1*r2 + k2*r2.^2 + k3*r2.^3) + 2*p1.*x.*y + p2*(r2 + 2*x.^2);
y = y.*(1+k1*r2 + k2*r2.^2 + k3*r2.^3) + 2*p2.*x.*y + p1*(r2 + 2*y.^2);

u = reshape(f_x*x + o_x,size(correctedImage));
v = reshape(f_y*y + o_y,size(correctedImage));

correctedImage = interp2(distortedImage,u,v);
end
           
