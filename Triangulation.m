% Title: Triangulation of 3D point based on min distance between skewed lines
% Author: Shreyash Annapureddy (Logic helped and discussed with Saivan Hamama)
% Date: 25/02/2016

function [sol_t1,sol_t2,tri_point] = Triangulation(transform_matrix,matched_points,K,pix2mm)
    syms t1 t2
    sol_t1 = zeros(4,1);
    sol_t2 = zeros(4,1);
    o_x = K(1,3)*pix2mm(1);
    o_y = K(2,3)*pix2mm(2);
    
    % Determine the vector equation of the projection line for img 0
    parallel_vec_0 = ((matched_points(:,:,1).*pix2mm) - [o_x;o_y;0]); 
    % Determine the parametric equation for some point on the
    % parallel_vec_0, call it Z
    Z = [o_x+(t1*parallel_vec_0(1));o_y+(t1*parallel_vec_0(2));t1*parallel_vec_0(3)];
    %Z = Z.*pix2mm;
    for i = 1:4
        % Transform the images coordaintes of the second image frame to
        % the first image frame
        matched_points(:,:,2) = matched_points(:,:,2).*pix2mm;
        new_feat     = transform_matrix(:,:,i)*[matched_points(:,:,2);1];
        new_optCent  = transform_matrix(:,:,i)*[o_x;o_y;0;1];
        parallel_vec = (new_feat(1:3) - new_optCent(1:3));
        
        V = [(new_optCent(1))+(t2*parallel_vec(1));...
             (new_optCent(2))+(t2*parallel_vec(2));...
             (new_optCent(3))+(t2*parallel_vec(3))];
     
        
        % We are trying to find some vector ZV that is perpendicular to
        % both L1 and L2, this represents the shortest distance between L1
        % and L2. Hence we first paramatrise point Z and point V
        vec_ZV = V - Z;
        
        % Using the orthogonality rule check if vec_ZV.L1 = vec_ZV.L2 = 0
        equ1 = dot(vec_ZV,parallel_vec_0);
        equ2 = dot(vec_ZV,parallel_vec);
        
        [solt1,solt2] = solve(equ1 == 0, equ2 == 0);
        sol_t1(i) = solt1;
        sol_t2(i) = solt2;
        
        P_point = [o_x+(solt1*parallel_vec_0(1));...
                   o_y+(solt1*parallel_vec_0(2));...
                   solt1*parallel_vec_0(3)];
               
        Q_point = [(new_optCent(1))+(solt2*parallel_vec(1));...
                   (new_optCent(2))+(solt2*parallel_vec(2));...
                   (new_optCent(3))+(solt2*parallel_vec(3))];
        tri_point(:,:,i) = (P_point+Q_point)/2;
    end
    tri_point = double(tri_point);
end