% Title : Plot projection lines from cam center to feature on the image
%         frame
%
% Author: Shreyash Annapureddy
% Date  : 11/03/2016  

function[transFeatures,transOptCnt,ProjectionPoints] = Projection_Points(T,camParam,features,pix2mm)
init_optCnt = [camParam.ox;camParam.oy;0].*pix2mm;
new_optCnt  = T*[init_optCnt;1];
newFeature = features(:,:,2).*pix2mm;
newFeature = T*[newFeature;1];

parallel_vec_init   = (features(:,:,1).*pix2mm) - init_optCnt;
parametric_init     = init_optCnt + (2*parallel_vec_init);

ProjectionPoints(:,:,1) = [init_optCnt,parametric_init];


parallel_vec_2 = newFeature(1:3) - new_optCnt(1:3);
parametric_2   = new_optCnt(1:3) + (2*parallel_vec_2);
ProjectionPoints(:,:,2) = [new_optCnt(1:3),parametric_2];

transFeatures = [features(1:3,:,1).*pix2mm,newFeature(1:3)];
transOptCnt   = [init_optCnt(1:3),new_optCnt(1:3)];
end