function feats = HistogramGLCM(I)
% GLCM features for 2D Histogram
% The features are defined by the GLCM metrics.
% Website: http://murphylab.web.cmu.edu/publications/boland/boland_node26.html
% C. Jiang

I_ = I/norm(I);
%% Angular Second Moment
feats.AngMnt = sum(sum(I_.*I_));

%% Contrast 

%% Entropy
logI_ = log(I_);
logI_(isinf(logI_))=0; logI_(isnan(logI_))=0;
feats.Etpy = -sum(sum(I_.*logI_));
end