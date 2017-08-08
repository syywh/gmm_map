%单个grid(x,y)中高度上2核混合高斯模型拟合

function model = singlecell_gmm2(points)
    clf
%     height = [];
%     for i = 1:size(points,1)
%         height = [height; cell2mat(points(i))];
%     end
    height = points;
    p = ones(size(height,1),1);

    
%     figure(11);
%     scatter(height, p);  
    options = statset('Display','final','MaxIter',500);
    
    AIC = zeros(1,4);
    GMModels = cell(1,4);
    for k = 1:4
        GMModels{k} = fitgmdist(height,k,'Options',options,'CovarianceType','diagonal','RegularizationValue',0.01);
        AIC(k)= GMModels{k}.AIC;
    end
    [minAIC,numComponents] = min(AIC);
    numComponents;

    model = GMModels{numComponents}   ;
    model.Converged;
    
% 
%     model = fitgmdist(height,1,'Options',options,'RegularizationValue',0.01);
%     model.mu
%     minX = min(height);
%     maxX = max(height);
%     x = minX:0.01:maxX;
%     y = pdf(model, x');
%     figure(7)
%     plot(x,y);
%     
%     model = fitgmdist(height,2,'Options',options,'RegularizationValue',0.01);
%     model.mu
%     minX = min(height);
%     maxX = max(height);
%     x = minX:0.01:maxX;
%     y = pdf(model, x');
%     figure(7)
%     plot(x,y);   
% 
%     model = fitgmdist(height,3,'Options',options,'RegularizationValue',0.01);
%     model.mu
%     minX = min(height);
%     maxX = max(height);
%     x = minX:0.01:maxX;
%     y = pdf(model, x');
%     figure(7)
%     plot(x,y);   
% %     model.Converged

