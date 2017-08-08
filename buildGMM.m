%构建GMM地图
% %每次运行的时候解除注释跑一次
clear;
% laser = pcread('/home/dxq/ros_ws/src/kitti/filteredMap.vtk.ply');
% laser = pcread('/home/dxq/matlab/work/GMM maps/map/floorFilteredMap00.vtk.ply'); %x-z camera coodinate ====
% laser = pcread('/home/dxq/catkin_ws/src/dynamic_SLAM/KeyFrame/0/DataPoints.vtk.ply');
% laser = pcread('/home/dxq/catkin_ws/src/offline_optimization/KeyFrame/wholeMap.ply');

Tvelo_cam = eye(4);
Tvelo_cam(1,1)= 0.0303;     Tvelo_cam(1,2)=-0.9995; 	Tvelo_cam(1,3)= -0.0031;	Tvelo_cam(1,4)= 0.0562;
Tvelo_cam(2,1)= -0.1099;	Tvelo_cam(2,2)=-0.0002;     Tvelo_cam(2,3)= -0.9939;	Tvelo_cam(2,4)= -0.0909;
Tvelo_cam(3,1)= 0.9935;     Tvelo_cam(3,2)= 0.0305;     Tvelo_cam(3,3)= -0.1098;	Tvelo_cam(3,4)= -0.0376;
R = Tvelo_cam(1:3,1:3);
[r,pitch,y] = dcm2angle(R);
R = angle2dcm(r,pitch,y);
Tvelo_cam(1:3,1:3)  = R;
tformVeloCam = affine3d(Tvelo_cam');
% 
pose = [-0.041749 -0.998968 -0.017865 25.847700 0.998789 -0.042194 0.025302 38.010000 -0.026030 -0.016788 0.999520 -2.670090 0.000000 0.000000 0.000000 1.000000];
pose = reshape(pose,4,4);

[a,b,c] = dcm2angle(pose(1:3,1:3)');
pose(1:3,1:3) = angle2dcm(a,b,c);
tform = affine3d(pose);

% pcshow(laser);
% 特定区域
% laser = pctransform(laser, (tformVeloCam.invert));
% laser = pctransform(laser, (tform.invert));
% laser = pctransform(laser, (tformVeloCam));


 range = 35;
 resolution = 0.5;


%range detection
zmin = 1000;    zmax = -1000;    xmin = 1000;    xmax = -1000;
for i = 1:size(laser.Location,1)
		if(laser.Location(i,3) < zmin)	zmin =laser.Location(i,3); end
		if(laser.Location(i,3) > zmax)	zmax = laser.Location(i,3); end
		if(laser.Location(i,1) < xmin)	xmin = laser.Location(i,1); end
		if(laser.Location(i,1) > xmax)	xmax =laser.Location(i,1);  end
end
% points_grid = cell(ceil((2*range+1)/resolution), ceil((2*range+1)/resolution));
Xgridmax = ceil(xmax/resolution);
Xgridmin = floor(xmin/resolution);
Zgridmin = floor(zmin/resolution);
Zgridmax = ceil(zmax/resolution);

points_grid = cell((Xgridmax-Xgridmin+1), (Zgridmax-Zgridmin+1));

if ( xmin < 0 )
    xbias = abs(Xgridmin)+1;
else
    xbias = 1;%matlab从1开始...
end
if ( zmin < 0 )
    zbias = abs(Zgridmin)+1;
else
    zbias = 1;
end

% 初始化
for(i = 1: (Xgridmax-Xgridmin+1))
    for(j = 1: (Zgridmax-Zgridmin+1))
        p.points = [];
        points_grid{i,j} = {p};
    end
end

points = [];

%% 为每个cell存入点的高度
for i = 1:size(laser.Location,1)
        point = laser.Location(i,:);
        points = [points ;point];
        points_grid{ floor ( point(1)/resolution )+xbias, floor ( point(3)/resolution )+zbias }{1,1}.points = ...
             [points_grid{ floor( point(1)/resolution )+xbias, floor( point(3)/resolution )+zbias }{1,1}.points; point(2)];

end
disp('insert points done...');
% points_laser = pointCloud(points);

% figure(1);
% pcshow(points_laser);

% figure(2);
% for(i = 1: 2*range+1)
%     for(j = 1: 2*range+1)
%         if( size(points_grid{i,j}{1,1}.points,1) > 15 )
%             
%             points_grid{i,j}{1,1}.model = singlecell_gmm2(points_grid{i,j}{1,1}.points);
% %              x = -3 : 0.1: 10;
% %              y = pdf(model, x');
% %              plot3(i*ones(size(x,2),1),y,x');
% %              hold on;
%         end
%     end
% end

%% 计算GMM模型
parfor(i = 1: (Xgridmax-Xgridmin+1))
    for(j = 1: (Zgridmax-Zgridmin+1))
        if( size(points_grid{i,j}{1,1}.points,1) > 5 )
            
            model = singlecell_gmm2(points_grid{i,j}{1,1}.points);
            if( model.Converged )
                points_grid{i,j}{1,1}.model  = model;
            end;

        end
    end
end

