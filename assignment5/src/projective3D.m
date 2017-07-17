close all;
points = ...
[-153.04413, -19.39994, -1.0777756;
 -251.51949, -27.538635, -0.91795433;
 -170.42828, -134.4877, -1.159376;
 -270.10541, -110.58663, -0.97864276;
 -191.99736, -285.43604, -1.2799823;
 -290.4317, -209.69798, -1.0484899;
 -215.7438, -448.29883, -1.4009337;
 -310.06503, -314.55872, -1.1234239;
 -299.80396, -647.42944, -1.839288;
 -342.51065, -539.61578, -1.6156162;
 -365.94406, -459.59119, -1.4407246;
 -390.0806, -404.72525, -1.3313332;
 -403.74139, -361.63342, -1.2384706;
 -637.23395, -1185.255, -3.1861694;
 -605.32056, -861.60553, -2.4408088;
 -585.87091, -674.45624, -2.0133021;
 -584.94153, -565.50238, -1.767195;
 -562.13422, -474.93335, -1.5571585;
 -4938.166, -8027.0596, -20.321671;
 -1719.9468, -2170.4092, -5.8501587;
 -1234.7599, -1278.4681, -3.6423593;
 -1013.6945, -904.9873, -2.7176797;
 -905.10718, -702.73682, -2.2238505];

% x1 = [points(1,:),1];
% x2 = [points(2,:),1];
% x3 = [points(3,:),1];
% x4 = [points(4,:),1];
% 
% line1 = pluker(x1'*x2 - x2'*x1);
% line2 = pluker(x3'*x4 - x4'*x3)';
% [~,~,V] = svd([line1;line2]);
% pt1 = V(:,end)
% line1*pt1
% line2*pt1
% 
% %intersection of point2
% x9 = [points(9,:),1];
% x10 = [points(10,:),1];
% x11 = [points(11,:),1];
% x12 = [points(12,:),1];
% 
% line3 = pluker(x9'*x10 - x10'*x9);
% line4 = pluker(x11'*x12 - x12'*x11);
% [~,~,V] = svd([line3;line4]);
% pt2 = V(:,end)
% 
% 
% x17 = [points(17,:),1];
% x21 = [points(21,:),1];
% x27 = [points(27,:),1];
% x32 = [points(32,:),1];
% 
% line5 = pluker(x17'*x21 - x21'*x17)';
% line6 = pluker(x27'*x32 - x32'*x27)';
% [U,S,V] = svd([line5;line6]);
% pt3 = V(:,end)
% 
% plane = null([pt1';pt2';pt3']);
% H = zeros(4,4);
% H(1:3,1:3) = eye(3);
% H(4,:) = plane;
% 
% points2 = [];
% for i = 1:length(points)
%     ptp = H*[points(i,:) 1]';
%     points2 = [points2; ptp(1:3)'/ptp(4)];
% end
% points = points2;
[m,n] = size(points);
figure();
plot3(points(1:2,1),points(1:2,2),points(1:2,3),'r-o');
hold on
plot3(points(1:6:7,1),points(1:6:7,2),points(1:6:7,3),'r-o');
hold on;
plot3(points(2:6:8,1),points(2:6:8,2),points(2:6:8,3),'r-o');
hold on
plot3(points(7:8,1),points(7:8,2),points(7:8,3),'r-o');
hold on
plot3(points(9:4:13,1),points(9:4:13,2),points(9:4:13,3),'b-o');
hold on
plot3(points(9:10:19,1),points(9:10:19,2),points(9:10:19,3),'b-o');
hold on
plot3(points(13:10:23,1),points(13:10:23,2),points(13:10:23,3),'b-o');
hold on
plot3(points(19:4:23,1),points(19:4:23,2),points(19:4:23,3),'b-o');
hold on

p = 8;
scatter3(points(1:p,1),points(1:p,2),points(1:p,3),'r');
hold on;
scatter3(points(p+1:end,1),points(p+1:end,2),points(p+1:end,3),'b');