%The size of all corners if 10 x 8 with total of 80 points
%define and initialize world coordinates that were measured with the ruler
%world coordinate system
function [K,P] = challenge()
clc;
clear all;
close all;
xW = buildWorldCoord(10);

% filename = 'Picture4.jpg';
filename = 'cal.jpg';
im_RGB = imread(filename); % RGB color image
im_gray = rgb2gray(im_RGB); % Gray-scale image
[M,N] = size(im_gray);
corners = corner(im_gray,160);
sorted = sortrows(corners);

coors = [];
k = 1;

%sort the data and make sure it is increasing in the y direction
cpoint = [];
for i = 1:10:160
    temp = sorted(i:i+9,:);
    [~,I] = sort(temp(:,2));
    cpoint = [cpoint;temp(I,:)];
end
sorted = cpoint;

%plot the corner points on the image
figure;imshow(im_RGB);hold on;
% for i = 1:160
%         plot(sorted(i,1),sorted(i,2),'r.','markers',8);
% %         text(sorted(i+(j-1)*10,1)+3,sorted(i+(j-1)*10,2), num2str(k) ,'color','y', 'FontSize',8);
%         k = k + 1;
% end

for i = 1:10
    for j = 1:16
        coors = [coors; sorted(i+(j-1)*10,:)];
        plot(sorted(i+(j-1)*10,1),sorted(i+(j-1)*10,2),'r.','markers',8);
%         text(sorted(i+(j-1)*10,1)+3,sorted(i+(j-1)*10,2), num2str(k) ,'color','y', 'FontSize',8);
        k = k + 1;
    end
end

%remove lens distortion
k1 =  3.763e-05; k2 = -4.2e-7; k3 = 5.4569e-10;
xc = N/2;yc = M/2;
cpoint = zeros(size(coors));
for i = 1:160
    x = coors(i,1); y = coors(i,2);
    r2 = sqrt((x - xc)^2 + (y - yc)^2);
    cpoint(i,1) = x + (x-xc)*(k1*r2 + k2*r2^2 + k3*r2^3);
    cpoint(i,2) = y + (y-yc)*(k1*r2 + k2*r2^2 + k3*r2^3);
end
%
%add 1 to each 2 d vector to make it 3 vector
sorted = coors;
coors2 = ones(160,3);
coors2(:,1:2) =cpoint;
coors = coors2;
%finish these part

[T1, xWn] = normalize(xW,3); 
[T2, coors] = normalize(coors,2);
%to build the left matrix A
A = zeros(320,8);
b = zeros(320,1);
for i = 1:length(coors)
    A(2*i-1:2*i,:)= [xWn(i,:)  zeros(1,4); ...
                     zeros(1,4) xWn(i,:)];
    b(2*i-1:2*i,:) = coors(i,1:2);
end
p = (A'*A)\A'*b;
%check error before LM
Pb = [p(1) p(2) p(3) p(4); p(5) p(6) p(7) p(8);0 0 0 1];
Pb = T2\(Pb*T1);
Pb = Pb/Pb(3,4);
xc = xW*Pb';
xnew = xc(:,:)./xc(:,3); 
% for i = 1: length(xnew)
%     plot(xnew(i,1),xnew(i,2),'b.','markers',8);
% end
error = sum(sum((coors2 - xnew).^2))/320;
fprintf('The error before LM is %d',error);
%finished checking

%% for the algebraic error
%lvberge optimization
% [M,N] = size(coors);
% obj = zeros(M*N+1,1);
% obj(1:M*N,:) = reshape(coors',[M*N,1]);
% % % obj(M*N,1) = 0;
% opt = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');
% [q,res] = lsqcurvefit(@fun,p,xWn,obj,[],[],opt); % Refined parameter
% P = [q(1) q(2) q(3) q(4); q(5) q(6) q(7) q(8);0 0 0 1];
% P = T2\P*T1;
%%
%%for reprojection error
% [M,N] = size(coors);
% [M2,N2] = size(xWn);
% obj = zeros(M*N+1,1);
% obj(1:M*N,:) = reshape(coors',[M*N,1]);
% para = zeros(8+3*M2,1);
% para(1:8,:) = p; para(9:end,:) = reshape(xWn(:,1:3)',[3*M2,1]);
% opt = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');
% [q,res] = lsqcurvefit(@funprojection,para,xWn,obj,[],[],opt); % Refined parameter
% P = [q(1) q(2) q(3) q(4); q(5) q(6) q(7) q(8);0 0 0 1];
% xWn(1:end,1:3) = reshape(q(9:end),[3,M2])';
% P = T2\P*T1;
% xW = xWn*(T1')^(-1);


%%sampson error 
[M,N] = size(coors);
obj = zeros(M+1,1);
global coors;
opt = optimoptions('lsqcurvefit','Algorithm','levenberg-marquardt');
[q,res] = lsqcurvefit(@funsampson,p,xWn,obj,[],[],opt); % Refined parameter
P = [q(1) q(2) q(3) q(4); q(5) q(6) q(7) q(8);0 0 0 1];
P = T2\P*T1;

%check error after lv
xc = xW*P';
xnew = xc(:,:)./xc(:,3); 
for i = 1: length(xnew)
    plot(xnew(i,1),xnew(i,2),'b.','markers',8);
end
error2 = sum(sum((coors2 - xnew).^2))/360;
fprintf('The error after LM is %d\n',error2);
%finshing ehcking
end

function [xW] = buildWorldCoord(num) 
   
% num is the horinontal line numbers;
    xW =zeros(160,4);
%construct the world coordinate
    for k = 1:num
        %coordinate for the first plane
        xW(16*(k-1)+1:16*k-8,1) = 0:24:168; % Corners in world plane (mm)
        xW(16*(k-1)+1:16*k-8,2) = 24*(k-1);
        xW(16*(k-1)+1:16*k-8,3) = 0; 
        xW(16*(k-1)+1:16*k-8,4) = 1; 
        %coordinate for the second plane
        xW(16*k-7:16*k,1) = 192; % Corners in world plane (mm)
        xW(16*k-7:16*k,2) = 24*(k-1);
        xW(16*k-7:16*k,3) = 24:24:192;
        xW(16*k-7:16*k,4) = 1;
    end
end

%%provide cost function for analysis
function [ F ] = fun(p,X)
% This function describes the camera projection
% from the world plane to the image plane.
% This function is used for the Levenberg-Marquardt method.
    P = [p(1) p(2) p(3) p(4); p(5) p(6) p(7) p(8);0 0 0 1]; % Intrinsic parameter matrix K
    [M,~] = size(X);
    F = zeros(3*M+1,1);
    for k = 1:M
         Y = P * X(k,:)';
         F(3*k-2:3*k,:) = Y'/Y(3);
    end
%     error = sum(sum((coors - F).^2))/320
    F(3*M+1,1) =  [p(1) p(2) p(3)]*[p(5) p(6) p(7)]';
end
function [ F ] = funprojection(p,X)
% This function describes the camera projection
% from the world plane to the image plane.
% This function is used for the Levenberg-Marquardt method.
    P = [p(1) p(2) p(3) p(4); p(5) p(6) p(7) p(8);0 0 0 1]; % Intrinsic parameter matrix 
    len = length(p);
    X = reshape(p(9:end),[3 (len-8)/3])';
    [M,~] = size(X);
    F = zeros(3*M+1,1);
    for k = 1:M
         Y = P * [X(k,:) 1]';
         F(3*k-2:3*k,:) = Y'/Y(3);
    end
%     error = sum(sum((coors - F).^2))/320
    F(3*M+1,1) =  [p(1) p(2) p(3)]*[p(5) p(6) p(7)]';
end
function [ F ] = funsampson(p,X)
% This function describes the camera projection
% from the world plane to the image plane.
% This function is used for the Levenberg-Marquardt method.
   global coors;
   P = [p(1) p(2) p(3) p(4); p(5) p(6) p(7) p(8);0 0 0 1]; % Intrinsic parameter matrix 
   [M, ~] = size(X);
   F = zeros(M+1,1);
   for k = 1:M
       Y = P * X(k,:)';
       e = Y(1:2)/Y(3) - coors(k,1:2)';
       J = [p(1) p(2) p(3) -1 0;p(5) p(6) p(7) 0 -1];
       error = e'*(J*J')^(-1)*e;
       F(k) = sqrt(error);
   end
   F(M+1,1) =  [p(1) p(2) p(3)]*[p(5) p(6) p(7)]';
end