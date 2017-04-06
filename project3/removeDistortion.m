function [k1 k2 k3] = removeDistortion()
    clc;
    clear all;
    close all;
    %The size of all corners if 10 x 8 with total of 80 points
    %define and initialize world coordinates that were measured with the ruler
    filename = 'picture2.jpg';
    % corner = get_corners(filename,10,8,1);
    im_RGB = imread(filename);
    im_gray = rgb2gray(im_RGB); % Gray-scale image
    [M,N] = size(im_gray);

    corners = corner(im_gray,80);
    %sort the corner in a good way
    %sort the to start
    sorted = sortrows(corners);
    cpoint = []
    for i = 1:10:80
        temp = sorted(i:i+9,:);
        [~,I] = sort(temp(:,2));
        cpoint = [cpoint;temp(I,:)];
    end
    
    sorted = cpoint;coors = [];
    figure;imshow(im_RGB);hold on;
    k = 1;
    for i = 1:10
        for j = 1:8
            coors = [coors; sorted(i+(j-1)*10,:)];
            plot(sorted(i+(j-1)*10,1),sorted(i+(j-1)*10,2),'r.','markers',8);
            text(sorted(i+(j-1)*10,1)+3,sorted(i+(j-1)*10,2), num2str(k) ,'color','y', 'FontSize',8);
            k = k + 1;
        end
    end
    %sort the corners completed
    %sort the corners
    
    BW = edge(im_gray,'canny',0.7); % Binary image from Canny detector
    BW = zeros(M,N);
    global hori_line;
    global vert_line;
    %line equation for horizontal and vertial
    hori_line = zeros(2,3);
    vert_line = zeros(2,3);
    for i = 1:9:10
        index = (i-1)*8+1;
        hori_line(floor(i/10)+1,:) =  cross([coors(index,:) 1],[coors(index+7,:) 1]);
    end
    for i = 1:7:8
        index = 72+i;
        vert_line(floor(i/8)+1,:) =  cross([coors(i,:) 1],[coors(index,:) 1]);
    end
    opt = optimset('Algorithm','levenberg-marquardt','TolFun',1e-8);
    global center;
    center = [N/2, M/2];
    p = [0.05,0.025,0];
    error = zeros(28,1);
    input = [coors(2:7,:); coors(74:79,:); coors(9:8:65,:); coors(16:8:72,:)];
    %input = [coors(2:7,:); coors(9:8:65,:)];
    [q,res] = lsqcurvefit(@funlens,p,input,error,[],[],opt); % Refined parameters
    
    %plot lines
    figure;imshow(im_RGB);hold on;
   for k = 1:2
         line = hori_line(k,:);
         x = 0:N;
         y = -(line(1)*x + line(3))/line(2);
         plot(x,y,'r');
   end 
  %plot vertical lines
   for k =1:2
         line = vert_line(k,:);
         y = 0:M;
         x = -(line(2)*y + line(3))/line(1);
         plot(x,y,'r');
   end 
    %
    %get coordinate afte radis disortion 
    xc = center(1); yc = center(2);
    k1 = q(1); k2 = q(2); k3 = q(3);
    nim = undistortimage(im_RGB, 1, xc, yc, k1, k2, 0,0);
    figure; imshow(nim);
end

function [ F ] = funlens(p,X )
    % This function describes the error function for lens distortion
    % p = [k1 k2 k3 x0 y0]
    % x = [x1 y1;x2 y2;x3 y3;....]
    % This function is used for the Levenberg-Marquardt method.
    global hori_line;
    global vert_line;
    global center
    [M,N] = size(X);
    F = zeros(M,1);
    for k = 1:M
        x = X(k,1);y = X(k,2);
        xc = center(1); yc = center(2);
        r2 = sqrt((x - xc)^2 + (y - yc)^2);
        xr = x + (x-xc)*(p(1)*r2 + p(2)*r2^2 + p(3)*r2^3);
        yr = y + (y-yc)*(p(1)*r2 + p(2)*r2^2 + p(3)*r2^3);

        if (k <= 12)
            lineh = floor((k-1)/6)+1;
            distanceh = hori_line(lineh,:)*[xr yr 1]';
            F(k) = distanceh/sqrt(hori_line(lineh,1).^2+hori_line(lineh,2).^2);
        else 
            linev = floor((k-13)/8)+1;
            %linev = floor((k-13)/8)+1;
            distancev = vert_line(linev,:)*[xr yr 1]';
            F(k) = distancev/sqrt(vert_line(linev,1).^2+vert_line(linev,2).^2);
        end
    end
    sum(F.^2)/M
end


