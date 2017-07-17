%input1 is the points of interest from image 1 and input2 are the matching
%points for image 2
input1= [536,137,563,131,582,126,609,118,632,112,658,107,540,176,563,171,584,...
    166,609,159,635,155,662,149,567,376,591,376,617,376,643,375,672,377,637,460,616,464,660,480];
input2= [7,122,35,122,56,120,82,119,106,116,132,115,5,165,31,163,54,161,...
    79,161,104,159,130,158,14,377,40,375,66,375,91,373,118,374,66,457,44,465,84,476];
%%initial homograph h is from question 2 
h = [0.6001022330089905, -0.07108177426335195, -308.1795157075791;
 0.1659014328938462, 0.5698649001899936, -99.94326770965529;
 0.0004723017188832685, -2.326302039590104e-05, 0.2948658686628874];

%%processing the input
input = [];
for i = 1:2:length(input1)
    input = [input; input1(i); input1(i+1); input2(i); input2(i+1)];
end
h1 = reshape(h',9,1);
opt = optimset('Algorithm','levenberg-marquardt','TolFun',1e-8);
input2 = zeros(length(input),1);
[h2,res] = lsqcurvefit(@fun,h1,input,input2,[],[],opt); % Refined homography by L.M
H = reshape(h2,3,3)';
sqrt(res)
%% start with image processing now
im1 = imread('Picture1.png');
im2 = imread('Picture2.png');
[M,N,C] = size(im1);
boundary1 = [1 1 1;N 1 1;1 M 1;N M 1]'; 
boundary2 = H*boundary1;
for i = 1:4
    boundary2(:,i) = boundary2(:,i)/boundary2(end,i);
end
xmin = round(min(boundary2(1,:)));
xmax = round(max(boundary2(1,:)));
ymin = round(min(boundary2(2,:)));
ymax = round(max(boundary2(2,:)));
%initialize the output image
height = ymax - ymin + 1;
width = xmax - xmin + 1;
img = zeros(ymax-ymin+1,xmax-xmin+1,C); % Initialize output image 
%% Assign pixel values into the output image %%
x_new = [0 0 1]'; % Homogeneous coordinate in new plane
for m = 1 : height
    x_new(2) = m + ymin - 1;
    for n = 1 : width
        x_new(1) = n + xmin - 1;
         for c = 1 : C
             x_org = H \ x_new;
             x = x_org(1) / x_org(3);
             fx = x - fix(x);
             y = x_org(2) / x_org(3);
             fy = y - fix(y);
             if (1 <= x && x <= N && 1 <= y && y <= M)
             % Use bilinear interpolation
              img(m,n,c) = (1 - fx) * (1 - fy) * im1(fix(y),fix(x),c) +...
             (1 - fx) * fy * im1(ceil(y),fix(x),c) +...
             fx * (1 - fy) * im1(fix(y),ceil(x),c) +...
             fx * fy * im1(ceil(y),ceil(x),c);
             end
          end
     end
end
img = uint8(img);
img(1-ymin:M-ymin,1-xmin:N-xmin,:) = im2; %  copyt image 2 to output image
figure; imshow(img); 
% imwrite(img,'q3_out.png');

