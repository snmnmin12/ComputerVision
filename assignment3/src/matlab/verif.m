function [avg, variance] = verif(P,folder)
       filepath = strcat(folder,'/');
       files = dir(strcat(filepath,'*.jpg'));
       filenames = cell(1,length(files));
       for i = 1: length(files)
           filenames{i} = strcat(filepath,files(i).name);
       end
       [xy0 XYZ] = getpoints(filenames);
       xy_p = XYZ*P';
       xy_p = xy_p(:,:)./xy_p(:,3);
       xy_p(:,1) = xy_p(:,1);
       xy_p(:,2) = xy_p(:,2) ;
       for i = 1:length(filenames)
             filename = filenames{i};
             img = imread(filename);
             figure(i);imshow(img);hold on
             plot(xy0(i,1),xy0(i,2),'r.',xy_p(i,1),xy_p(i,2),'b.');
       end
       error = xy0-xy_p;
       avg = mean(error)
       variance = std(error)
       
end

function [xy XYZ] = getpoints(filenames)
    xy = [];
    XYZ = [];
    n = 0;
    for i = 1:length(filenames)
        filename = filenames{i};
        img = imread(filename);
        image(img);
        %axis off
        axis image
        % Initially, the list of points is empty.
        zoom on;
            %keyboard;
        [xi, yi] = ginput(1);
        hold on;
        plot(xi, yi, 'ro')
        hold off;
        n = n+1;
        xy = [xy ; xi yi]; % add a new column with the current values
        input = inputdlg('[X Y Z]'); % show input dialog
        XYZi = str2num(input{1}); % convert to number
        XYZ =[XYZ; XYZi]; % add a new column with the current values
        close;
    end
end