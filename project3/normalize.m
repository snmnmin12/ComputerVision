function [T,M] = normalize(A,ndim)
%This is to normalize in 3 D case
    if (isempty(A))
        printf('size of matrix is not right!');
    end
    if (ndim == 2)
        nordis = sqrt(2);
    elseif (ndim == 3)
        nordis = sqrt(3);
    else 
        printf('we can not handle this');
    end
    [m,n] = size(A);
    Mean = mean(A);
    total = sum(sqrt(sum((A - Mean).^2,2)));
    s = m*nordis/total;
    lc = [-s*Mean(1) -s*Mean(2) 1];
    if (ndim == 3)
        lc = [-s*Mean(1) -s*Mean(2) -s*Mean(3) 1];
    end
    T = s*eye(n);
    T(:,n) = lc';
    M = A*T';
end