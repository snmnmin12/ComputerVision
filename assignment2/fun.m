function [F] = fun(h,input)
    L = length(input); % Length of xdata
    F = zeros(L,1); % Initialize function values
    Dsum = 0.0;
    for i = 1:4:L
        x = input(i);
        y = input(i+1);
        xb = input(i+2);
        yb = input(i+3);
        h11 =  h(1);h12 = h(2);h13 = h(3);
        h21 = h(4);h22= h(5);h23 = h(6);
        h31=h(7);h32=h(8);h33=h(9);
        J = [-h21 + yb*h31,-h22+yb*h32, 0, h31*x+h32*y+h33;
            h11-xb*h31, h12-xb*h32,-h31*x-h32*y-h33,0];
        epi = [0 0 0 -x -y -1 x*yb y*yb yb;x y 1 0 0 0 -x*xb -y*xb -xb]*[h11 h12 h13 h21 h22 h23 h31 h32 h33]';
        Dsum = Dsum + epi'*(J*J')^(-1)*epi;
        deltax = -J'*(J*J')^(-1)*epi;
        F(i:i+3,:) = deltax;
    end
    Dsum
end