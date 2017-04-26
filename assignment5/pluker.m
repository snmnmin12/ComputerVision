function [pla] = pluker(A)
    pla = zeros(size(A));
    pla(1,2) = A(3,4);
    pla(2,1) = -pla(1,2);
    pla(1,3) = A(4,2);
    pla(3,1) = -pla(1,3);
    pla(1,4) = A(2,3);
    pla(4,1) = -pla(1,4);
    pla(2,3) = A(1,4);
    pla(3,2) = -pla(2,3);
    pla(4,2) = A(1,3);
    pla(2,4) = -pla(4,2);
    pla(3,4) = A(1,2);
    pla(4,3) = -pla(3,4);
end