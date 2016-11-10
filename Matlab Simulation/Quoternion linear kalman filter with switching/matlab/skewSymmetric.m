function [Matrix]=skewSymmetric(a,X)
Matrix = [a X(3)*(-1) X(2);X(3) a X(1)*(-1);X(2)*(-1) X(1) a];
