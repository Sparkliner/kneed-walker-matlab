function [ product ] = Cross2D( r1, r2 )
%CROSS2D Performs 2D 'Cross Product' on column vectors and returns a scalar.

p = cross([r1;0],[r2;0]);

product = p(end);
end

