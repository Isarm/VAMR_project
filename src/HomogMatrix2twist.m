function twist = HomogMatrix2twist(H)

% HomogMatrix2twist Convert 4x4 homogeneous matrix to twist coordinates
%
% Input:
% -H(4,4): Euclidean transformation matrix (rigid body motion)
%
% Output:
% -twist(6,1): twist coordinates. Stack linear and angular parts [v;w]
%
% Observe that the same H might be represented by different twist vectors
% Here, twist(4:6) is a rotation vector with norm in [0,pi]

se_matrix = logm(H);

% careful for rotations of pi; the top 3x3 submatrix of the returned
% se_matrix by logm is not skew-symmetric (bad).

v = se_matrix(1:3,4);

w = Matrix2Cross(se_matrix(1:3,1:3));

twist = [v; w];

end

% MATRIX2CROSS  Compute 3D vector corresponding to an antisymmetric matrix
%
% Computes the 3D vector x corresponding to an antisymmetric matrix M such
% that M*y = cross(x,y) for all 3D vectors y.
%
% Input: 
%   - M(3,3) : antisymmetric matrix
%
% Output: 
%   - x(3,1) : column vector
%
% See also CROSS2MATRIX

function x = Matrix2Cross(M)

x = [-M(2,3); M(1,3); -M(1,2)];

end
