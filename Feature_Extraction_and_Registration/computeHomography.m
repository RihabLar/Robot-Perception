function H12 = computeHomography(CL1uv, CL2uv, Model)
    % COMPUTEHOMOGRAPHY Computes the homography matrix between two sets of points
    % based on the specified transformation model.
    %
    % INPUTS:
    %   CL1uv - Nx2 matrix of coordinates [u1, v1] in the first image.
    %   CL2uv - Nx2 matrix of corresponding coordinates [u2, v2] in the second image.
    %   Model - String specifying the transformation model: 'Translation', 'Similarity', 'Affine', or 'Projective'.
    %
    % OUTPUT:
    %   H12 - Homography matrix describing the planar transformation.

    switch Model
        case 'Translation'
            % Translation Model
            % Computes the translation in the x and y directions.
            tx = mean(CL1uv(:,1) - CL2uv(:,1));
            ty = mean(CL1uv(:,2) - CL2uv(:,2));
            % Construct the translation homography matrix
            H12 = [1 0 tx; 0 1 ty; 0 0 1];

        case 'Similarity'
            % Similarity Model
            % Assumes uniform scaling (s), rotation (θ), and translation (tx, ty)
            % The transformation equations are:
            % u2 = s * (cos(θ) * u1 - sin(θ) * v1) + tx
            % v2 = s * (sin(θ) * u1 + cos(θ) * v1) + ty
            N = size(CL1uv, 1);
            A = zeros(2*N, 4);
            B = zeros(2*N, 1);

            for i = 1:N
                u1 = CL1uv(i,1); v1 = CL1uv(i,2);
                u2 = CL2uv(i,1); v2 = CL2uv(i,2);
                A(2*i-1,:) = [u2 -v2 1 0];
                A(2*i,:)   = [v2 u2 0 1];
                B(2*i-1) = u1;
                B(2*i) = v1;
            end

            % Solve the system of equation
            Q = A \ B;
            a = Q(1); b = Q(2); tx = Q(3); ty = Q(4);
            % a = s * cos(θ)
            % b = s * sin(θ)
            H12 = [a -b tx; b a ty; 0 0 1];

        case 'Affine'
            % Affine Model
            % Accounts for linear transformations including scaling, rotation, translation, and shear.
            % The transformation equations are:
            % u2 = a*u1 + b*v1 + tx
            % v2 = c*u1 + d*v1 + ty
            N = size(CL1uv, 1);
            A = zeros(2*N, 6);
            B = zeros(2*N, 1);

            for i = 1:N
                u1 = CL1uv(i,1); v1 = CL1uv(i,2);
                u2 = CL2uv(i,1); v2 = CL2uv(i,2);
                A(2*i-1,:) = [u2 v2 1 0 0 0];
                A(2*i,:)   = [0 0 0 u2 v2 1];
                B(2*i-1) = u1;
                B(2*i) = v1;
            end

            % Solve the system of equations
            Q = A \ B;
            a = Q(1); b = Q(2); tx = Q(3);
            c = Q(4); d = Q(5); ty = Q(6);
            % Construct the affine homography matrix
            H12 = [a b tx; c d ty; 0 0 1];

        case 'Projective'
            % Projective Model
            % The transformation equations are:
            % u2 = (a*u1 + b*v1 + tx) / (g*u1 + h*v1 + 1)
            % v2 = (c*u1 + d*v1 + ty) / (g*u1 + h*v1 + 1)
            N = size(CL1uv, 1);
            A = zeros(2*N, 8);
            B = zeros(2*N, 1);

            for i = 1:N
                u1 = CL1uv(i,1); v1 = CL1uv(i,2);
                u2 = CL2uv(i,1); v2 = CL2uv(i,2);
                A(2*i-1,:) = [u2 v2 1 0 0 0 -u2*u1 -u2*v1];
                A(2*i,:)   = [0 0 0 u2 v2 1 -v2*u1 -v2*v1];
                B(2*i-1) = u1;
                B(2*i) = v1;
            end

            % Solve the system of equations
            Q = A \ B;
            a = Q(1); b = Q(2); tx = Q(3);
            c = Q(4); d = Q(5); ty = Q(6);
            g = Q(7); h = Q(8);
            % Construct the projective homography matrix
            H12 = [a b tx; c d ty; g h 1];

        otherwise
            % Handle invalid models
            warning('Invalid model, returning identity homography');
            H12 = eye(3);
    end
end
