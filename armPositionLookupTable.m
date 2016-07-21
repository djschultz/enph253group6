%Lengths of members in inches
L1 = 5.91;
L2 = 5.91;
%L3 = 1.9;
L3 = 1.25;
L4 = 5.91;
L5 = 2.02;
L6 = 1.34;
r = 1.5;

threshold = acos((L2^2 + (r+L1)^2 - L3^2)/(2*L2*(r+L1)));
N = 240;
A = [0;0];
B = zeros(N,N,2);
B(:,:,1) = -0.8;
B(:,:,2) = 0.84;
theta = linspace(0,4*pi/3,N);
phi = linspace(0,4*pi/3,N);
mu = pi*ones(1,N) - theta;
%Initialize 3-dimensional matrices
%First index runs over theta
%Second index runs over phi
%Third index runs over x,y
C = zeros(N,N,2);
D = zeros(N,N,2);
F = zeros(N,N,2);
G = zeros(N,N,2);
H = zeros(N,N,2);
eta = zeros(N,N);
alpha = zeros(N,N);

%First index runs over x
%Second index runs over y
%Third index runs over theta, phi.
angles = zeros(N,N,2);


for k = 1:N
    thetaK = theta(1,k);
    C(k,:,1) = r*cos(thetaK);
    C(k,:,2) = r*sin(thetaK);
    for m = 1:N
        phiM = phi(1,m);
        alpha(k,m) = thetaK - phiM;
        G(:,m,1) = L2*cos(phiM);
        G(:,m,2) = L2*sin(phiM);
        eta(k,m) = acos((2*r^2 - 2*L2*r*cos(alpha(k,m)))/(2*r*sqrt(L2^2 + r^2 - 2*L2*r*cos(alpha(k,m))))) + acos((L2^2 + r^2 + L1^2 - L3^2 - 2*L2*r*cos(alpha(k,m)))/(2*L1*sqrt(L2^2+r^2 -2*L2*r*cos(alpha(k,m)))));
        %{
        if alpha(k,m) < threshold
            eta(k,m) = 2*pi - eta(k,m);
        end
        %}
        angles(k,m,1) = thetaK;
        angles(k,m,2) = phiM;
        D(k,m,1) = C(k,m,1) + L1*cos(eta(k,m) - mu(1,k));
        D(k,m,2) = C(k,m,2) + L1*sin(eta(k,m) - mu(1,k));
    end
end

E = G + B;
I = (G - D)*(L3 + L4)/L3 + D;
Cx = C(:,:,1);
Cy = C(:,:,2);
Gx = G(:,:,1);
Gy = G(:,:,2);
Dx = D(:,:,1);
Dy = D(:,:,2);
xPos = I(:,:,1);
yPos = I(:,:,2);
imagEta = imag(eta);
csvwrite('imagEta.csv', imagEta);
eta(logical(imag(eta))) = 0;
realEta = eta;
csvwrite('realEta.csv', realEta);
%{
for k = 1:N
    xPosVec(k*(N-1)+1:k*N,1) = I(k,:,1);
    end
%}

%csvwrite('theta.csv', theta);
%csvwrite('phi.csv', phi);
xPos(logical(imag(xPos))) = 0;
yPos(logical(imag(yPos))) = 0;

figure(1)
surf(180*theta/pi, 180*phi/pi, xPos,'Edgecolor', 'none');
xlabel('Theta');
ylabel('Phi');
zlabel('x Position');
title('x Position Surface Plot');

figure(2)
surf(180*theta/pi, 180*phi/pi, yPos, 'Edgecolor', 'none');
xlabel('Theta');
ylabel('Phi');
zlabel('y Position');
title('y Position Surface Plot');

xPos = xPos;
xPosVec = xPos(:)';
csvwrite('xPos.csv', xPosVec);

yPos = yPos;
yPosVec = yPos(:)';
csvwrite('yPos.csv', yPosVec);

%{
figure(3)
plot([1:1:N^2], xPosVec, 'o');

figure(4)
plot([1:1:N^2], yPosVec, 'o');
%}
