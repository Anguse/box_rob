A = load('testfile.txt');

angle = A(:,2)*pi/180;
dist = A(:,3);

X = cos(angle).*dist;
Y = sin(angle).*dist;

plot(Y,X,'*')