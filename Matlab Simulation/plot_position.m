function plot_position(q)
%P1=Quat2RotMat(q)*[1;2;0];
%P2=Quat2RotMat(q)*[1;-2;0];
P1=rotx(q(1))*roty(q(2))*rotz(q(3))*[1;2;0];
P2=rotx(q(1))*roty(q(2))*rotz(q(3))*[1;-2;0];
plot_rectangle(P1,P2);
end


function r=Quat2RotMat(q)
w=q(4);
x=q(1);
y=q(2);
z=q(3);
 
r=zeros(3,3);
r(1,1)=1-2*y*y-2*z*z;
r(1,2)=2*x*y+2*w*z;
r(1,3)=2*x*z-2*w*y;
 
r(2,1)=2*x*y-2*w*z;
r(2,2)=1-2*x*x-2*z*z;
r(2,3)=2*z*y+2*w*x;
 
r(3,1)=2*x*z+2*w*y;
r(3,2)=2*y*z-2*w*x;
r(3,3)=1-2*x*x-2*y*y;
r=r'; %matlab????????????????
end


function plot_rectangle(P1,P2)
mtx = [P1(1),P1(2);P2(1),P2(2)]^(-1)*[P1(3),P2(3)]';
x = [-2:0.1:2];
y = [-2:0.1:2];
[x y]=meshgrid(x,y);
for i=1:41
    for j=1:41
        P = [x(i,j),y(i,j)];
        if(Inside(P,[[0,0];P1(1:2)';P2(1:2)']) || Inside(P,[[0,0];P1(1:2)';-P2(1:2)']) ||  Inside(P,[[0,0];-P1(1:2)';P2(1:2)']) || Inside(P,[[0,0];-P1(1:2)';-P2(1:2)']))
            z(i,j)=mtx(1)*x(i,j)+mtx(2)*y(i,j);
        else 
            z(i,j)=NaN;
        end
    end
end

mesh(z);
end

 function flag = SameSide(p1, p2, A, B)
         cp1 = cross(B-A, p1-A);
         cp2 = cross(B-A, p2-A);
         flag = sign(dot(cp1, cp2)); % 1-???0-???-1-??
 end
     
function flag = Inside(P, Triangle)
 P(3) = 0;
 Triangle(:,3) = 0;
 A = Triangle(1, :);
 B = Triangle(2, :);
 C = Triangle(3, :);

Pa = SameSide(P,A,B,C);
 Pb = SameSide(P,B,A,C);
 Pc = SameSide(P,C,A,B);

if Pa>=0 && Pb>=0 && Pc>=0
     flag = 1;
else
     flag = 0;
end
end