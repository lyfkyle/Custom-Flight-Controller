G = [0;0;9.81];
M = [0;0;1];
sResult = zeros(3,100);
sZ = zeros(3,100);
sTrue = zeros(3,100);
X_prev = [1;0;0;0];
X_prev_true = [1;0;0;0];
X = [1;0;0;0];
P_prev = eye(4);
I = eye(4);
ra = eye(3)*0.008;
rm = eye(3)*0.1;
gx_true = 50;
gy_true = 90;
gz_true = 45;
rg = eye(3)*0.1*pi/180;
%rg = zeros(3,3);
dt = 1/100;

for counter = 1:1
  %compute true state
  temp = [gx_true*pi/180;gy_true*pi/180;gz_true*pi/180];
  magnitude = norm(temp);
  if (magnitude<1e-4)
      magnitude = 0;
      temp = zeros(3,1);
  else
      temp = temp/magnitude*sin(magnitude*dt/2);
  end
  a = cos(magnitude*dt/2);
  skew = skewSymmetric(a,temp);
  A_true_top = [a,temp'*(-1)];
  A_true_btm = [temp,skew];
  A_true = [A_true_top;A_true_btm];
  X_true = A_true*X_prev_true;
  D_true = quat2dcm((X_true'));
  
  %add measurement noise
  tmp = 4*rand(3,1)+(-2);
  %Z = D_true*G +tmp;
  Za = [1;-1;9.5];
  Zm = [0.5;0.5;0.5]
  
  X_prev_true = X_true;
  sTrue(:,counter) = D_true*G;
  
  %add process noise
  gx = gx_true%+ 10*rand(1,1) + (-5);
  gy = gy_true%+ 10*rand(1,1) + (-5);
  gz = gz_true%+ 10*rand(1,1) + (-5);
  
  %predict state
  temp1 = [gx*pi/180;gy*pi/180;gz*pi/180];
  magnitude = norm(temp1);
  if (magnitude<1e-4)
      magnitude = 0;
      temp1 = zeros(3,1);
  else
      temp1 = temp1/magnitude*sin(magnitude*dt/2);
  end
  a = cos(magnitude*dt/2);
  skew = skewSymmetric(a,temp1);
  A_top = [a,(temp1')*(-1)];
  A_btm = [temp1,skew];
  A = [A_top;A_btm];
  X = A*X_prev;
  
  %calculate Q;
  skewQ = skewSymmetric(X(1),X(2:4));
  skewX = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];
  Q = dt*dt/4*skewX*rg*(skewX');
  
  test = A*P_prev;
  test1 = test*(A');
  P = test1 + Q;
  
  %X_prev = X;
  %P_prev = P;
  
  %calculate H.
  tmp = Za-G;
  Hleft = [0;tmp];
  tmp1 = Za+G;
  skewH = skewSymmetric(0,tmp1);
  Hright = [(-1)*tmp';skewH*(-1)];
  Htop = [Hleft,Hright];
  
  tmp = Zm-M;
  Hleft = [0;tmp];
  tmp1 = Zm+M;
  skewH = skewSymmetric(0,tmp1);
  Hright = [(-1)*tmp';skewH*(-1)];
  Hbtm = [Hleft,Hright];
  H = [Htop;Hbtm];
  
  %calculate R.
  Ra = 0.25*skewX*ra*(skewX');
  Rm = 0.25*skewX*rm*(skewX');
  Rtop = [Ra,zeros(4)];
  Rbtm = [zeros(4),Rm];
  R = [Rtop;Rbtm];
  
  %update
  test2 = H*P*H';
  test2(4) = 0;
  test2(11) = 0;
  test2(18) = 0;
  test2(25) = 0;
  test2(40) = 0;
  test2(47) = 0;
  test2(54) = 0;
  test2(61) = 0;
  S = test2+R;
  K = (P*H')/S;
  test3 = K*H;
  X_updated_raw = (I-K*H)*X;
  X_updated = X_updated_raw/norm(X_updated_raw);%normalize
  P_updated = (I-K*H)*P*(I-K*H)' + K*R*K';
  P_updated_1 = (I-K*H)*P;
  
  %clean up
  %X = X_updated;
  X_prev = X_updated;
  %P = P_updated;
  P_prev = P_updated;
  
  %get result
  D_updated = quat2dcm((X_updated'));
  result = D_updated*G;
  
  sResult(:,counter) = result;
  sZ(:,counter) = Z;
  
  
end


for k=1:3                              
  subplot(3,1,k);
  plot(1:100, sResult(k,:), '-', 1:100, sZ(k,:), '--',1:100, sTrue(k,:), ':');
  %plot(1:100, sResult(k,:)-sTrue(k,:))
 %axis([1,100,-1,12]);
end

%disp(sZ);
%disp(sResult);
