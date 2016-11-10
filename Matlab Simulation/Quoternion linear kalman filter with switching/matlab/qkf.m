G = [0;0;9.81];
sResult = zeros(3,100);
sZ = zeros(3,100);
sTrue = zeros(3,100);
X_prev = [1;0;0;0];
X_prev_true = [1;0;0;0];
X = [1;0;0;0];
P_prev = eye(4)*(0.1);
I = eye(4);
ra = eye(3)*2;
gx_true = 20;
gy_true = 45;
gz_true = 90;
rg = eye(3)*5*pi/180;
%rg = zeros(3,3);
dt = 1/100;

for counter = 1:100
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
  Z = D_true*G +tmp;
  
  X_prev_true = X_true;
  sTrue(:,counter) = D_true*G;
  
  %add process noise
  gx = gx_true+ 10*rand(1,1) + (-5);
  gy = gy_true+ 10*rand(1,1) + (-5);
  gz = gz_true+ 10*rand(1,1) + (-5);
  
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
  skew = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];
  Q = dt*dt/4*skew*rg*(skew');
  
  P = A*P_prev*A'+ Q;
  
  %X_prev = X;
  %P_prev = P;
  
  %calculate H.
  tmp = Z-G;
  Hleft = [0;tmp];
  tmp1 = Z+G;
  skewH = skewSymmetric(0,tmp1);
  Hright = [(-1)*tmp';skewH*(-1)];
  H = [Hleft,Hright];
  
  %calculate Ra.
  Ra = skew*ra*(skew');
  
  %update
  S = H*P*H'+Ra;
  K = (P*H')/S;
  X_updated = (I-K*H)*X;
  X_updated = X_updated/norm(X_updated);%normalize
  P_updated = (I-K*H)*P*(I-K*H)' + K*Ra*K';
  
  %clean up
  X = X_updated;
  X_prev = X_updated;
  P = P_updated;
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
