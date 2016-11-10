function result = ClassicEKF(acc,gyro,dt)

  global G;
  global X_prev;
  global P_prev;
  global I;
  global ra;
  global rg;
  
  %dt = 1/1600;
  
  %measurement
  Z = acc';
  
  %predict state
  temp1 = [gyro(1)*pi/180;gyro(2)*pi/180;gyro(3)*pi/180];
  magnitude = norm(temp1);
  if (magnitude<1e-4)
      magnitude = 0;
      temp1 = zeros(3,1);
  else
      temp1 = temp1/magnitude*sin(magnitude*dt/2);
  end
  a = cos(magnitude/2*dt);
  skew = skewSymmetric(a,temp1);
  A_top = [a,(temp1')*(-1)];
  A_btm = [temp1,skew];
  A = [A_top;A_btm];
  
  X = A*X_prev;
  
  skewQ = skewSymmetric(X(1),X(2:4));
  tmp1 = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];
  Q = dt*dt/4*tmp1*rg*(tmp1');
  P = A*P_prev*A'+ Q;
  
  D = quat2dcm((X'));
  Ra = D*ra*(D');
  
  %compute H.
  H = computeJacobian(X,G);
  
  %update
  S = H*P*H'+Ra;
  K = (P*H')/S;
  X_updated = X + K*(Z - D*G);
  X_updated = X_updated/norm(X_updated);
  
  H = computeJacobian(X_updated,G);
  D_updated = quat2dcm((X_updated'));
  Ra = D_updated*ra*D_updated';
  
  P_updated = (I-K*H)*P*(I-K*H)' + K*Ra*K';
  
  %clean up
  %X = X_updated;
  X_prev = X_updated;
  %P = P_updated;
  P_prev = P_updated;
  
  result = X_updated;
  
end
  

  

  

  
  
  

  
  


%for k=1:3                              
  %subplot(3,1,k);
  %plot(1:100, sResult(k,:), '-', 1:100, sZ(k,:), '--',1:100, sTrue(k,:), ':');
  %plot(1:100, sResult(k,:)-sTrue(k,:))
 %axis([1,100,-1,12]);
%end

