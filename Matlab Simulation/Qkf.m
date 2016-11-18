function result = Qkf(acc,gyro,mag,dt,extAcc)

  global G;
  global M;
  global X_prev;
  global P_prev;
  global I;
  global ra;
  global rg;
  global rm;
  
  %dt = 1/1600;
  
  %measurement
  Za = acc;
  Zm = mag;
  
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
  skewX = [X(2)*(-1) X(3)*(-1) X(4)*(-1);skewQ];
  Q = dt*dt/4*skewX*rg*(skewX');
  P = A*P_prev*A'+ Q;
  
  
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
  
  %calculate Ra.
  Ra = 0.25*skewX*ra*(skewX');
  Rm = 0.25*skewX*rm*(skewX');
  Rtop = [Ra,zeros(4)];
  Rbtm = [zeros(4),Rm];
  R = [Rtop;Rbtm];
  
  %update
  S = H*P*H'+R;
  K = (P*H')/S;
  
  %if external acceleration, kalman gain for acceleration should be 0.
  if(extAcc == 1)
      K(1:4,1:4)=0
  end
  
  X_updated = (I-K*H)*X;
  X_updated = X_updated/norm(X_updated);
  P_updated = (I-K*H)*P*(I-K*H)' + K*R*K';
  
  %clean up
  %X = X_updated;
  X_prev = X_updated;
  %P = P_updated;
  P_prev = P_updated;
  
  result = X_updated;
  
end
  

  

  

  
 

