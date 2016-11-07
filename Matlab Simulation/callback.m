function callback(s, BytesAvailable,p)
    global M;
    global i;
    global sMag;  
    
    extAcc = 0;
    out = fscanf(s);    
    IMUdata = str2num(out);
    
    mag = (IMUdata(7:9))';
    Rot = [0 1 0;1 0 0;0 0 -1];
    mag = Rot*mag;
    
    if(i<50)
    sMag = sMag+mag;
    i = i+1;
    end
    
    if (i==50)
    M = (sMag/50)
    i = 51;
    end
    
    if (i==51)
    %acc = (IMUdata(1:3)/32768)*2;
    acc = (IMUdata(1:3))';
    
    if ((norm(acc)-1) > 0.2)
        extAcc = 1;
        disp('extAcc');
    end
    
    %gyro = (IMUdata(4:6)/32768)*500; 
    gyro = (IMUdata(4:6))';
    
    dt = IMUdata(10)/1000000;
    
    %Result = (ClassicEKF(acc,gyro,dt))';
    Result = (Qkf(acc,gyro,mag,dt,extAcc))';
       
    %t = [t ii];
    %m = [m data(1)];
    dcm = (quat2dcm(Result))';
    P1=dcm*[1;2;0];
    P2=dcm*[1;-2;0];
    set(p, 'XData',[P1(1) P2(1) -P1(1) -P2(1) P1(1)],'YData',[P1(2) P2(2) -P1(2) -P2(2) P1(2)],'ZData',[P1(3) P2(3) -P1(3) -P2(3) P1(3)]);
    
    drawnow
%    x = x + 1;
    axis([-5 5 -5 5 -5 5]);
%    ii=ii+1;
    end
end