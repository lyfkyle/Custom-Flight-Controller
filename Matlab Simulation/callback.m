function callback(s, BytesAvailable,p)
    global M;
    global i;
    global sMag;  
    
    extAcc = 0;
    
    %get all data, convert to number
    out = fscanf(s);    
    IMUdata = str2num(out);
    
    %extract mag data
    mag = (IMUdata(7:9))';
    
    %Do frame transformation. Magnetometer is mounted with different axis
    %alignment from gyro and accelero.
    Rot = [0 1 0;1 0 0;0 0 -1];
    mag = Rot*mag;
    
    %collect steady state gyro measurement by averaging intial 50 gyro
    %measurement.
    if(i<50)
    sMag = sMag+mag;
    i = i+1;
    end
    
    if (i==50)
    M = (sMag/50);
    i = 51;
    end
    
    %now execute filter
    if (i==51)
      % get accelerometer measurement 
      acc = (IMUdata(1:3))';
    
      %check if external acceleration is present
      if ((norm(acc)-1) > 0.2)
        extAcc = 1;
        disp('extAcc');
      end
    
      %collect gyro measurement
      gyro = (IMUdata(4:6))';
    
      %collect elapsedTimme
      dt = (IMUdata(10)+10000)/1000000;
    
      %Execute filter
      %Result = (ClassicEKF(acc,gyro,dt))';
      Result = (Qkf(acc,gyro,mag,dt,extAcc))';
       
      %draw simulation
      dcm = (quat2dcm(Result))';
      P1=dcm*[1;2;0];
      P2=dcm*[1;-2;0];
      set(p, 'XData',[P1(1) P2(1) -P1(1) -P2(1) P1(1)],'YData',[P1(2) P2(2) -P1(2) -P2(2) P1(2)],'ZData',[P1(3) P2(3) -P1(3) -P2(3) P1(3)]);
    
      drawnow
      axis([-5 5 -5 5 -5 5]);

    end
end