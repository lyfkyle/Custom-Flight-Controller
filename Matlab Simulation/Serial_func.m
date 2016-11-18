clc;
figure;
global G;
global M;
global X_prev;
global P_prev;
global I;
global ra;
global rg;
global rm;
global i;
global sMag;
global s;
i=1;
t = [0];
m = [0];
sMag = [0;0;0];
G = [0;0;1];
X_prev = [1;0;0;0];
P_prev = eye(4)*(0.1);
I = eye(4);
ra = eye(3)*0.008;
rg = eye(3)*0.1*pi/180;
rm = eye(3)*0.1;
  
p = plot(t,m,'MarkerSize',5);
axis([-5 5 -5 5 -5 5]);
grid on;

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Clear',...
        'Position', [20 20 50 20],...
        'Callback', @stop);    

try
    s=serial('com6');
catch
    error('cant serial');
end
set(s,'BaudRate',115200,'DataBits',8,'StopBits',1,'Parity','none','FlowControl','none');
s.BytesAvailableFcnMode = 'terminator';%terminator is set to LF?line feed = "\n"?, meaning that one line has ended.
s.BytesAvailableFcn = {@callback,p};

%s.BytesAvailableFcn = @callback;

fopen(s);

pause;