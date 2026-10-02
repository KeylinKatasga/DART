function run_passive_demo
% Classroom-only vertical soft-sphere drop and fixed-pivot passive pendulum.
% No flight steering, target, actuator, or fin geometry model.
root = fileparts(fileparts(mfilename('fullpath')));
out = fullfile(root,'results','matlab');
if ~exist(out,'dir'), mkdir(out); end
g=9.81; m=0.005; rho=1.225; Cd=0.47; radius=0.035;
c=0.5*rho*Cd*pi*radius^2/m; height=1;
opts=odeset('RelTol',1e-9,'AbsTol',1e-11,'MaxStep',0.01,'Events',@ground);
[t,x]=ode45(@(~,x)[x(2);-g-c*abs(x(2))*x(2)],[0 3],[height;0],opts);
vterm=sqrt(g/c);
zexact=height-log(cosh(sqrt(g*c)*t))/c;
verror=max(abs(x(:,1)-zexact));
assert(verror<1e-7,'Drop analytic check failed');
E=m*g*x(:,1)+0.5*m*x(:,2).^2;
assert(max(diff(E))<1e-9,'Drop energy check failed');
writetable(table(t,x(:,1),x(:,2),E,'VariableNames',{'time_s','height_m','velocity_m_s','energy_J'}),fullfile(out,'drop.csv'));
L=0.3; mp=0.02; b=0.001; J=mp*L^2;
[tp,y]=ode45(@(~,y)[y(2);-g/L*sin(y(1))-b/J*y(2)],linspace(0,12,1201),[15*pi/180;0],odeset('RelTol',1e-9,'AbsTol',1e-11));
Ep=0.5*J*y(:,2).^2+mp*g*L*(1-cos(y(:,1)));
assert(max(diff(Ep))<1e-9,'Pendulum energy check failed');
writetable(table(tp,y(:,1),y(:,2),Ep,'VariableNames',{'time_s','angle_rad','rate_rad_s','energy_J'}),fullfile(out,'pendulum.csv'));
f=figure('Visible','off'); tiledlayout(2,2);
nexttile; plot(t,x(:,1)); xlabel('Time (s)'); ylabel('Height (m)'); grid on;
nexttile; plot(t,x(:,2)); xlabel('Time (s)'); ylabel('Vertical velocity (m/s)'); grid on;
nexttile; plot(tp,y(:,1)*180/pi); xlabel('Time (s)'); ylabel('Pendulum angle (deg)'); grid on;
nexttile; plot(tp,Ep); xlabel('Time (s)'); ylabel('Pendulum energy (J)'); grid on;
exportgraphics(f,fullfile(out,'passive_demo.png'),'Resolution',180); close(f);
fid=fopen(fullfile(out,'validation.txt'),'w');
fprintf(fid,'MATLAB executed: %s\nDrop time: %.9f s\nMaximum height error: %.3g m\nTerminal speed parameter: %.6f m/s\nEnergy tests passed.\n',version,t(end),verror,vterm); fclose(fid);
end
function [value,isterminal,direction]=ground(~,x)
value=x(1); isterminal=1; direction=-1;
end
