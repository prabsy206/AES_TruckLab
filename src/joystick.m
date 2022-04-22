joy=vrjoystick(1);
myParam=Simulink.Parameter;
myParam1=Simulink.Parameter;
myParam2=Simulink.Parameter;

r=rateControl(10);
reset(r)
for i=1:1000000

%set_param('robotROSConnectToRobotExample/steer','Gain','a');
myParam.Value=axis(joy,1)
myParam1.Value=axis(joy,3)
myParam2.Value=button(joy,23)
waitfor(r);
set_param(bdroot,'SimulationCommand','Update')
end

    
    
    