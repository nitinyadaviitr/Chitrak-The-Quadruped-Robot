clc
clear all

x_dot0 = 2
z = 0.8
g = 9.81;
Tc = sqrt(z/g)

axis(gca, 'equal');
axis([-2 2 -2 2]);
grid on;

origin =[0 0]
stride_length =0.6;
x_0 = -stride_length/2;

stride_time = Tc*log((-stride_length/2 -Tc*x_dot0)/(stride_length/2 -Tc*x_dot0)); 
i=1;
for step= 1:1:6
 i=1;   
for t=0:0.01:10
i =i+1;
time(i)=t;
x(i)= x_0*cosh(t/Tc) + Tc*x_dot0*sinh(t/Tc);
x_dot(i) = x_0*sinh(t/Tc)/Tc +x_dot0*cosh(t/Tc);

line1 = line([origin(1) (origin(1) + x(i))],[origin(2) z] , 'LineWidth',3 ,'Color','blue');
pause(0.1);
delete(line1);
%plot(time,x)
if(x(i)>stride_length/2)
    break;
end    
end
    origin = origin + [stride_length/2 0];
        
end
