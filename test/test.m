clc; clear all; close all force;


function dY = ODE_example(t,Y)

y = Y(1);

dY = 3*exp(-4*t)-2*y;

end


%%

Y0 = 1;
  
%%

t_start = 0;
dt = 0.001;
t_end = 5;
n = t_end/dt+1;
Y = Y0;
Y_data = zeros(1,n);
t_data = zeros(1,n);

% exact solution
fn_y = @(t) 2.5*exp(-2*t) - 1.5*exp(-4*t);
exact_solution = zeros(1,n);

for i = 1 : n
    t = t_start + (i-1)*dt;
    
    K1 = ODE_example(t,Y);
    K2 = ODE_example(t+dt*0.5,Y+0.5*dt*K1);
    K3 = ODE_example(t+dt*0.5,Y+0.5*dt*K2);
    K4 = ODE_example(t+dt,Y+dt*K3);
    
    Y = Y + dt*(K1 + 2*K2 + 2*K3 + K4)/6;
    
    Y_data(:,i) = Y;
    t_data(:,i) = t;
    
    exact_solution(:,i) = fn_y(t);
end
figure; hold on; grid on;
plot(t_data,exact_solution,'r-','LineWidth',2)
plot(t_data,Y_data,'b--','LineWidth',2)
legend('Exact solution','RK 4th order')
xlabel('time [sec]'); ylabel('y(t)');

