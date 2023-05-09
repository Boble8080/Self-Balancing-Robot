w = 0.5;
m = 0.2;
c = 0.1;
i = 0.006;
g = 9.8;
l = 0.3;
q = (w+m)*(i+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((i+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (c*(i + m*l^2))*s^3/q - ((w + m)*m*g*l)*s^2/q - c*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (c*(i + m*l^2))*s^2/q - ((w + m)*m*g*l)*s/q - c*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf
