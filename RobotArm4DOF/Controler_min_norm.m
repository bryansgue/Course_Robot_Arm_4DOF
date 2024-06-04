function [Vref]=Controler_min_norm(l2,l3,l4,q,he,hdp)
%% JACOBIANO
q1 = q(1);
q2 = q(2);
q3 = q(3);
q4 = q(4);

J = jacobiana_Brazo4DOF(l2,l3,l4,q1,q2,q3,q4);

%% MATRIZ DE GANANCIA
K = 1*eye(3);

Vref=pinv(J)*(hdp + K*tanh(0.5*he));

end