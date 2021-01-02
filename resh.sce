clear
Tk = 273.16;
K = [1, log(133.5), (log(133.5))^3; 
     1, log(10.000), (log(10.000))^3;
     1, log(0.6744), (log(0.6744))^3];
k = [-1/(Tk -25);-1/(Tk + 25);-1/(Tk + 100)];
karr=linsolve(K,k);

R = 0.340:0.01:133.000;
A = karr(1,1);
B = karr(2,1);
C = karr(3,1);


T = (A + B*log(R) +  C* (abs(log(R)))^3)^(-1) - Tk;
plot (R, T)
