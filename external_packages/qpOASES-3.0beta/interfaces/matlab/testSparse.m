m = 50;
n = 100;

L = sprand(n, n, 0.03);
H = L' * L;
A = sprand(m, n, 0.05);

lbA = -rand(m,1);
ubA = rand(m,1); 
ub = ones(n,1);
lb = -ones(n,1);
g = 10*rand(n,1);
x = zeros(n,1);

[x, fval, exitflag, iter, lambda] = qpOASES(full(H), g, full(A), lb, ub, lbA, ubA);
iter

[x, fval, exitflag, iter, lambda] = qpOASES(H, g, A, lb, ub, lbA, ubA, x);
iter

