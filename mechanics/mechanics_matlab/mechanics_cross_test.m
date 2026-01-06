clear
close
clc

syms p q r
x = sym('x', [3 1]);
pqr = [p; q; r];

% skew-symmetric matrix of angular vels
A = ssk(pqr);

% products are equivalent
all(simplify(A*x == cross([p; q; r], x)))

%% functions
function A = ssk(pqr)
% skew-symmetric matrix of angular vels
p = pqr(1);
q = pqr(2);
r = pqr(3);
A = [0 -r q; r 0 -p; -q p 0];
end
