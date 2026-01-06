clc
syms p q r phi theta psi phidot thetadot psidot

cp = cos(phi);      sp = sin(phi);
ct = cos(theta);    st = sin(theta);
cs = cos(psi);      ss = sin(psi);

Rz = [cs ss 0; -ss cs 0; 0 0 1];
Ry = [ct 0 -st; 0 1 0; st 0 ct];
Rx = [1 0 0; 0 cp sp; 0 -sp cp];

R = Rx*Ry*Rz;

%% euler rates to body rates
% treat euler rates as vectors, decompose in the inertial-aligned frame
clc
phidot_comp = simplify(eye(3) * [phidot; 0; 0]);
thetadot_comp = simplify(Rx * [0; thetadot; 0]);
psidot_comp = simplify(Rx * Ry * [0; 0; psidot]);

L = [phidot_comp ./ phidot, ...
    thetadot_comp ./ thetadot, ...
    psidot_comp ./ psidot]
% latexPrint(L)

%% body rates to euler rates (inv(L))
% clc
Li = simplify(inv(L));
Li = subs(Li, sin(theta)/cos(theta), tan(theta))
% latexPrint(Li)

%% functions
function latexPrint(m)
    for i = 1:width(m)
        for j = 1:height(m)
            strprnt = string(m(i, j));

            strprnt = strrep(strprnt, "sin", "\\sin");
            strprnt = strrep(strprnt, "cos", "\\cos");
            strprnt = strrep(strprnt, "tan", "\\tan");
            strprnt = strrep(strprnt, "(phi)", "\\phi");
            strprnt = strrep(strprnt, "(theta)", "\\theta");
            strprnt = strrep(strprnt, "(psi)", "\\psi");
            strprnt = strrep(strprnt, "*", "");
            fprintf(strprnt);
            if j < height(m)
                fprintf(" & ");
            end
        end
        fprintf(" \\\\\n")
    end

end
