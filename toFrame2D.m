function [pf, J_pf_f, J_pf_p] = toFrame2D(F, p)

T = F(1:2);
th = F(3);

R = [cos(th) -sin(th); sin(th) cos(th)];

pf = R' * (p - T);

J_pf_p = R';

x=T(1);
y=T(2);
px=p(1);
py=p(2);

J_pf_f =[...
[ -cos(th), -sin(th),   cos(th)*(py - y) - sin(th)*(px - x)]
[  sin(th), -cos(th), - cos(th)*(px - x) - sin(th)*(py - y)]
];

end

function f()
%%
syms x y th px py real
F=[x;y;th];
p=[px;py];
pf=toFrame2D(F,p);

J_pf_f=simplify(jacobian(pf, F))
end
