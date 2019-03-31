function [p, J_p_f, J_p_pf] = fromFrame2D(F, pf)
    
T = F(1:2);
th = F(3);

R = [cos(th) -sin(th) ; sin(th) cos(th)];

p = R*pf + T;

J_p_pf = R;

pfx = pf(1);
pfy = pf(2);

J_p_f = [...
    
[1, 0, - pfy*cos(th) - pfx*sin(th)]
[0, 1,   pfx*cos(th) - pfy*sin(th)]
];

end


function f()
%%
syms x y th real
F = [x;y;th];
syms pfx pfy real
pf = [pfx;pfy];
p = fromFrame2D(F, pf);

J_p_f = simplify(jacobian(p,F))
end