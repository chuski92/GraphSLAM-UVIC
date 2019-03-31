function [lmk,J_lmk_rob, J_lmk_y]=inv_observe(rob, y)

%polars a cartesianes
[lmkrob, J_lmkrob_y]=p2c(y);

%rob a mon
[lmk,J_lmk_rob,J_lmk_lmkrob]=fromFrame2D(rob, lmkrob);

%regla cadena
J_lmk_y=J_lmk_lmkrob*J_lmkrob_y;

end
function f()
%%
syms x y th a d real
p = [d;a];
r=[x;y;th];
[l,J_l_r,J_l_y] = inv_observe(r,p);
simplify(jacobian(l,r) - J_l_r)
simplify(jacobian(l,p) - J_l_y)
end