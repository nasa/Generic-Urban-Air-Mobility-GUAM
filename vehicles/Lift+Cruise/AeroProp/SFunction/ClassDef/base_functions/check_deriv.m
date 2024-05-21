function [] = check_deriv(fun)
% Specify the center

%x =[ u v w p q r]
x = [ 10 0 1 0 0 0];

%u = [ T To del i]
u = [3 0 0 pi/4];

z = [2 1 1 1 1 1];

v = [1 1 0.26 pi/8];

ep = (-1:.001:1)';

b = [1 1 1 1 1 1 1.0 .5];

X = x+ep*z;
U = u+ep*v;
B = repmat(b,length(ep),1);

[y, y_x, y_u] = fun(X, U, B);

Dy_zv = [y_x y_u]*[z v]';

Dy_zv_fd = diff(y)./diff(ep) ;
Dy_zv_fd = [ Dy_zv_fd; Dy_zv_fd(end) ];

figure
  plot(ep, y)
  grid on, zoom on
title('$f( x+ \epsilon z, u + \epsilon v)$')

figure
  plot(ep, [ Dy_zv Dy_zv_fd])
  grid on, zoom on
title('$ \frac{d}{d \epsilon} f( x+ \epsilon z, u + \epsilon v$)')
