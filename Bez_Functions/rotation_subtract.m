function y = rotation_subtract(u,v)
%     R(e) = R(u)*R(v)';
    cu = cos(u);cv=cos(v);
    su = sin(u);sv=sin(v);
    r11 = cu*cv+su*sv;
    r21 = su*cv-cu*sv;
    y = atan2(r21,r11);
end