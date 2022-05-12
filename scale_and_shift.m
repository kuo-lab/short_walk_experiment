function [ff_dif] = scale_and_shift(P1,P2,FF1,FF2,x0)
scale = x0(1);
shift = x0(2);
P2_shift = (P2*scale) + shift;
halfp1 = P1(FF1(2:end-2))+0.5*diff(P1(FF1(2:end-1)));
ff_dif = sum((halfp1-P2_shift(FF2(2:length(halfp1)+1))).^2);

end