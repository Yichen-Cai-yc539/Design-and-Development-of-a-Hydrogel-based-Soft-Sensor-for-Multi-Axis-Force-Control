function F = f2delt(rota_o, rota, Fo)

% S = rovec2mat(rota_o);
% R = rovec2mat(rota);


S = rotationVectorToMatrix(rota_o);
R = rotationVectorToMatrix(rota);
Fo = Fo';
% Fo = [0 0 sum(f1, f2, f3, f4)]';

Fos = S * Fo;
F = R \ Fos;

end