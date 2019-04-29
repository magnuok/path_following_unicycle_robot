%
%
MATLAB = 1;

% Não tem alcance para o fundo do corredor.
% Uma porta aberta comporta-se como o fundo do corredor.
% Porta fechada-> perturbação na curva(mínima, comparavel a um milhao de
% outras coisas(bancos)
% http://www-personal.umich.edu/~johannb/Papers/paper151.pdf
% Laser Reads 240 degrees with a vector of 682 points
% 240 -682
% 



i=0;
for k=1:1000,
    
    scan = LidarScan(lidar);
    dlmwrite('scannerdata.txt', scan, '-append');
    
    if MATLAB
        disp(['scan ', num2str(k), ' length: ', num2str(length(scan))]);
    end
    plot(scan);
    
   % pause(0.1);
end

%%
lidar=dlmread('scannerdata.txt');
k=65;
figure(2);
    disp(['scan ', num2str(k), ' length: ', num2str(length(lidar))]);
 plot(lidar(k:k,:));
 
 figure(3);
 dydx = gradient(y(:)) ./ gradient(x(:));
 dy=diff(y)./diff(lidar(k:k,:))
plot(lidar(k:k,2:end),length(lidar))
