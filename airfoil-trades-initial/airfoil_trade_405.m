m = mfoil('naca','0018', 'npanel',500);
n = 50; %steps
alpha = linspace(-14, 14, n);
mach = ones(1,n)*0.261;

cl = zeros(1, 100); % initialize array
cd = zeros(1, 100); % initialize array
cm = zeros(1,100);
converged = zeros(1, 100); % initialize array

for i = 1:n
    m.setoper('alpha', alpha(i));
    m.setoper('Re', 587136);
    m.setoper('Ma', mach(i));
    m.solve;
    if (m.glob.conv) %If converged
        cl(i) = m.post.cl; %save data
        cd(i) = m.post.cd;
        cm(i) = m.post.cm;
        converged(i) = 1;
    end
end

alpha = alpha(converged==1); %removed not converged values
mach = mach(converged==1);
cl = cl(converged==1); %removed not converged values
cd= cd(converged==1);
cm = cm(converged==1);

figure(2);
plot(alpha, cl);
title('c_l vs \alpha')
xlabel('\alpha')
ylabel('c_l')

figure(3);
plot(alpha, cd);
title('c_d vs \alpha')
ylabel('c_d')
xlabel('\alpha')

figure(4);
plot(alpha, cm);
title('c_m vs \alpha')
ylabel('c_m');
xlabel('\alpha');

%% Data Processing
vals = [alpha' cl' cd' cm']
T = array2table(vals);
T.Properties.VariableNames(1:4) = {'Alpha','C_l','C_d', 'C_m'}
writetable(T,'NACA 0018/0018.csv')
%{
writematrix(alpha','airfoil_trades.xlsx','Range','F3:F52')
writematrix(cl','airfoil_trades.xlsx','Range','G3:G52')
writematrix(cd','airfoil_trades.xlsx','Range','H3:H52')
writematrix(cm','airfoil_trades.xlsx','Range','I3:I52')
%}