T = readtable('3.csv')
t = table2array(T(1:size(T,1),1));
temperature = table2array(T(1:size(T,1),2));
setpoint = table2array(T(1:size(T,1),3));
control = table2array(T(1:size(T,1),4));

hold on
plot(t, temperature)
plot(t, setpoint)
xlim([0 1900])
xlabel('czas [s]')
ylabel('temperatura [°C]')
legend('Temperatura','Temperatura zadana')
hold off


