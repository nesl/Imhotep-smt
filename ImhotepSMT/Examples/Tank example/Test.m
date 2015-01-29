t = 120;
noise_power = 0;
tt = t - 1;
x                                               = X_linear(tt-n+1,:)'
E                                               = attack(:,tt-n+1:tt)';
for mycounter = 1 : p
    Y_bar{mycounter} = O_SMT{mycounter}*x + 20*E(:,mycounter);% + noise_power*randn(length(Y_bar{mycounter}),1);
end
smt.init(n,p,tau,1);
for counter = 1 : p
    smt.addSensorMeasurements(Y_bar{counter}, O_SMT{counter}, 3*noise_power, counter); %noisepower is multilplied by n*p to account for the overall noise over all sensors over all measurments per sensor
end
smt.markSensorAsSafe(1);

[xhat, Khatt] = smt.solve()