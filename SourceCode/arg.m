clear;
rng(14);    
T = 1;      % 采样间隔
simTime = 60; % 时间
tspan = 0:T:simTime;
s0 = [10,-0.2,-5,0.2]; %初始状态变量 [rx;ux;ry;uy]
P0 = diag([100,1,100,1]);%初始状态变量协方差
Q = diag([0; 0.0001; 0; 0.0001]); % 过程激励噪声协方差
R = diag([0.1;0.1]); % 测量噪声协方差

numSteps = length(tspan);
trueStates = NaN(4,numSteps);
trueStates(:,1) = s0;
estimateStates = NaN(size(trueStates));
measurements = NaN(2,numSteps);

for i = 2:length(tspan)
    if i ~= 1
        trueStates(:,i) = stateModel(trueStates(:,i-1),T) + sqrt(Q)*randn(4,1);  
    end
    measurements(:,i) = measureModel(trueStates(:,i)) + sqrt(R)*randn(2,1);
end

filter = extendedKalmanFilter(State=[11,-0.2,-4,0.2],StateCovariance=P0, ...
    StateTransitionFcn=@stateModel,ProcessNoise=Q, ...
    MeasurementFcn=@measureModel,MeasurementNoise=R);
estimateStates(:,1) = filter.State;

for i=2:length(tspan)
    predict(filter,T);
    estimateStates(:,i) = correct(filter,measurements(:,i));
end

ax = subplot(2,2,4)
plot(trueStates(1,1),trueStates(3,1),"r*",DisplayName="初始状态")
hold on
plot(trueStates(1,:),trueStates(3,:),"r",DisplayName="真实轨迹")
xlabel("x (m)",'FontSize',12)
ylabel("y (m)",'FontSize',12)
plot(estimateStates(1,1),estimateStates(3,1),"b*",DisplayName="初始估计")
plot(estimateStates(1,:),estimateStates(3,:),"b",DisplayName="预测轨迹")
legend(Location="southwest")
subtitle("R = diag([0.1;0.1])",'FontSize',12)
hold on