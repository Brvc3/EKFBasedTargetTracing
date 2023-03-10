clear;
rng(14);    
T = 1;      % 采样间隔
simTime = 60; % 时间
tspan = 0:T:simTime;
s0 = [10,-0.2,-5,0.2]; %初始状态变量 [rx;ux;ry;uy]
P0 = diag([100,1,100,1]);%初始状态变量协方差
Q = diag([0; 0.0001; 0; 0.0001]); % 过程激励噪声协方差
R = diag([2e-6;0.1]); % 测量噪声协方差

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

%% 横轴rx 纵轴ry
figure
set(gca,'Fontname','times new Roman');
set(gca,'FontSize',20);
plot(trueStates(1,:),trueStates(3,:),DisplayName="真实轨迹")
hold on
plot(trueStates(1,1),trueStates(3,1),"*",DisplayName="初始状态")
xlabel("{r_x}[n](m)",'FontSize',20)
ylabel("{r_y}[n](m)",'FontSize',20)
title("真实轨迹",'FontSize',20)
%axis square

%% 横轴n 纵轴R&beta
figure
plot(tspan,measurements(1,:)*180/pi)
xlabel("时间n (s)",'FontSize',20)
ylabel("$\hat{\beta} (deg)$",'Interpreter','latex','FontSize',20)
title("观测值\beta",'FontSize',20)
figure
plot(tspan,measurements(2,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("$\hat{R}(m)$",'Interpreter','latex','FontSize',20)
title("观测值R",'FontSize',20)

%% 扩展卡尔曼滤波
filter = extendedKalmanFilter(State=[10.2,-0.2,-4.8,0.2],StateCovariance=P0, ...
    StateTransitionFcn=@stateModel,ProcessNoise=Q, ...
    MeasurementFcn=@measureModel,MeasurementNoise=R);
estimateStates(:,1) = filter.State;

for i=2:length(tspan)
    predict(filter,T);
    estimateStates(:,i) = correct(filter,measurements(:,i));
end

%% n-rx/rx_e
figure
subplot(2,1,1)
plot(tspan,trueStates(1,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("状态方程决定的{r_x}[n]",'FontSize',20)
title("状态方程决定的r_x和EKF估计得到的r_x",'FontSize',20)
subplot(2,1,2)
plot(tspan,estimateStates(1,:))
xlabel("时间n (s)")
ylabel("${\hat{r}_x}[n]$",'Interpreter','latex','FontSize',20)

%% n-ry/ry_e
figure
subplot(2,1,1)
plot(tspan,trueStates(3,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("状态方程决定的{r_y}[n]",'FontSize',20)
title("状态方程决定的r_y和EKF估计得到的r_y",'FontSize',20)
subplot(2,1,2)
plot(tspan,estimateStates(3,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("${\hat{r}_y}[n]$",'Interpreter','latex','FontSize',20)

%% n-ux/ux_e
figure
subplot(2,1,1)
plot(tspan,trueStates(2,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("状态方程决定的{u_x}[n]",'FontSize',20)
title("状态方程决定的u_x和EKF估计得到的u_x",'FontSize',20)
subplot(2,1,2)
plot(tspan,estimateStates(2,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("${\hat{u}_x}[n]$",'Interpreter','latex','FontSize',20)

%% n-uy/uy_e
figure
subplot(2,1,1)
plot(tspan,trueStates(4,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("状态方程决定的{u_y}[n]",'FontSize',20)
title("状态方程决定的u_y和EKF估计得到的u_y",'FontSize',20)
subplot(2,1,2)
plot(tspan,estimateStates(4,:))
xlabel("时间n (s)",'FontSize',20)
ylabel("${\hat{u}_y}[n]$",'Interpreter','latex','FontSize',20)

%% 
figure
plot(trueStates(1,1),trueStates(3,1),"r*",DisplayName="初始状态")
hold on
plot(trueStates(1,:),trueStates(3,:),"r",DisplayName="真实轨迹")
xlabel("x (m)",'FontSize',20)
ylabel("y (m)",'FontSize',20)
plot(estimateStates(1,1),estimateStates(3,1),"b*",DisplayName="初始估计")
plot(estimateStates(1,:),estimateStates(3,:),"b",DisplayName="预测轨迹")
legend(Location="southwest")
title("真实轨迹和预测轨迹",'FontSize',20)
axis equal

%% check
figure
plot(tspan,abs(trueStates(1,:)-estimateStates(1,:)),"r",DisplayName="x坐标误差")
hold on;
plot(tspan,abs(trueStates(3,:)-estimateStates(3,:)),"b",DisplayName="y坐标误差")
axis([0,simTime, 0, 2])
xlabel("n (s)",'FontSize',20)
ylabel("误差 (m)",'FontSize',20)
legend(Location="southwest")
title("真实轨迹和预测轨迹误差",'FontSize',20)

