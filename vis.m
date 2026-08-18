[BetaGrid,SteerGrid] = meshgrid(BodySlipAnglesDeg,SteerAnglesDeg);

%% Choose a velocity to inspect
plotVelocity = 9; % [m/s]
[~,velocityIndex] = min(abs(Velocities-plotVelocity));
actualVelocity = Velocities(velocityIndex);

ayG = squeeze(ayMap(velocityIndex,:,:))/g;
yawMoment = squeeze(yawMomentMap(velocityIndex,:,:));
valid = squeeze(convergedMap(velocityIndex,:,:));

ayG(~valid) = NaN;
yawMoment(~valid) = NaN;

%% Steering/body-slip maps
figure('Name','Steady-State Maps','Color','w');
tiledlayout(1,2,'TileSpacing','compact','Padding','compact');

nexttile;
contourf(BetaGrid,SteerGrid,ayG,20,'LineColor','none');
colorbar;
hold on;
if any(isfinite(yawMoment(:))) && min(yawMoment(:),[],'omitnan') <= 0 && ...
        max(yawMoment(:),[],'omitnan') >= 0
    contour(BetaGrid,SteerGrid,yawMoment,[0 0], ...
        'k','LineWidth',2.5,'DisplayName','M_z = 0');
end
xlabel('Body slip \beta [deg]');
ylabel('Handwheel angle [deg]');
title(sprintf('Lateral Acceleration at %.1f m/s',actualVelocity));
grid on;

nexttile;
contourf(BetaGrid,SteerGrid,yawMoment,20,'LineColor','none');
colorbar;
hold on;
if any(isfinite(yawMoment(:))) && min(yawMoment(:),[],'omitnan') <= 0 && ...
        max(yawMoment(:),[],'omitnan') >= 0
    contour(BetaGrid,SteerGrid,yawMoment,[0 0],'k','LineWidth',2.5);
end
xlabel('Body slip \beta [deg]');
ylabel('Handwheel angle [deg]');
title(sprintf('Net Yaw Moment at %.1f m/s [N m]',actualVelocity));
grid on;

%% Milliken-style moment diagram
figure('Name','Yaw Moment Diagram','Color','w');
validPoints = isfinite(ayG) & isfinite(yawMoment);
scatter(ayG(validPoints),yawMoment(validPoints),45, ...
    SteerGrid(validPoints),'filled');
yline(0,'k-','LineWidth',1.5);
xlabel('Lateral acceleration [g]');
ylabel('Net yaw moment [N m]');
title(sprintf('Yaw Moment Diagram at %.1f m/s',actualVelocity));
cb = colorbar;
cb.Label.String = 'Handwheel angle [deg]';
grid on;

%% Maximum converged ay and maximum zero-moment ay versus speed
maxConvergedAyG = NaN(size(Velocities));
maxSteadyAyG = NaN(size(Velocities));
steadyMomentTolerance = 0; % [N m]

for velocityIndex = 1:numel(Velocities)
    localAyG = squeeze(ayMap(velocityIndex,:,:))/g;
    localMoment = squeeze(yawMomentMap(velocityIndex,:,:));
    localValid = squeeze(convergedMap(velocityIndex,:,:));

    allPoints = localValid & isfinite(localAyG) & isfinite(localMoment);
    if any(allPoints(:))
        maxConvergedAyG(velocityIndex) = max(localAyG(allPoints));
    end

    steadyPoints = allPoints & abs(localMoment) <= steadyMomentTolerance;
    if any(steadyPoints(:))
        maxSteadyAyG(velocityIndex) = max(localAyG(steadyPoints));
    end
end

figure('Name','Lateral Acceleration versus Speed','Color','w');
plot(Velocities,maxConvergedAyG,'o-','LineWidth',1.8, ...
    'DisplayName','Maximum converged');
hold on;
plot(Velocities,maxSteadyAyG,'s-','LineWidth',1.8, ...
    'DisplayName',sprintf('|M_z| \leq %.0f N m',steadyMomentTolerance));
xlabel('Velocity [m/s]');
ylabel('Lateral acceleration [g]');
title('Maximum Lateral Acceleration versus Speed');
legend('Location','best');
grid on;

%% Optional: animate the ay map over velocity
% Uncomment this section to view the map changing with speed.
% figure('Name','Velocity Sweep Animation','Color','w');
% for velocityIndex = 1:numel(Velocities)
%     localAyG = squeeze(ayMap(velocityIndex,:,:))/g;
%     localMoment = squeeze(yawMomentMap(velocityIndex,:,:));
%     localValid = squeeze(convergedMap(velocityIndex,:,:));
%     localAyG(~localValid) = NaN;
%     localMoment(~localValid) = NaN;
%
%     clf;
%     contourf(BetaGrid,SteerGrid,localAyG,20,'LineColor','none');
%     colorbar;
%     hold on;
%     if any(isfinite(localMoment(:))) && ...
%             min(localMoment(:),[],'omitnan') <= 0 && ...
%             max(localMoment(:),[],'omitnan') >= 0
%         contour(BetaGrid,SteerGrid,localMoment,[0 0], ...
%             'k','LineWidth',2.5);
%     end
%     xlabel('Body slip \beta [deg]');
%     ylabel('Handwheel angle [deg]');
%     title(sprintf('Lateral Acceleration at %.1f m/s',Velocities(velocityIndex)));
%     grid on;
%     drawnow;
%     pause(0.15);
% end