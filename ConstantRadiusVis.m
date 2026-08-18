
% Run steady_state_sweep.m first. This script expects:
% ayMap, radiusMap, yawMomentMap, convergedMap,
% Velocities, SteerAnglesDeg, and BodySlipAnglesDeg.

g = 9.80665;

requiredVariables = {'ayMap','radiusMap','yawMomentMap','convergedMap', ...
    'Velocities','SteerAnglesDeg','BodySlipAnglesDeg'};
for k = 1:numel(requiredVariables)
    assert(exist(requiredVariables{k},'var') == 1, ...
        'Run steady_state_sweep.m first. Missing variable: %s', ...
        requiredVariables{k});
end

[BetaGrid,SteerGrid] = meshgrid(BodySlipAnglesDeg,SteerAnglesDeg);

%% Plot settings
minimumRadius = 9.125;          % [m], lower edge of desired radius band
maximumRadius = 10.625;          % [m], upper edge of desired radius band
steadyMomentTolerance = 1;  % [N m], approximate equilibrium points
momentLimit = 100; % [N m], value of color bar highest color
assert(minimumRadius >= 0 && maximumRadius > minimumRadius, ...
    'Radius limits must satisfy 0 <= minimumRadius < maximumRadius.');

targetRadius = (minimumRadius + maximumRadius)/2;


%% Build steering/body-slip maps for the selected radius band
% At each steering/body-slip point, several velocities may produce a radius
% inside the requested band. Choose the velocity whose radius is closest
% to the center of the band. The selected point supplies both the velocity
% color on the left map and yaw-moment color on the right map.
allRadius = abs(radiusMap);
allValid = convergedMap & isfinite(allRadius) & isfinite(yawMomentMap);
inRadiusBand = allValid & allRadius >= minimumRadius & ...
    allRadius <= maximumRadius;

mapSize = [numel(SteerAnglesDeg),numel(BodySlipAnglesDeg)];
selectedVelocityMap = NaN(mapSize);
selectedRadiusMap = NaN(mapSize);
selectedMomentMap = NaN(mapSize);
selectedAyGMap = NaN(mapSize);

for steerIndex = 1:numel(SteerAnglesDeg)
    for betaIndex = 1:numel(BodySlipAnglesDeg)
        candidateIndices = find(inRadiusBand(:,steerIndex,betaIndex));
        if isempty(candidateIndices)
            continue
        end

        candidateRadii = allRadius(candidateIndices,steerIndex,betaIndex);
        [~,closestIndex] = min(abs(candidateRadii-targetRadius));
        chosenVelocityIndex = candidateIndices(closestIndex);

        selectedVelocityMap(steerIndex,betaIndex) = ...
            Velocities(chosenVelocityIndex);
        selectedRadiusMap(steerIndex,betaIndex) = ...
            allRadius(chosenVelocityIndex,steerIndex,betaIndex);
        selectedMomentMap(steerIndex,betaIndex) = ...
            yawMomentMap(chosenVelocityIndex,steerIndex,betaIndex);
        selectedAyGMap(steerIndex,betaIndex) = ...
            ayMap(chosenVelocityIndex,steerIndex,betaIndex)/g;
    end
end

validBandMap = isfinite(selectedVelocityMap) & isfinite(selectedMomentMap);
hasBandZeroContour = any(validBandMap(:)) && ...
    min(selectedMomentMap(validBandMap)) <= 0 && ...
    max(selectedMomentMap(validBandMap)) >= 0;

%% Radius-filtered maps: velocity and yaw moment side by side
figure('Name','Selected Radius Band: Velocity and Yaw Moment');

t = tiledlayout(1,2,'TileSpacing','compact','Padding','compact');
t.Title.String = sprintf('Configurations with %.1f m \leq |R| \leq %.1f m', ...
    minimumRadius,maximumRadius);

velocityAxes = nexttile(t);
if any(validBandMap(:))
    numberOfVelocityLevels = max(12,numel(Velocities));
    velocityLevels = linspace(min(Velocities),max(Velocities), ...
        numberOfVelocityLevels);

    contourf(velocityAxes,BetaGrid,SteerGrid,selectedVelocityMap, ...
        velocityLevels,'LineColor','none');
    hold(velocityAxes,'on');

    if hasBandZeroContour
        contour(velocityAxes,BetaGrid,SteerGrid,selectedMomentMap, ...
            [0 0],'k','LineWidth',2.8);
    end

    nearSteady = validBandMap & ...
        abs(selectedMomentMap) <= steadyMomentTolerance;
    if any(nearSteady(:))
        scatter(velocityAxes,BetaGrid(nearSteady),SteerGrid(nearSteady), ...
            18,'o','MarkerEdgeColor','k','LineWidth',0.5);
    end

    cb = colorbar(velocityAxes);
    cb.Label.String = 'Vehicle velocity [m/s]';
    colormap(velocityAxes,turbo);
    clim(velocityAxes,[min(Velocities) max(Velocities)]);
else
    text(velocityAxes,0.5,0.5,'No configurations in radius band', ...
        'Units','normalized','HorizontalAlignment','center', ...
        'FontWeight','bold');
end
xlabel(velocityAxes,'Body slip \beta [deg]');
ylabel(velocityAxes,'Handwheel angle [deg]');
title(velocityAxes,'Colored by Velocity');
grid(velocityAxes,'on');
box(velocityAxes,'on');
set(velocityAxes,'Layer','top');

momentAxes = nexttile(t);
if any(validBandMap(:))
    finiteMoments = selectedMomentMap(validBandMap);
    if momentLimit < eps
        momentLimit = 1;
    end
    momentLevels = linspace(-momentLimit,momentLimit,25);

    contourf(momentAxes,BetaGrid,SteerGrid,selectedMomentMap, ...
        momentLevels,'LineColor','none');
    hold(momentAxes,'on');

    if hasBandZeroContour
        contour(momentAxes,BetaGrid,SteerGrid,selectedMomentMap, ...
            [0 0],'k','LineWidth',2.8);
    end

    cb = colorbar(momentAxes);
    cb.Label.String = 'Net yaw moment [N m]';

    % Blue-white-red diverging map: blue is negative, red is positive.
    nColors = 256;
    lowerMap = [linspace(0,1,nColors/2)', ...
                linspace(0,1,nColors/2)', ...
                ones(nColors/2,1)];
    upperMap = [ones(nColors/2,1), ...
                linspace(1,0,nColors/2)', ...
                linspace(1,0,nColors/2)'];
    momentColorMap = [lowerMap; upperMap];
    colormap(momentAxes,momentColorMap);
    clim(momentAxes,[-momentLimit momentLimit]);
else
    text(momentAxes,0.5,0.5,'No configurations in radius band', ...
        'Units','normalized','HorizontalAlignment','center', ...
        'FontWeight','bold');
end
xlabel(momentAxes,'Body slip \beta [deg]');
ylabel(momentAxes,'Handwheel angle [deg]');
title(momentAxes,'Colored by Net Yaw Moment');
grid(momentAxes,'on');
box(momentAxes,'on');
set(momentAxes,'Layer','top');

linkaxes([velocityAxes momentAxes],'xy');
fontsize(18,"points")
