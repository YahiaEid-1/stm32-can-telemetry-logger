function Offline_BoardDisplay
clc; close all;

data = readtable("LOG.CSV");

t = data.time_ms / 1000;
t = t - t(1);

ax_data = data.accel_x / 4096;
ay_data = data.accel_y / 4096;
az_data = data.accel_z / 4096;

gx_data = data.gyro_x / 131;
gy_data = data.gyro_y / 131;
gz_data = data.gyro_z / 131;

roll = atan2d(ay_data, az_data);
pitch = atan2d(-ax_data, sqrt(ay_data.^2 + az_data.^2));
yaw = zeros(size(roll));

L = 1.0;
W = 2.0;
H = 0.08;

vertices = [
    -L/2 -W/2 -H/2;
     L/2 -W/2 -H/2;
     L/2  W/2 -H/2;
    -L/2  W/2 -H/2;
    -L/2 -W/2  H/2;
     L/2 -W/2  H/2;
     L/2  W/2  H/2;
    -L/2  W/2  H/2
];

faces = [
    1 2 3 4;
    5 6 7 8;
    1 2 6 5;
    2 3 7 6;
    3 4 8 7;
    4 1 5 8
];

fig = uifigure("Name", "CAN Telemetry Dashboard", "Position", [100 100 1100 700]);

ax3d = uiaxes(fig, "Position", [40 170 500 480]);
axis(ax3d, "equal");
grid(ax3d, "on");
xlabel(ax3d, "X");
ylabel(ax3d, "Y");
zlabel(ax3d, "Z");
xlim(ax3d, [-2 2]);
ylim(ax3d, [-2 2]);
zlim(ax3d, [-2 2]);
view(ax3d, 35, 25);
title(ax3d, "Offline 3D Orientation Replay");

board = patch(ax3d, ...
    "Vertices", vertices, ...
    "Faces", faces, ...
    "FaceAlpha", 0.75);

axPlot = uiaxes(fig, "Position", [590 270 460 350]);
plot(axPlot, t, roll);
hold(axPlot, "on");
plot(axPlot, t, pitch);
xlabel(axPlot, "Time (s)");
ylabel(axPlot, "Angle (deg)");
legend(axPlot, "Roll", "Pitch");
title(axPlot, "Estimated Roll and Pitch");
grid(axPlot, "on");

timeLine = xline(axPlot, t(1), "--");

timeLabel = uilabel(fig, ...
    "Position", [40 115 600 30], ...
    "Text", "Time: 0.00 s | Roll: 0.0 deg | Pitch: 0.0 deg", ...
    "FontSize", 14);

slider = uislider(fig, ...
    "Position", [80 80 900 3], ...
    "Limits", [1 length(t)], ...
    "Value", 1);

slider.MajorTicks = linspace(1, length(t), 6);
slider.MajorTickLabels = string(round(linspace(t(1), t(end), 6), 1));

playButton = uibutton(fig, ...
    "push", ...
    "Text", "Play", ...
    "Position", [80 25 100 35]);

pauseButton = uibutton(fig, ...
    "push", ...
    "Text", "Pause", ...
    "Position", [200 25 100 35]);

isPlaying = false;

slider.ValueChangingFcn = @(src, event) updateDisplay(round(event.Value));
slider.ValueChangedFcn = @(src, event) updateDisplay(round(src.Value));

playButton.ButtonPushedFcn = @(src, event) playReplay();
pauseButton.ButtonPushedFcn = @(src, event) pauseReplay();

updateDisplay(1);

    function updateDisplay(k)
        k = max(1, min(k, length(t)));

        r = deg2rad(roll(k));
        p = deg2rad(pitch(k));
        y = deg2rad(yaw(k));

        Rx = [
            1 0 0;
            0 cos(r) -sin(r);
            0 sin(r) cos(r)
        ];

        Ry = [
            cos(p) 0 sin(p);
            0 1 0;
            -sin(p) 0 cos(p)
        ];

        Rz = [
            cos(y) -sin(y) 0;
            sin(y) cos(y) 0;
            0 0 1
        ];

        R = Rz * Ry * Rx;

        rotatedVertices = (R * vertices')';

        board.Vertices = rotatedVertices;
        timeLine.Value = t(k);

        timeLabel.Text = sprintf("Time: %.2f s | Roll: %.1f deg | Pitch: %.1f deg", ...
            t(k), roll(k), pitch(k));

        slider.Value = k;

        drawnow;
    end

    function playReplay()
        isPlaying = true;

        startIndex = round(slider.Value);

        for k = startIndex:1:length(t)
            if ~isPlaying
                break;
            end

            updateDisplay(k);
            pause(0.02);
        end
    end

    function pauseReplay()
        isPlaying = false;
    end

end