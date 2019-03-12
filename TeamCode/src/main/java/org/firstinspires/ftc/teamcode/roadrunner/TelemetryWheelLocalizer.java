package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.drive.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.jetbrains.annotations.NotNull;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 * Note: this could be optimized significantly with REV bulk reads
 */
@Config
public class TelemetryWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 1;
    public static double WHEEL_RADIUS = 2; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = 4; // in; offset of the lateral wheel
    public static double SIDE_OFFSET = 0; // in; offset from center axis of left and right wheels

    private ExpansionHubMotor leftEncoder, rightEncoder, frontEncoder;
    private RoadRunnerDriveBase driveBase;

    public TelemetryWheelLocalizer(HardwareMap hardwareMap, RoadRunnerDriveBase driveBase) {
        super(Arrays.asList(
                new Vector2d(SIDE_OFFSET, LATERAL_DISTANCE / 2), // left
                new Vector2d(SIDE_OFFSET, -LATERAL_DISTANCE / 2), // right
                new Vector2d(FORWARD_OFFSET, 0) // front
        ), Arrays.asList(0.0, 0.0, Math.PI / 2));

        //TODO: fill in these encoder configs lol
        leftEncoder = hardwareMap.get(ExpansionHubMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(ExpansionHubMotor.class, "rightEncoder");
        frontEncoder = hardwareMap.get(ExpansionHubMotor.class, "frontEncoder");
        this.driveBase = driveBase;
    }

    public static double encoderTicksToInches(int ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        RevBulkData bulkData = driveBase.getBulkData();
        if (bulkData == null) {
            return Arrays.asList(0.0, 0.0, 0.0);
        }
        return Arrays.asList(
                encoderTicksToInches(bulkData.getMotorCurrentPosition(leftEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(rightEncoder)),
                encoderTicksToInches(bulkData.getMotorCurrentPosition(frontEncoder))
        );
    }
}

