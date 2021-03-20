package org.firstinspires.ftc.teamcode.hardware.compbot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.Encoder;

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
 */
@Config
public class LocalizerComp extends ThreeTrackingWheelLocalizer {
    public static double ticksPerRev = 2400;
    public static double wheelRadius = 1.5; // in
    public static double gearRatio = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 17.3; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -3.6; // in; offset of the lateral wheel

    public static boolean LEFT_REVERSED = true;
    public static boolean RIGHT_REVERSED = true;
    public static boolean FRONT_REVERSED = true;

    public static double X_MULTIPLIER = 1.04;
    public static double Y_MULTIPLIER = 1.04;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public LocalizerComp(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LF"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "RF"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "LB"));

        // FINISHED: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        if(RIGHT_REVERSED) rightEncoder.setDirection(Encoder.Direction.REVERSE);
        if(LEFT_REVERSED) leftEncoder.setDirection(Encoder.Direction.REVERSE);
        if(FRONT_REVERSED) frontEncoder.setDirection(Encoder.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return wheelRadius * 2 * Math.PI * gearRatio * ticks / ticksPerRev;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getCurrentPosition()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getCurrentPosition()) * Y_MULTIPLIER
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // FINISHED: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(rightEncoder.getRawVelocity()) * X_MULTIPLIER,
                encoderTicksToInches(frontEncoder.getRawVelocity()) * Y_MULTIPLIER
        );
    }
}
