package org.firstinspires.ftc.teamcode.tuning.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.BaseDriveConstants;
import org.firstinspires.ftc.teamcode.tuning.DriveConstants;
import org.firstinspires.ftc.teamcode.tuning.SampleMecanumDrive;

@TeleOp
public class MotorTest extends LinearOpMode {
    private final BaseDriveConstants constants = DriveConstants.INSTANCE;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, constants, this);

        waitForStart();

        while(opModeIsActive()) {
            drive.setMotorPowers(0.5, 0.0, 0.0, 0.0);
            sleep(1000);
            telemetry.addData("Positions: ", drive.getWheelPositions().toString());
            telemetry.update();
            drive.setMotorPowers(0.0, 0.0, 0.0, 0.5);
            telemetry.update();
            sleep(1000);
            telemetry.addData("Positions: ", drive.getWheelPositions().toString());
            telemetry.update();
            drive.setMotorPowers(0.0, 0.5, 0.0, 0.0);
            sleep(1000);
            telemetry.addData("Positions: ", drive.getWheelPositions().toString());
            telemetry.update();
            drive.setMotorPowers(0.0, 0.0, 0.5, 0.0);
            sleep(1000);
            telemetry.addData("Positions: ", drive.getWheelPositions().toString());
            telemetry.update();
        }
    }
}
