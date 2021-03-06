package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp;

public class MechanismController {

    private MecanumDriveComp drive;

    public static double INTAKE_POWER = 1.0;
    public static double SHOOTER_POWER = 0.83;

    public MechanismController(BaseMecanumDrive drive) {
        this.drive = (MecanumDriveComp) drive;
    }

    public void grabGoal() {

    }

    public void dropGoal() {

    }

    public void startIntake() {
        drive.intake.setPower(INTAKE_POWER);
    }

    public void stopIntake() {
        drive.intake.setPower(0.0);
    }

    public void startShooter() {
        drive.shooter.setPower(SHOOTER_POWER);
    }

    public void stopShooter() {
        drive.shooter.setPower(0.0);
    }

    public void shootRing() {
        shootRing(false);
    }

    public void shootRing(boolean pause) {
        drive.leftShooterTrigger.setPosition(0.7);
        drive.rightShooterTrigger.setPosition(0.475);
        if(pause) {
            ElapsedTime runtime = new ElapsedTime();
            while(runtime.seconds() < 0.3);
        }
    }

    public void retractShooterServos() {
        drive.leftShooterTrigger.setPosition(0.925);
        drive.rightShooterTrigger.setPosition(0.25);
    }
}
