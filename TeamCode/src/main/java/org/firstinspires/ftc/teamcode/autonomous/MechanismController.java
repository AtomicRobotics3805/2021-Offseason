package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp;

public class MechanismController {

    private MecanumDriveComp drive;

    public static double INTAKE_POWER = 1.0;
    public static double SHOOTER_POWER = 0.7;

    private boolean servosExtended;
    private boolean intakeOn;
    private boolean shooterOn;
    private boolean wobbleGoalUp;

    public MechanismController(BaseMecanumDrive drive) {
        this.drive = (MecanumDriveComp) drive;
    }

    public void switchWobbleGoal() {
        if(wobbleGoalUp) dropGoal();
        else grabGoal();
    }

    public void grabGoal() {
        drive.wobbleArm.setTargetPosition(-1050);
        drive.wobbleHand.setPower(0.5);
        while(drive.wobbleArm.isBusy())
            drive.wobbleArm.setPower(0.5);
        drive.wobbleHand.setPower(1);
        ElapsedTime timer = new ElapsedTime();
        while(timer.seconds() < 1) {
            drive.wobbleArm.setTargetPosition(-300);
            drive.wobbleArm.setPower(0.1);
        }
    }

    public void alignGoal() {
        drive.wobbleArm.setTargetPosition(-1050);
        drive.wobbleArm.setPower(0.5);
        drive.wobbleHand.setPower(0);
    }

    public void raiseArm() {
        drive.wobbleArm.setTargetPosition(-800);
        drive.wobbleArm.setPower(0.1);
    }

    public void raiseArmStartingPosition() {
        drive.wobbleArm.setTargetPosition(0);
        drive.wobbleArm.setPower(0.1);
    }

    public void raiseArmHigh() {
        drive.wobbleArm.setTargetPosition(-300);
        drive.wobbleArm.setPower(0.1);
    }

    public void dropGoal() {
        if(drive.wobbleArm.getTargetPosition() != -800) {
            drive.wobbleArm.setTargetPosition(-800);
            drive.wobbleArm.setPower(0.3);
            while (drive.wobbleArm.isBusy()) {
                drive.wobbleArm.setPower(0.3);
            }
        }
        drive.wobbleHand.setPower(-1);
    }

    public void switchIntake() {
        if(intakeOn) stopIntake();
        else startIntake();
    }

    public void startIntake() {
        drive.intake.setPower(INTAKE_POWER);
        intakeOn = true;
    }

    public void stopIntake() {
        drive.intake.setPower(0.0);
        intakeOn = false;
    }

    public void switchShooter() {
        if(shooterOn) stopShooter();
        else startShooter();
    }

    public void startShooter() {
        drive.shooter.setPower(SHOOTER_POWER);
        shooterOn = true;
    }

    public void stopShooter() {
        drive.shooter.setPower(0.0);
        shooterOn = false;
    }

    public void shootRing() {
        shootRing(false);
    }

    public void shootRing(boolean pause) {
        drive.leftShooterTrigger.setPosition(0.6);
        drive.rightShooterTrigger.setPosition(0.575);
        servosExtended = true;
        if(pause) {
            ElapsedTime timer = new ElapsedTime();
            while(timer.seconds() < 0.5);
            retractShooterServos();
            while(timer.seconds() < 1.0);
        }
    }

    public void retractShooterServos() {
        drive.leftShooterTrigger.setPosition(0.925);
        drive.rightShooterTrigger.setPosition(0.25);
        servosExtended = false;
    }

    public boolean areServosExtended() {
        return servosExtended;
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public boolean isWobbleGoalUp() {
        return wobbleGoalUp;
    }
}
