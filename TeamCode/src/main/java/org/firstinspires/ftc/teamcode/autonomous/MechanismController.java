package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp;

public class MechanismController {

    private MecanumDriveComp drive;

    public static double COUNTS_PER_REV = 25.9;
    public static double MAX_RPM = 1500;
    public static double INTAKE_POWER = 1.0;
    public static double SHOOTER_POWER = 0.66;
    public static double AUTO_SHOOTER_POWER = 0.66;
    public static double SHOOTER_VELOCITY = SHOOTER_POWER * COUNTS_PER_REV * MAX_RPM / 60.0;
    public static double AUTO_SHOOTER_VELOCITY = AUTO_SHOOTER_POWER * COUNTS_PER_REV * MAX_RPM / 60.0;
    public static double MANUAL_ARM_POWER = 0.3;
    public static double MANUAL_CLAW_POWER = 1.0;

    private boolean servosExtended;
    private boolean intakeOn;
    private boolean shooterOn;
    private boolean wobbleGoalUp;

    private boolean teleOp = true;

    public int targetPos = 0;

    public MechanismController(BaseMecanumDrive drive) {
        this.drive = (MecanumDriveComp) drive;
    }

    public MechanismController(BaseMecanumDrive drive, boolean teleOp) {
        this.drive = (MecanumDriveComp) drive;
        this.teleOp = teleOp;
    }

    public void switchWobbleGoal() {
        if(wobbleGoalUp) dropGoal();
        else grabGoal();
    }

    public void grabGoal() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(drive.wobbleArm.getTargetPosition() != -1100) {
            drive.wobbleArm.setTargetPosition(-1100);
            drive.wobbleHand.setPower(-1);
            while (drive.wobbleArm.isBusy())
                drive.wobbleArm.setPower(0.5);
        }
        drive.wobbleHand.setPower(1);
        ElapsedTime timer = new ElapsedTime();
        while (timer.seconds() < 1) {
            drive.wobbleArm.setTargetPosition(-300);
            drive.wobbleArm.setPower(0.1);
        }
    }

    public void alignGoal() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.wobbleArm.setTargetPosition(-1100);
        drive.wobbleArm.setPower(0.3);
        drive.wobbleHand.setPower(0);
    }

    public void raiseArm() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.wobbleArm.setTargetPosition(-1000);
        drive.wobbleArm.setPower(0.1);
    }

    public void raiseArmStartingPosition() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.wobbleArm.setTargetPosition(0);
        drive.wobbleArm.setPower(0.1);
    }

    public void raiseArmHigh() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.wobbleArm.setTargetPosition(-300);
        drive.wobbleArm.setPower(0.1);
    }

    public void dropGoal() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(drive.wobbleArm.getTargetPosition() != -1000) {
            drive.wobbleArm.setTargetPosition(-1000);
            drive.wobbleArm.setPower(0.2);
            while (drive.wobbleArm.isBusy()) {
                drive.wobbleArm.setPower(0.2);
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

    public void reverseIntake() {
        drive.intake.setPower(-INTAKE_POWER);
        intakeOn = true;
    }

    public void switchShooter() {
        if(shooterOn) stopShooter();
        else startShooter();
    }

    public void startShooter() {
        //drive.shooter.setVelocity(teleOp ? SHOOTER_VELOCITY : AUTO_SHOOTER_VELOCITY);
        drive.shooter.setPower(teleOp ? SHOOTER_POWER : AUTO_SHOOTER_POWER);
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

    public void lowerArmManually() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.wobbleArm.setPower(-MANUAL_ARM_POWER);
    }

    public void raiseArmManually() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.wobbleArm.setPower(MANUAL_ARM_POWER);
    }

    public void stopArmManually() {
        drive.wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.wobbleArm.setPower(MANUAL_ARM_POWER);
        drive.wobbleArm.setTargetPosition(targetPos);
    }

    public void closeClawManually() {
        drive.wobbleHand.setPower(MANUAL_CLAW_POWER);
    }

    public void openClawManually() {
        drive.wobbleHand.setPower(-MANUAL_CLAW_POWER);
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
