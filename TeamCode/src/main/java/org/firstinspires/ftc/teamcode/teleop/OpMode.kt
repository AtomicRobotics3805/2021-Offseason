package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.tuning.SampleMecanumDrive

@TeleOp(name = "Competition OpMode")
class OpMode : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val drive = SampleMecanumDrive(hardwareMap)
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        waitForStart()
        while (!isStopRequested) {
            drive.setWeightedDrivePower(
                    Pose2d(
                            (-gamepad1.left_stick_y).toDouble(),
                            (-gamepad1.left_stick_x).toDouble(),
                            (-gamepad1.right_stick_x).toDouble()
                    )
            )
            drive.update()
            val poseEstimate = drive.poseEstimate
            telemetry.addData("x", poseEstimate.x)
            telemetry.addData("y", poseEstimate.y)
            telemetry.addData("heading", poseEstimate.heading)
            telemetry.update()
        }
    }
}