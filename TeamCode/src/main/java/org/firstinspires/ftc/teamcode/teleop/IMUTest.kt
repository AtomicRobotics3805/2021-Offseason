package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Constants
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.util.commands.CommandScheduler
import org.firstinspires.ftc.teamcode.util.toDegrees
import kotlin.math.abs
import kotlin.math.round

@TeleOp(name="IMU Test")
class IMUTest : LinearOpMode() {
    val imuHeading: Double
        get() {
            var heading = MecanumDrive.rawExternalHeading.toDegrees
            if (heading < 0) heading += 360
            return heading
        }

    override fun runOpMode() {

        Constants.opMode = this

        MecanumDrive.initialize()
        Intake.initialize()
        Shooter.initialize()
        Wobble.initialize()
        Controls.registerGamepads()

        var lastAngleReachedWheels = 0.0
        var lastAngleReachedIMU = 0.0
        var delay = 0.0
        val timer = ElapsedTime()

        waitForStart()

        CommandScheduler.commands += MecanumDrive.driverControlled(gamepad1)


        while (opModeIsActive()) {
            CommandScheduler.run()

            if (abs(lastAngleReachedWheels - MecanumDrive.poseEstimate.heading.toDegrees) >= 90) {
                lastAngleReachedWheels = round(MecanumDrive.poseEstimate.heading.toDegrees / 90.0) * 90.0
                timer.reset()
            }
            if (abs(lastAngleReachedIMU - imuHeading) >= 90) {
                lastAngleReachedIMU = round(imuHeading / 90.0) * 90.0
                if (lastAngleReachedIMU == lastAngleReachedWheels)
                    delay = timer.seconds()
            }

            MecanumDrive.telemetry.addData("Gamepad Left Stick X", gamepad1.left_stick_x)
            MecanumDrive.telemetry.addData("Gamepad Left Stick Y", gamepad1.left_stick_y)
            MecanumDrive.telemetry.addData("IMU Angle", imuHeading)
            MecanumDrive.telemetry.addData("Dead Wheel Angle", MecanumDrive.poseEstimate.heading.toDegrees)
            MecanumDrive.telemetry.addData("Delay", delay)
            MecanumDrive.telemetry.update()
        }
    }
}