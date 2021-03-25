package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetection.*
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.teleop.OneDriverTeleOp
import org.firstinspires.ftc.teamcode.util.toDegrees
import kotlin.math.atan

class PathManager(private var drive: MecanumDriveComp, private var mech: MechanismController, private val color: Color, private val opMode: OpMode? = null) {

    private val startPose = Pose2d(-63.0, 48.0.y, 0.0.toRadians)

    private val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 1.0))

    private val towerPose = Vector2d(72.0, 36.0)

    private val runtime = ElapsedTime()

    private lateinit var lastPose: Pose2d

    // number of degrees the shooter shoots offcenter
    private val OFFSET = 14.3

    enum class Color {
        BLUE,
        RED
    }

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings

    private val startToLow = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineTo(Vector2d(8.0, 50.0.y), 320.0.a.flip.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .build()
    private val startToMid = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineTo(Vector2d(14.0, 36.0), 270.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .build()
    private val startToHigh = drive.trajectoryBuilder(startPose, startPose.heading - 20.0.toRadians)
            .splineToLinearHeading(Pose2d(42.0, 44.0.y, 90.0.toRadians), 270.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .build()

    private val lowToPowershot = drive.trajectoryBuilder(startToLow.end(), startToLow.end().heading - 90.0.toRadians)
            .splineToLinearHeading(Pose2d(-7.0, 26.0, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 270.0.toRadians)
            .build()
    private val midToPowershot = drive.trajectoryBuilder(startToMid.end(), startToMid.end().heading - 90.0.toRadians)
            .splineToLinearHeading(Pose2d(-7.0, 26.0, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 270.0.toRadians)
            .build()
    private val highToPowershot = drive.trajectoryBuilder(startToHigh.end(), startToHigh.end().heading - 90.0.toRadians)
            .splineToLinearHeading(Pose2d(-7.0, 26.0, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 270.0.toRadians)
            .build()

    // travel to second wobble goal
    private val powershotToWobble = drive.trajectoryBuilder(Pose2d(midToPowershot.end().vec(), powerShotAngle(midToPowershot.end().vec(), 2)), powerShotAngle(midToPowershot.end().vec(), 2))
            .splineTo(Vector2d(-20.0, 34.0), 135.0.toRadians)
            .splineTo(Vector2d(-50.0, 40.0), 180.0.toRadians)
            .build()

    private val wobbleToShootTower =
            drive.trajectoryBuilder(powershotToWobble.end(), true)
                    .splineTo(Vector2d(-10.0, 32.0), (Vector2d(-10.0, 32.0) - towerPose angleBetween Vector2d(1.0, 0.0)) + 180.0.toRadians + OFFSET.toRadians) // -10.0 inches because you are shooting
                    .build()

    private val wobbleToLowToPark =
            drive.trajectoryBuilder(powershotToWobble.end(), true)
                    .splineToSplineHeading(Pose2d(8.0, 50.0.y, 0.0.toRadians), 320.0.a.flip.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineTo(Vector2d(10.0, 28.0.y), 180.0.a.toRadians)
                    .build()
    private val towerToMidToPark =
            drive.trajectoryBuilder(wobbleToShootTower.end(), true)
                    .splineTo(Vector2d(14.0, 36.0), 90.0.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .build()
    private val towerToHighToPark =
            drive.trajectoryBuilder(wobbleToShootTower.end(), true)
                    .splineTo(Vector2d(42.0, 58.0.y), 270.0.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineTo(Vector2d(10.0, 28.0.y), 180.0.toRadians)
                    .build()

    fun followPath(stackSize: StackSize) {
        drive.poseEstimate = startPose
        mech.raiseArmHigh()
        when (stackSize) {
            StackSize.NONE -> followPathLow()
            StackSize.ONE -> followPathMid()
            StackSize.FOUR -> followPathHigh()
        }
        mech.stopIntake()
        OneDriverTeleOp.startingPose = drive.poseEstimate
        mech.raiseArmStartingPosition()
    }

    private fun followPathLow() {
        mech.startIntake()
        mech.startShooter()
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToLow)

        drive.followTrajectory(lowToPowershot)

        shootPowershot()
        mech.stopIntake()

        // travel to second wobble goal
        drive.followTrajectory(powershotToWobble)
        mech.stopShooter()

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(wobbleToLowToPark)
    }

    private fun followPathMid() {
        // start shooter
        mech.startIntake()
        mech.startShooter()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToMid)

        drive.followTrajectory(midToPowershot)

        shootPowershot()

        drive.followTrajectory(powershotToWobble)

        // pick up second wobble goal
        mech.grabGoal()

        // travel to tower
        drive.followTrajectory(wobbleToShootTower)

        // shoot top goal
        mech.shootRing(true)
        mech.shootRing(true)
        mech.stopShooter()

        // stop intake
        mech.stopIntake()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(towerToMidToPark)
    }

    private fun followPathHigh() {
        // start intake
        mech.startIntake()
        // start shooter
        mech.startShooter()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToHigh)

        drive.followTrajectory(highToPowershot)

        shootPowershot()

        drive.followTrajectory(powershotToWobble)

        mech.grabGoal()

        // travel to tower
        drive.followTrajectory(wobbleToShootTower)

        // shoot top goal
        mech.shootRing(true)
        mech.shootRing(true)
        mech.shootRing(true)
        mech.stopShooter()

        // stop intake
        mech.stopIntake()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(towerToHighToPark)

        // stop intake
        mech.stopIntake()
    }

    private fun shootPowershot() {
        val pos = Vector2d(drive.poseEstimate)

        // shoot powershot 1
        mech.shootRing()

        // turn to powershot 2
        drive.turnAsync(powerShotAngle(pos, 1) - drive.poseEstimate.heading)
        opMode?.telemetry?.addData("Turn Amount", (powerShotAngle(pos, 1) - drive.poseEstimate.heading).toDegrees)
        opMode?.telemetry?.update()

        runtime.reset()
        do {
            drive.update()
            if(runtime.seconds() > 0.5) mech.retractShooterServos()
            if(!drive.isBusy && runtime.seconds() > 1.0) {
                mech.shootRing()
            }
        } while(runtime.seconds() <= 1.0 || drive.isBusy)

        // turn to powershot 3
        drive.turnAsync(powerShotAngle(pos, 2) - drive.poseEstimate.heading)
        opMode?.telemetry?.addData("Turn Amount", (powerShotAngle(pos, 2) - drive.poseEstimate.heading).toDegrees)
        opMode?.telemetry?.update()

        runtime.reset()
        do {
            drive.update()
            if(runtime.seconds() > 0.5) mech.retractShooterServos()
            if(!drive.isBusy && runtime.seconds() > 1.0) {
                mech.shootRing(true)
                opMode?.telemetry?.addData("Shot Ring", 3)
                opMode?.telemetry?.update()
            }
        } while(runtime.seconds() <= 1.0 || drive.isBusy)

        runtime.reset()
    }

    private fun towerAngle(position: Vector2d): Double {
        return (position - towerPose angleBetween Vector2d(1.0, 0.0)) + 5.0.toRadians + OFFSET.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return (position - powerShotPose[num] angleBetween Vector2d(1.0, 0.0)) + OFFSET.toRadians
    }

    val Double.a get () = (if (color == Color.BLUE) this else 360 - this)

    val Double.flip get () = (if (color == Color.BLUE) this else {
        if(this - 180 < 0) this + 180
        this - 180
    })

    val Double.y get () = (if (color == Color.BLUE) this else this * -1)
}