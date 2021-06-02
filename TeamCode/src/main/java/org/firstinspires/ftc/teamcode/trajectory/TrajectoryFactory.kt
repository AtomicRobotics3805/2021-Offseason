package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants.startPose
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectories.a
import org.firstinspires.ftc.teamcode.util.trajectories.flip
import org.firstinspires.ftc.teamcode.util.trajectories.y

@Suppress("unused", "MemberVisibilityCanBePrivate")
@Config
object TrajectoryFactory {
    @JvmField
    var powerShotPose = listOf(Vector2d(72.0, 18.0), Vector2d(72.0, 12.0), Vector2d(72.0, 0.0))

    @JvmField
    var towerPose = Vector2d(72.0, 30.0)
    
    @JvmField
    var OFFSET = 18.3

    val testTrajectory = MecanumDrive.trajectoryBuilder(Pose2d(-63.0, 48.0, 0.0), 0.0)
            //.splineTo(Vector2d(-53.0, 48.0), 0.0)
            .back(10.0)
            .build()
    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    val startToLow = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineTo(Vector2d(3.0, 52.0.y), 320.0.a.flip.toRadians)
            .build()
    val startToMid = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineTo(Vector2d(17.0, 40.0), 270.0.toRadians)
            .build()
    val startToHigh = MecanumDrive.trajectoryBuilder(startPose, startPose.heading + 20.0.toRadians)
            .splineToConstantHeading(Vector2d(58.0, 44.0), 0.0.toRadians)
            .build()

    val lowToPowershot = MecanumDrive.trajectoryBuilder(startToLow.trajectory.end(), startToLow.trajectory.end().heading - 90.0.toRadians)
            .splineToLinearHeading(Pose2d(-8.0, 26.0, towerAngle(Vector2d(-7.0, 26.0)) + 2.0.toRadians), 270.0.toRadians)
            .build()
    val midToPowershot = MecanumDrive.trajectoryBuilder(startToMid.trajectory.end(), startToMid.trajectory.end().heading - 90.0.toRadians)
            .splineToLinearHeading(Pose2d(-8.0, 26.0,  towerAngle(Vector2d(-7.0, 26.0)) + 4.0.toRadians), 270.0.toRadians)
            .build()
    val highToPowershot = MecanumDrive.trajectoryBuilder(startToHigh.trajectory.end(), startToHigh.trajectory.end().heading - 135.0.toRadians)
            .splineToLinearHeading(Pose2d(-8.0, 26.0, towerAngle(Vector2d(-7.0, 26.0)) + 3.0.toRadians), 90.0.toRadians)
            .build()

    val powershotToWobble = MecanumDrive.trajectoryBuilder(lowToPowershot.trajectory.end(), lowToPowershot.trajectory.end().heading)
            .splineTo(Vector2d(-20.0, 34.0), 135.0.toRadians)
            .splineTo(Vector2d(-50.0, 41.5), 180.0.toRadians)
            .build()
    val powershotToRingToWobble = MecanumDrive.trajectoryBuilder(midToPowershot.trajectory.end(), midToPowershot.trajectory.end().heading)
            .splineTo(Vector2d(-20.0, 34.0), 135.0.toRadians)
            .splineTo(Vector2d(-50.0, 41.5), 180.0.toRadians)
            .build()

    val powershotToWobbleHigh = MecanumDrive.trajectoryBuilder(highToPowershot.trajectory.end(), highToPowershot.trajectory.end().heading)
            .splineToLinearHeading(Pose2d(-32.0, 24.0, 90.0.toRadians), 180.0.toRadians)
            .build()

    val wobbleToShootTower =
            MecanumDrive.trajectoryBuilder(powershotToWobble.trajectory.end(), true)
                    .splineTo(Vector2d(-10.0, 32.0), towerAngle(Vector2d(-10.0, 32.0)) + 180.0.toRadians)
                    .build()

    val wobbleToLow =
            MecanumDrive.trajectoryBuilder(powershotToWobble.trajectory.end(), true)
                    .splineToLinearHeading(Pose2d(10.0, 46.0.y, 0.0.toRadians), 320.0.a.flip.toRadians)
                    .build()

    val lowToPark =
            MecanumDrive.trajectoryBuilder(wobbleToLow.trajectory.end(), wobbleToLow.trajectory.end().heading - 90.0.toRadians)
                    .splineTo(Vector2d(10.0, 28.0.y), 270.0.a.toRadians)
                    .build()
    val wobbleToHigh =
            MecanumDrive.trajectoryBuilder(powershotToWobbleHigh.trajectory.end(), powershotToWobbleHigh.trajectory.end().heading - 120.0.toRadians)
                    .splineToLinearHeading(Pose2d(58.0, 44.0, 0.0.toRadians), 60.0.toRadians)
                    .build()
    val highToPark =
            MecanumDrive.trajectoryBuilder(wobbleToHigh.trajectory.end(), true)
                    .splineTo(Vector2d(10.0, 44.0.y), 180.0.a.toRadians)
                    .build()

    val towerToMid =
            MecanumDrive.trajectoryBuilder(wobbleToShootTower.trajectory.end(), wobbleToShootTower.trajectory.end().heading - 200.0.toRadians)
                    .splineTo(Vector2d(18.0, 38.0), 70.0.toRadians)
                    .build()
    val midToPark =
            MecanumDrive.trajectoryBuilder(towerToMid.trajectory.end(), towerToMid.trajectory.end().heading)
                    .strafeRight(4.0)
                    .build()

    fun towerAngle(position: Vector2d): Double {
        return (position - towerPose angleBetween Vector2d(1.0, 0.0)) + 5.0.toRadians + OFFSET.toRadians
    }
    
    fun powerShotAngle(position: Vector2d, num: Int): Double {
        return (position - powerShotPose[num] angleBetween Vector2d(1.0, 0.0)) + OFFSET.toRadians
    }
}