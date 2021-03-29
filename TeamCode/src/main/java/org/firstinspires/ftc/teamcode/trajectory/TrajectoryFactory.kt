package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.Singleton.Color.BLUE
import org.firstinspires.ftc.teamcode.Singleton.color
import org.firstinspires.ftc.teamcode.Singleton.drive
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectories.a
import org.firstinspires.ftc.teamcode.util.trajectories.flip
import org.firstinspires.ftc.teamcode.util.trajectories.y
import kotlin.math.atan

object TrajectoryFactory {
    val startPose = Pose2d(-48.0, 48.0.y, 0.0.toRadians)

    val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0.y), Vector2d(72.0, 4.0.y))

    val towerPose = Vector2d(72.0, 36.0.y)

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    var startToLowToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 200.0.a.toRadians)
            .build()
    var startToMidToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(18.0, 28.0.y, 270.0.toRadians), 270.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .build()
    var startToHighToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 0.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .build()

    // travel to ring(s)
    var shootPowershotToRing: Trajectory = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .lineToLinearHeading(Pose2d(-24.0, 36.0.y, 140.0.a.toRadians))
            .build()

    // travel to second wobble goal
    var shootPowershotToWobble = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 43.0.y, 190.0.flip.a.toRadians), 170.0.a.toRadians)
            .build()
    var ringToWobble = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 43.0.y, 190.0.flip.toRadians), 200.0.a.toRadians)
            .build()

    var wobbleToLowToPark =
            if(color == BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
            {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.a.toRadians), 270.0.a.toRadians)
                    .build()
    var wobbleToMidToPark =
            if(color == BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
            {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(14.0, 38.0.y, 270.0.flip.a.toRadians), 220.0.a.toRadians)
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 270.0.flip.a.toRadians), 270.0.a.toRadians)
                    .build()
    var wobbleToHighToPark =
            if(color == BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
            {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 350.0.a.toRadians)
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.toRadians), 160.0.a.toRadians)
                    .build()

    fun towerAngle(x: Double, y: Double): Double {
        return (atan(towerPose.y - y / towerPose.x - x) - 90.0.toRadians).a
    }

    fun powerShotAngle(position: Vector2d, num: Int): Double {
        return Math.toRadians(Math.toDegrees(atan(powerShotPose[num].y - position.y / powerShotPose[num].x - position.x) + 90.0.toRadians).a)
    }
}