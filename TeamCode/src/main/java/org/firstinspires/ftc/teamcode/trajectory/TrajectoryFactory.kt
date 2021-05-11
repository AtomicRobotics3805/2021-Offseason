package org.firstinspires.ftc.teamcode.trajectory

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import org.firstinspires.ftc.teamcode.Constants.Color.BLUE
import org.firstinspires.ftc.teamcode.Constants.color
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.util.trajectories.a
import org.firstinspires.ftc.teamcode.util.trajectories.flip
import org.firstinspires.ftc.teamcode.util.trajectories.y
import kotlin.math.atan

@Suppress("unused")
object TrajectoryFactory {
    private val startPose = Pose2d(-48.0, 48.0.y, 0.0.toRadians)

    private val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0.y), Vector2d(72.0, 4.0.y))

    private val towerPose = Vector2d(72.0, 36.0.y)

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    val startToLowToShootPowershot = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 200.0.a.toRadians)
            .build()
    val startToMidToShootPowershot = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(18.0, 28.0.y, 270.0.toRadians), 270.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .build()
    val startToHighToShootPowershot = MecanumDrive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 0.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .build()

    // travel to ring(s)
    val shootPowershotToRing = MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)
            .lineToLinearHeading(Pose2d(-24.0, 36.0.y, 140.0.a.toRadians))
            .build()

    // travel to second wobble goal
    val shootPowershotToWobble = MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 43.0.y, 190.0.flip.a.toRadians), 170.0.a.toRadians)
            .build()
    val ringToWobble = MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 43.0.y, 190.0.flip.toRadians), 200.0.a.toRadians)
            .build()

    val wobbleToLowToPark =
            if(color == BLUE)
                MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, true)
            else
            {MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.a.toRadians), 270.0.a.toRadians)
                    .build()
    val wobbleToMidToPark =
            if(color == BLUE)
                MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, true)
            else
            {MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(14.0, 38.0.y, 270.0.flip.a.toRadians), 220.0.a.toRadians)
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 270.0.flip.a.toRadians), 270.0.a.toRadians)
                    .build()
    val wobbleToHighToPark =
            if(color == BLUE)
                MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, true)
            else
            {MecanumDrive.trajectoryBuilder(MecanumDrive.poseEstimate, MecanumDrive.poseEstimate.heading)}
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