package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetection.*
import kotlin.math.atan

class PathManager (private var drive: CompMecanumDrive, var mech: MechanismController) {

    private val startPose = Pose2d(-48.0, 48.0, 0.0.toRadians)

    private val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 4.0))

    private val towerPose = Vector2d(72.0,36.0)

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    private var startToLowToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(8.0, 50.0, 140.0.toRadians), 320.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 200.0.toRadians)
            .build()
    private var startToMidToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(-30.0, 52.0, 0.0.toRadians), 30.0.toRadians)
            .splineToSplineHeading(Pose2d(18.0, 38.0, 90.0.toRadians), 270.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.toRadians)
            .build()
    private var startToHighToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(-27.0, 52.0, 0.0.toRadians), 0.0.toRadians)
            .splineToSplineHeading(Pose2d(42.0, 58.0, 90.0.toRadians), 350.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.toRadians)
            .build()

    // travel to ring(s)
    private var shootPowershotToRing: Trajectory = drive.trajectoryBuilder(startToMidToShootPowershot.end(), startToMidToShootPowershot.end().heading)
            .lineToLinearHeading(Pose2d(-24.0, 36.0, 320.0.toRadians))
            .build()

    // travel to second wobble goal
    private var shootPowershotToWobble = drive.trajectoryBuilder(startToLowToShootPowershot.end(), true)
            .splineToSplineHeading(Pose2d(-48.0, 35.0, 350.0.toRadians), 180.0.toRadians)
            .build()
    private var ringToWobble = drive.trajectoryBuilder(shootPowershotToRing.end(), true)
            .splineToSplineHeading(Pose2d(-48.0, 35.0, 350.0.toRadians), 200.0.toRadians)
            .build()

    // travel to drop zone, drop wobble goal between movements, park
    private var wobbleToLowToPark = drive.trajectoryBuilder(shootPowershotToWobble.end(), shootPowershotToWobble.end().heading)
            .splineToSplineHeading(Pose2d(8.0, 50.0, 140.0.toRadians), 330.0.toRadians)
            .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 270.0.toRadians)
            .build()
    private var wobbleToMidToPark = drive.trajectoryBuilder(ringToWobble.end(), ringToWobble.end().heading)
            .splineToSplineHeading(Pose2d(18.0, 38.0, 90.0.toRadians), 270.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 240.0.toRadians)
            .build()
    private var wobbleToHighToPark = drive.trajectoryBuilder(ringToWobble.end(), ringToWobble.end().heading)
            .splineToSplineHeading(Pose2d(42.0, 58.0, 90.0.toRadians), 350.0.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 240.0.toRadians)
            .build()

    fun followPath(stackSize: StackSize) {
        mech.startIntake()
        when (stackSize) {
            StackSize.NONE -> followPathLow()
            StackSize.ONE -> followPathMid()
            StackSize.FOUR -> followPathHigh()
        }
        mech.stopIntake()
    }

    private fun followPathLow() {
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToLowToShootPowershot)

        var pos = Vector2d(startToLowToShootPowershot.end())

        // shoot powershot 1
        mech.shootRing()

        // turn to powershot 2
        drive.turn(powerShotAngle(pos, 2))

        // shoot powershot 2
        mech.shootRing()

        // turn to powershot 3
        drive.turn(powerShotAngle(pos, 3))

        // shoot powershot 3
        mech.shootRing()

        // travel to second wobble goal
        drive.followTrajectory(shootPowershotToWobble)

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(wobbleToLowToPark)
    }

    private fun followPathMid() {
        // start intake
        mech.startIntake()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToMidToShootPowershot)

        // shoot powershot 1
        mech.shootRing()

        // turn to powershot 2
        drive.turn(powerShotAngle(Vector2d(startToMidToShootPowershot.end()), 2))

        // shoot powershot 2
        mech.shootRing()

        // turn to powershot 3
        drive.turn(powerShotAngle(Vector2d(startToMidToShootPowershot.end()), 3))

        // shoot powershot 3
        mech.shootRing()

        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        drive.turn(towerAngle(Vector2d(shootPowershotToRing.end())))

        // shoot top goal
        mech.shootRing()

        // travel to second wobble goal
        drive.followTrajectory(ringToWobble)

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(wobbleToMidToPark)

        // stop intake
        mech.stopIntake()
    }

    private fun followPathHigh() {
        // start intake
        mech.startIntake()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToHighToShootPowershot)

        // shoot powershot 1
        mech.shootRing()

        // turn to powershot 2
        drive.turn(powerShotAngle(Vector2d(startToHighToShootPowershot.end()), 2))

        // shoot powershot 2
        mech.shootRing()

        // turn to powershot 3
        drive.turn(powerShotAngle(Vector2d(startToHighToShootPowershot.end()), 3))

        // shoot powershot 3
        mech.shootRing()

        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        drive.turn(towerAngle(Vector2d(shootPowershotToRing.end())))

        // shoot top goal
        mech.shootRing(true)
        mech.shootRing(true)
        mech.shootRing()

        // travel to second wobble goal
        drive.followTrajectory(ringToWobble)

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(wobbleToHighToPark)

        // stop intake
        mech.stopIntake()
    }

    private fun towerAngle(position: Vector2d): Double {
        return atan(towerPose.y - position.y / towerPose.x - position.x) - 90.0.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return atan(powerShotPose[num].y - position.y / powerShotPose[num].x - position.x) - 90.0.toRadians
    }
}