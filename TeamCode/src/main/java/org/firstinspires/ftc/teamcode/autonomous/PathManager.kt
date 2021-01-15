package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import org.firstinspires.ftc.teamcode.hardware.compbot.CompMecanumDrive
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetection.*
import kotlin.math.atan

class PathManager (val drive: CompMecanumDrive, val mech: MechanismController) {

    private val startPose = Pose2d(-48.0, 48.0, 0.0.toRadians)

    private val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 4.0))

    private val towerPose = Vector2d(72.0,36.0)

    private var trajectories = mutableListOf<MutableList<Trajectory>>(mutableListOf(), mutableListOf(), mutableListOf())


    public fun generatePaths() {
        // separate mechanism movements performed in follow methods

        // PATH A
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        trajectories[0][0] = drive.trajectoryBuilder(startPose, startPose.heading)
                .splineToSplineHeading(Pose2d(8.0, 50.0, 140.0.toRadians), 320.0.toRadians)
                .addDisplacementMarker(MarkerCallback{ mech.dropGoal() })
                .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 200.0.toRadians)
                .build()

        // shoot rings

        // travel to second wobble goal
        trajectories[0][1] = drive.trajectoryBuilder(trajectories[0][0].end(), true)
                .splineToSplineHeading(Pose2d(-48.0, 35.0, 350.0.toRadians), 180.0.toRadians)
                .build()

        // pick up second wobble goal

        // travel to drop zone, drop wobble goal between movements, park
        trajectories[0][2] = drive.trajectoryBuilder(trajectories[0][1].end(), trajectories[0][1].end().heading)
                .splineToSplineHeading(Pose2d(8.0, 50.0, 140.0.toRadians), 330.0.toRadians)
                .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 270.0.toRadians)
                .build()


        // PATH B
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        trajectories[1][0] = drive.trajectoryBuilder(startPose, startPose.heading)
                .splineToSplineHeading(Pose2d(-30.0, 52.0, 0.0.toRadians), 30.0.toRadians)
                .splineToSplineHeading(Pose2d(18.0, 38.0, 90.0.toRadians), 270.0.toRadians)
                .addDisplacementMarker(MarkerCallback{ mech.dropGoal() })
                .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.toRadians)
                .build()

        // shoot rings

        // travel to ring
        trajectories[1][1] = drive.trajectoryBuilder(trajectories[1][0].end(), trajectories[1][0].end().heading)
                .lineToLinearHeading(Pose2d(-24.0, 36.0, 320.0.toRadians))
                .build()

        // turn to tower

        // shoot ring

        // travel to second wobble goal
        trajectories[1][2] = drive.trajectoryBuilder(trajectories[1][1].end(), true)
                .splineToSplineHeading(Pose2d(-48.0, 35.0, 350.0.toRadians), 200.0.toRadians)
                .build()

        // pick up second wobble goal

        // travel to drop zone, drop wobble goal between movements, park
        trajectories[1][3] = drive.trajectoryBuilder(trajectories[1][2].end(), trajectories[1][2].end().heading)
                .splineToSplineHeading(Pose2d(18.0, 38.0, 90.0.toRadians), 270.0.toRadians)
                .addDisplacementMarker(MarkerCallback{ mech.dropGoal() })
                .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 240.0.toRadians)
                .build()


        // PATH C
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        trajectories[1][0] = drive.trajectoryBuilder(startPose, startPose.heading)
                .splineToSplineHeading(Pose2d(-27.0, 52.0, 0.0.toRadians), 0.0.toRadians)
                .splineToSplineHeading(Pose2d(42.0, 58.0, 90.0.toRadians), 350.0.toRadians)
                .addDisplacementMarker(MarkerCallback{ mech.dropGoal() })
                .splineToSplineHeading(Pose2d(-7.0, 28.0, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.toRadians)
                .build()

        // shoot rings

        // travel to ring
        trajectories[1][1] = drive.trajectoryBuilder(trajectories[1][0].end(), trajectories[1][0].end().heading)
                .lineToLinearHeading(Pose2d(-24.0, 36.0, 320.0.toRadians))
                .build()

        // turn to tower

        // shoot ring

        // travel to second wobble goal
        trajectories[1][2] = drive.trajectoryBuilder(trajectories[1][1].end(), true)
                .splineToSplineHeading(Pose2d(-48.0, 35.0, 350.0.toRadians), 200.0.toRadians)
                .build()

        // pick up second wobble goal

        // travel to drop zone, drop wobble goal between movements, park
        trajectories[1][3] = drive.trajectoryBuilder(trajectories[1][2].end(), trajectories[1][2].end().heading)
                .splineToSplineHeading(Pose2d(42.0, 58.0, 90.0.toRadians), 350.0.toRadians)
                .addDisplacementMarker(MarkerCallback{ mech.dropGoal() })
                .splineToSplineHeading(Pose2d(10.0, 28.0, 0.0.toRadians), 240.0.toRadians)
                .build()
    }

    public fun followPath(stackSize: StackSize) {
        mech.startIntake()
        when (stackSize) {
            StackSize.NONE -> followPathA()
            StackSize.ONE -> followPathB()
            StackSize.FOUR -> followPathC()
        }
        mech.stopIntake()
    }

    private fun followPathA() {
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(trajectories[0][0]);

        var pos = Vector2d(trajectories[0][0].end())

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
        drive.followTrajectory(trajectories[0][1])

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(trajectories[0][2])
    }

    private fun followPathB() {
        // start intake
        mech.startIntake()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(trajectories[1][0]);

        var pos = Vector2d(trajectories[1][0].end())

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

        // travel to ring
        drive.followTrajectory(trajectories[1][1])

        // turn to tower
        drive.turn(towerAngle(Vector2d(trajectories[1][1].end())))

        // shoot ring
        mech.shootRing()

        // travel to second wobble goal
        drive.followTrajectory(trajectories[1][2])

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(trajectories[1][3])

        // stop intake
        mech.stopIntake()
    }

    private fun followPathC() {
        // start intake
        mech.startIntake()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(trajectories[2][0]);

        var pos = Vector2d(trajectories[2][0].end())

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

        // travel to ring
        drive.followTrajectory(trajectories[2][1])

        // turn to tower
        drive.turn(towerAngle(Vector2d(trajectories[2][1].end())))

        // shoot rings
        mech.shootRing(true)
        mech.shootRing(true)
        mech.shootRing()

        // travel to second wobble goal
        drive.followTrajectory(trajectories[2][2])

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(trajectories[2][3])

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