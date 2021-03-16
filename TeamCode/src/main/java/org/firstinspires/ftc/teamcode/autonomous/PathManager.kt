package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.MarkerCallback
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.util.Vector2d
import org.firstinspires.ftc.teamcode.util.toRadians
import org.firstinspires.ftc.teamcode.autonomous.ObjectDetection.*
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive
import org.firstinspires.ftc.teamcode.hardware.compbot.MecanumDriveComp
import org.firstinspires.ftc.teamcode.teleop.CompTeleOp
import kotlin.math.atan

class PathManager(private var drive: MecanumDriveComp, private var mech: MechanismController, private val color: Color) {

    private val startPose = Pose2d(-48.0, 48.0.y, 0.0.toRadians)

    private val powerShotPose = listOf(Vector2d(72.0, 20.0), Vector2d(72.0, 12.0), Vector2d(72.0, 4.0))

    private val towerPose = Vector2d(72.0, 36.0)

    private val runtime = ElapsedTime()

    private lateinit var lastPose: Pose2d

    enum class Color {
        BLUE,
        RED
    }

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    private var startToLowToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 200.0.a.toRadians)
            .addDisplacementMarker{ mech.alignGoal() }
            .build()
    private var startToMidToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(18.0, 38.0.y, 270.0.toRadians), 270.0.a.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .addDisplacementMarker{ mech.alignGoal() }
            .build()
    private var startToHighToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 0.0.a.toRadians)
            .addDisplacementMarker{ mech.dropGoal() }
            .splineToSplineHeading(Pose2d(-7.0, 28.0.y, powerShotAngle(Vector2d(-7.0, 28.0), 0)), 160.0.a.toRadians)
            .addDisplacementMarker{ mech.alignGoal() }
            .build()

    // travel to ring(s)
    private var shootPowershotToRing: Trajectory = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .lineToLinearHeading(Pose2d(-24.0, 36.0.y, 140.0.a.toRadians))
            .addTemporalMarker(0.3) {
                mech.retractShooterServos()
            }
            .build()

    // travel to second wobble goal
    private var shootPowershotToWobble = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 35.0.y, 190.0.flip.a.toRadians), 170.0.a.toRadians)
            .addTemporalMarker(0.3) {
                mech.retractShooterServos()
            }
            .build()
    private var ringToWobble = drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)
            .splineToSplineHeading(Pose2d(-48.0, 35.0.y, 190.0.flip.toRadians), 200.0.a.toRadians)
            .addTemporalMarker(0.3) {
                mech.retractShooterServos()
            }
            .build()

    private var wobbleToLowToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
                {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.a.toRadians), 270.0.a.toRadians)
                    .addDisplacementMarker{ mech.alignGoal() }
                    .build()
    private var wobbleToMidToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
                {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(14.0, 38.0.y, 270.0.flip.a.toRadians), 220.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 270.0.flip.a.toRadians), 270.0.a.toRadians)
                    .addDisplacementMarker{ mech.alignGoal() }
                    .build()
    private var wobbleToHighToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(drive.poseEstimate, true)
            else
                {drive.trajectoryBuilder(drive.poseEstimate, drive.poseEstimate.heading)}
                    .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 350.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.toRadians), 160.0.a.toRadians)
                    .addDisplacementMarker{ mech.alignGoal() }
                    .build()

    fun followPath(stackSize: StackSize) {
        drive.poseEstimate = startPose
        when (stackSize) {
            StackSize.NONE -> followPathLow()
            StackSize.ONE -> followPathMid()
            StackSize.FOUR -> followPathHigh()
        }
        mech.stopIntake()
        CompTeleOp.startingPose = drive.poseEstimate
    }

    private fun followPathLow() {
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToLowToShootPowershot)
        //mech.startShooter()

        shootPowershot()

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
        //mech.startShooter()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToMidToShootPowershot)

        shootPowershot()

        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        drive.turn(towerAngle(Vector2d(drive.poseEstimate)))

        // shoot top goal
        //mech.shootRing()
        mech.stopShooter()

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
        //mech.startShooter()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToHighToShootPowershot)

        shootPowershot()

        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        drive.turn(towerAngle(Vector2d(drive.poseEstimate)))

        // shoot top goal
        //mech.shootRing(true)
        //mech.shootRing(true)
        //mech.shootRing()
        mech.stopShooter()

        // travel to second wobble goal
        drive.followTrajectory(ringToWobble)

        // pick up second wobble goal
        mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        drive.followTrajectory(wobbleToHighToPark)

        // stop intake
        mech.stopIntake()
    }

    private fun shootPowershot() {
        val pos = Vector2d(startToLowToShootPowershot.end())

        // shoot powershot 1
        //mech.shootRing()

        // turn to powershot 2
        drive.turnAsync(1.0)

        runtime.reset()
        while(drive.isBusy) {
            drive.update()
            if(runtime.seconds() > 0.3) mech.retractShooterServos()
        }

        // shoot powershot 2
        //mech.shootRing()

        // turn to powershot 3
        drive.turnAsync(powerShotAngle(pos, 2))

        runtime.reset()
        while(drive.isBusy) {
            drive.update()
            if(runtime.seconds() > 0.3) mech.retractShooterServos()
        }

        // shoot powershot 3
        //mech.shootRing()

        mech.stopShooter()
    }

    private fun towerAngle(position: Vector2d): Double {
        return atan(towerPose.y - position.y / towerPose.x - position.x) + 90.0.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return atan(powerShotPose[num].y - position.y / powerShotPose[num].x - position.x) + 90.0.toRadians
    }

    val Double.a get () = (if (color == Color.BLUE) this else 360 - this)

    val Double.flip get () = (if (color == Color.BLUE) this else {
        if(this - 180 < 0) this + 180
        this - 180
    })

    val Double.y get () = (if (color == Color.BLUE) this else this * -1)
}