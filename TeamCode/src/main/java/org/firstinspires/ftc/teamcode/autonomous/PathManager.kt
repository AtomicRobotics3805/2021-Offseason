package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
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

    enum class Color {
        BLUE,
        RED
    }

    // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
    private var startToLowToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .addDisplacementMarker(60.0) { mech.dropGoal() }
            .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 26.0.y, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 200.0.a.toRadians,
                    MinVelocityConstraint(listOf(
                            AngularVelocityConstraint(drive.constants.maxAngVel),
                            MecanumVelocityConstraint(15.0, drive.constants.trackWidth)))
                    , ProfileAccelerationConstraint(drive.constants.maxAccel))
            .addDisplacementMarker{ mech.raiseArm() }
            .build()
    private var startToMidToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .addDisplacementMarker(72.0) { mech.dropGoal() }
            .splineToSplineHeading(Pose2d(14.0, 44.0.y, 270.0.toRadians), 270.0.a.toRadians)
            .splineToSplineHeading(Pose2d(-7.0, 26.0.y, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 160.0.a.toRadians,
            MinVelocityConstraint(listOf(
                    AngularVelocityConstraint(drive.constants.maxAngVel),
                    MecanumVelocityConstraint(15.0, drive.constants.trackWidth)))
            , ProfileAccelerationConstraint(drive.constants.maxAccel))
            .addDisplacementMarker{ mech.raiseArm() }
            .build()
    private var startToHighToShootPowershot = drive.trajectoryBuilder(startPose, startPose.heading)
            .addDisplacementMarker(96.0) { mech.dropGoal() }
            .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 0.0.a.toRadians,
                    MinVelocityConstraint(listOf(
                            AngularVelocityConstraint(drive.constants.maxAngVel),
                            MecanumVelocityConstraint(15.0, drive.constants.trackWidth)))
                    , ProfileAccelerationConstraint(drive.constants.maxAccel))
            .splineToSplineHeading(Pose2d(-7.0, 26.0.y, powerShotAngle(Vector2d(-7.0, 26.0), 0)), 160.0.a.toRadians)
            .addDisplacementMarker{ mech.raiseArm() }
            .build()

    // travel to ring(s)
    private var shootPowershotToRing: Trajectory = drive.trajectoryBuilder(startToMidToShootPowershot.end(), startToMidToShootPowershot.end().heading)
            .lineToLinearHeading(Pose2d(-30.0, 40.0.y, 130.0.a.toRadians))
            .addTemporalMarker(0.5) {
                mech.retractShooterServos()
            }
            .build()

    // travel to second wobble goal
    private var shootPowershotToWobble = drive.trajectoryBuilder(startToMidToShootPowershot.end(), startToMidToShootPowershot.end().heading)
            .splineToSplineHeading(Pose2d(-53.0, 40.0.y, 190.0.flip.a.toRadians), 170.0.a.toRadians)
            .addTemporalMarker(0.5) {
                mech.retractShooterServos()
            }
            .build()
    private var ringToWobble = drive.trajectoryBuilder(shootPowershotToRing.end(), shootPowershotToRing.end().heading)
            .splineToSplineHeading(Pose2d(-53.0, 40.0.y, 190.0.flip.toRadians), 200.0.a.toRadians)
            .addTemporalMarker(0.5) {
                mech.retractShooterServos()
            }
            .build()

    private var wobbleToLowToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(shootPowershotToWobble.end(), true)
            else
                {drive.trajectoryBuilder(shootPowershotToWobble.end(), shootPowershotToWobble.end().heading)}
                    .splineToSplineHeading(Pose2d(8.0, 50.0.y, 320.0.a.flip.toRadians), 320.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.a.toRadians), 270.0.a.toRadians)
                    .addDisplacementMarker{ mech.raiseArmStartingPosition() }
                    .build()
    private var wobbleToMidToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(shootPowershotToWobble.end(), true)
            else
                {drive.trajectoryBuilder(shootPowershotToWobble.end(), shootPowershotToWobble.end().heading)}
                    .splineToSplineHeading(Pose2d(18.0, 34.0.y, 270.0.toRadians), 270.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 270.0.flip.a.toRadians), 270.0.a.toRadians)
                    .addDisplacementMarker{ mech.raiseArmStartingPosition() }
                    .build()
    private var wobbleToHighToPark =
            if(color == Color.BLUE)
                drive.trajectoryBuilder(shootPowershotToWobble.end(), true)
            else
                {drive.trajectoryBuilder(shootPowershotToWobble.end(), shootPowershotToWobble.end().heading)}
                    .splineToSplineHeading(Pose2d(42.0, 58.0.y, 270.0.toRadians), 350.0.a.toRadians)
                    .addDisplacementMarker{ mech.dropGoal() }
                    .splineToSplineHeading(Pose2d(10.0, 28.0.y, 180.0.toRadians), 160.0.a.toRadians)
                    .addDisplacementMarker{ mech.raiseArmStartingPosition() }
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
    }

    private fun followPathLow() {
        mech.startIntake()
        mech.startShooter()
        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToLowToShootPowershot)

        shootPowershot()
        mech.stopIntake()

        // travel to second wobble goal
        drive.followTrajectory(shootPowershotToWobble)
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
        drive.followTrajectory(startToMidToShootPowershot)

        shootPowershot()


        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        //drive.turn(towerAngle(Vector2d(drive.poseEstimate)) - drive.poseEstimate.heading)

        // stop intake
        //mech.stopIntake()

        // shoot top goal
        //mech.shootRing(true)
        //mech.shootRing(true)
        //mech.stopShooter()

        // travel to second wobble goal
        //drive.followTrajectory(ringToWobble)

        // pick up second wobble goal
        //mech.grabGoal()

        // travel to drop zone, drop wobble goal between movements, park
        //drive.followTrajectory(wobbleToMidToPark)
    }

    private fun followPathHigh() {
        // start intake
        mech.startIntake()
        // start shooter
        mech.startShooter()

        // travel to drop zone, drop wobble goal between movements, prepare to shoot rings
        drive.followTrajectory(startToHighToShootPowershot)

        shootPowershot()

        // travel to ring
        drive.followTrajectory(shootPowershotToRing)

        // turn to tower
        drive.turn(towerAngle(Vector2d(startToMidToShootPowershot.end())) - startToMidToShootPowershot.end().heading)

        // stop intake
        mech.stopIntake()

        // shoot top goal
        mech.shootRing(true)
        mech.shootRing(true)
        mech.shootRing(true)
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
        return atan(towerPose.y - position.y / towerPose.x - position.x) + 104.3.toRadians
    }

    private fun powerShotAngle(position: Vector2d, num: Int): Double {
        return atan(powerShotPose[num].y - position.y / powerShotPose[num].x - position.x) + 104.3.toRadians
    }

    val Double.a get () = (if (color == Color.BLUE) this else 360 - this)

    val Double.flip get () = (if (color == Color.BLUE) this else {
        if(this - 180 < 0) this + 180
        this - 180
    })

    val Double.y get () = (if (color == Color.BLUE) this else this * -1)
}