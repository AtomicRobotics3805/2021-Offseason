package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.Trajectory
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.hardware.compbot.MechanismConstants.RING_DELAY
import org.firstinspires.ftc.teamcode.hardware.compbot.Mechanisms.shooter
import org.firstinspires.ftc.teamcode.hardware.compbot.Mechanisms.wobble
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.Vector2d

@Autonomous
class AutoRoutines {
    private val startPose = Pose2d()

    val initRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +wobble.raiseArm()
                +wobble.closeClaw()
            }
        }

    val lowRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +shooter.startMotor()
                +FollowTrajectory(TrajectoryFactory.startToLowToShootPowershot)
                +sequential {
                    +Delay(0.8)
                    +wobble.openClaw()
                }
            }
            +shootPowershotRoutine
            +parallel {
                +shooter.stopMotor()
                +FollowTrajectory(TrajectoryFactory.shootPowershotToWobble)
                +wobble.lowerArm()
            }
            +wobble.closeClaw()
            +parallel {
                +FollowTrajectory(TrajectoryFactory.wobbleToLowToPark)
            }
        }

    private val shootPowershotRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +shooter.shootRing()
                +sequential {
                    +Delay(RING_DELAY / 2)
                    +Turn(TrajectoryFactory.powerShotAngle(
                            Vector2d(TrajectoryFactory.startToLowToShootPowershot.end()), 1))
                }
            }
            +parallel {
                +shooter.shootRing()
                +sequential {
                    +Delay(RING_DELAY / 2)
                    +Turn(TrajectoryFactory.powerShotAngle(
                            Vector2d(TrajectoryFactory.startToLowToShootPowershot.end()), 2))
                }
            }
            +shooter.shootRing()
        }
}