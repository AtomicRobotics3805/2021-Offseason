package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.commands.*
import org.firstinspires.ftc.teamcode.util.commands.delays.Delay
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Intake
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Shooter.RING_DELAY
import org.firstinspires.ftc.teamcode.subsystems.mechanisms.Wobble
import org.firstinspires.ftc.teamcode.trajectory.TrajectoryFactory
import org.firstinspires.ftc.teamcode.util.Vector2d

@Autonomous
@Suppress("Unused")
class AutoRoutines {
    private val startPose = Pose2d()

    val initRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Wobble.raiseArm
                +Wobble.closeClaw
                +Intake.start
            }
        }

    val lowRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Shooter.start
                +MecanumDrive.followTrajectory(TrajectoryFactory.startToLowToShootPowershot)
                +sequential {
                    +Delay(0.8)
                    +Wobble.openClaw
                }
            }
            +shootPowershotRoutine
            +parallel {
                +Shooter.stop
                +MecanumDrive.followTrajectory(TrajectoryFactory.shootPowershotToWobble)
                +Wobble.lowerArm
            }
            +Wobble.closeClaw
            +parallel {
                +MecanumDrive.followTrajectory(TrajectoryFactory.wobbleToLowToPark)
            }
        }

    private val shootPowershotRoutine: AtomicCommand
        get() = sequential {
            +parallel {
                +Shooter.shootRing
                +sequential {
                    +Delay(RING_DELAY / 2)
                    +MecanumDrive.turn(TrajectoryFactory.powerShotAngle(
                            Vector2d(TrajectoryFactory.startToLowToShootPowershot.trajectory.end()), 1))
                }
            }
            +parallel {
                +Shooter.shootRing
                +sequential {
                    +Delay(RING_DELAY / 2)
                    +MecanumDrive.turn(TrajectoryFactory.powerShotAngle(
                            Vector2d(TrajectoryFactory.startToLowToShootPowershot.trajectory.end()), 2))
                }
            }
            +Shooter.shootRing
        }
}