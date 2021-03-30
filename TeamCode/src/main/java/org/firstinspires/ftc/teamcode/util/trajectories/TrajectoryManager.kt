package org.firstinspires.ftc.teamcode.util.trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint
import org.firstinspires.ftc.teamcode.Constants.constants

fun trajectoryBuilder(startPose: Pose2d): TrajectoryBuilder {
    val velConstraint = MinVelocityConstraint(listOf(
            AngularVelocityConstraint(constants.maxAngVel),
            MecanumVelocityConstraint(constants.maxVel, constants.trackWidth)
    ))
    val accelConstraint = ProfileAccelerationConstraint(constants.maxAccel)
    return TrajectoryBuilder(startPose, false, velConstraint, accelConstraint)
}