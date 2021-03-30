package org.firstinspires.ftc.teamcode.util.trajectories

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint

class ParallelTrajectoryBuilder(val builder: TrajectoryBuilder) {
    private val segmentLengths = mutableListOf<Double>()

    fun build() = ParallelTrajectory(builder.build(), segmentLengths)

    fun lineTo(
            endPosition: Vector2d,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.lineTo(endPosition, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun lineToConstantHeading(
            endPosition: Vector2d,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.lineToConstantHeading(endPosition, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun lineToLinearHeading(
            endPose: Pose2d,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.lineToLinearHeading(endPose, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun lineToSplineHeading(
            endPose: Pose2d,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.lineToSplineHeading(endPose, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun strafeTo(
            endPosition: Vector2d,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.strafeTo(endPosition, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun forward(
            distance: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.forward(distance, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun back(
            distance: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.back(distance, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun strafeLeft(
            distance: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.strafeLeft(distance, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun strafeRight(
            distance: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.strafeRight(distance, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun splineTo(
            endPosition: Vector2d,
            endTangent: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.splineTo(endPosition, endTangent, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun splineToConstantHeading(
            endPosition: Vector2d,
            endTangent: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.splineToConstantHeading(endPosition, endTangent, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun splineToLinearHeading(
            endPose: Pose2d,
            endTangent: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.splineToLinearHeading(endPose, endTangent, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }

    fun splineToSplineHeading(
            endPose: Pose2d,
            endTangent: Double,
            velConstraintOverride: TrajectoryVelocityConstraint?,
            accelConstraintOverride: TrajectoryAccelerationConstraint?
    ) {
        builder.splineToSplineHeading(endPose, endTangent, velConstraintOverride, accelConstraintOverride)
        segmentLengths.add(builder.build().path.length())
    }
}