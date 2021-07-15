package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.localization.Localizer
import edu.wpi.first.wpilibj.estimator.ExtendedKalmanFilter
import edu.wpi.first.wpilibj.estimator.KalmanFilterLatencyCompensator
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.math.Discretization
import edu.wpi.first.wpilibj.math.StateSpaceUtil
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.VecBuilder
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N3
import edu.wpi.first.wpiutil.math.numbers.N5
import edu.wpi.first.wpiutil.math.numbers.N6
import java.util.function.BiConsumer
import java.util.function.BiFunction
import com.acmerobotics.roadrunner.geometry.Pose2d as RoadRunnerPose2d



/**
 * @author FRC Team Spartronics4915
 */
/*
object LocalizationEstimator : Localizer {
    override var poseEstimate: RoadRunnerPose2d
        get() = TODO("Not yet implemented")
        set(value) {}
    override val poseVelocity: RoadRunnerPose2d?
        get() = TODO("Not yet implemented")

    private lateinit var observer: ExtendedKalmanFilter<N5, N3, N6>
    private lateinit var visionCorrect: BiConsumer<Matrix<N3, N1>, Matrix<N3, N1>>
    private lateinit var latencyCompensator: KalmanFilterLatencyCompensator<N5, N3, N6>

    private var nominalDt = -1.0
    private var prevTimeSeconds = -1.0

    private lateinit var gyroOffset: Rotation2d
    private lateinit var previousAngle: Rotation2d
    fun initialize(
            gyroAngle: Rotation2d, initialPoseMeters: Pose2d,
            stateStdDevs: Matrix<N5, N1>,
            localMeasurementStdDevs: Matrix<N6, N1>, visionMeasurementStdDevs: Matrix<N3, N1>,
            nominalDtSeconds: Double
    ) {
        nominalDt = nominalDtSeconds

        observer = ExtendedKalmanFilter(
                Nat.N5(), Nat.N3(), Nat.N6(),
                this::f,
                BiFunction { x: Matrix<N5?, N1?>, u: Matrix<N3?, N1?>? ->
                    VecBuilder.fill(
                            x[0, 0], x[1, 0], x[2, 0],
                            x[3, 0], x[4, 0],
                            x[2, 0]
                    )
                },
                stateStdDevs, localMeasurementStdDevs,
                nominalDt
        )
        latencyCompensator = KalmanFilterLatencyCompensator<N5, N3, N6>()

        val visionContR = StateSpaceUtil.makeCovarianceMatrix(Nat.N3(), visionMeasurementStdDevs)
        val visionDiscR = Discretization.discretizeR(visionContR, nominalDt)
        visionCorrect = BiConsumer { u: Matrix<N3, N1>, y: Matrix<N3, N1> ->
            observer.correct(
                    Nat.N3(), u, y,
                    { x, _ -> Matrix<N3, N1>(x.storage.extractMatrix(0, 3, 0, 1)) },
                    visionDiscR
            )
        }

        gyroOffset = initialPoseMeters.rotation.rotateBy(gyroAngle.inverse())
        previousAngle = initialPoseMeters.rotation
        observer.setXhat(fillStateVector(initialPoseMeters, 0.0, 0.0))
    }

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        val oldXHat = observer.xhat
        val oldP = observer.p
        val oldSnapshots = arrayListOf(latencyCompensator.)
            latencyCompensator.applyPastGlobalMeasurement(
                    Nat.N3(),
                    observer, nominalDt,
                    VecBuilder.fill(
                            visionRobotPoseMeters.getTranslation().getX(),
                            visionRobotPoseMeters.getTranslation().getY(),
                            visionRobotPoseMeters.getRotation().getRadians()
                    ),
                    visionCorrect,
                    timestampSeconds
            )
    }

    override fun update() {
        TODO("Not yet implemented")
    }

}
*/