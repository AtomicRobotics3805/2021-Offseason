package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
/*
import edu.wpi.first.wpilibj.estimator.ExtendedKalmanFilter
import edu.wpi.first.wpilibj.estimator.KalmanFilterLatencyCompensator
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpiutil.math.Matrix
import edu.wpi.first.wpiutil.math.Nat
import edu.wpi.first.wpiutil.math.VecBuilder
import edu.wpi.first.wpiutil.math.numbers.N1
import edu.wpi.first.wpiutil.math.numbers.N3
import edu.wpi.first.wpiutil.math.numbers.N5
import edu.wpi.first.wpiutil.math.numbers.N6
 */
import java.util.function.BiConsumer


object LocalizationEstimator : Localizer {
    override var poseEstimate: Pose2d
        get() = TODO("Not yet implemented")
        set(value) {}
    override val poseVelocity: Pose2d?
        get() = TODO("Not yet implemented")
    /*

    private lateinit var observer: ExtendedKalmanFilter<N5, N3, N6>
    private lateinit var visionCorrect: BiConsumer<Matrix<N3, N1>, Matrix<N3, N1>>
    private lateinit var latencyCompensator: KalmanFilterLatencyCompensator<N5, N3, N6>

    private var nominalDt = -1.0
    private var prevTimeSeconds = -1.0

    private lateinit var gyroOffset: Rotation2d
    private lateinit var previousAngle: Rotation2d

    fun initialize() {
        
    }

    fun addVisionMeasurement(visionRobotPoseMeters: Pose2d, timestampSeconds: Double) {
        val oldXHat = observer.xhat
        val oldP = observer.p
        val oldSnapshots = arrayListOf(latencyCompensator.)

        // You're probably looking at this thinking "WTF is in that catch block"
        // Well... There was a bug that I couldn't track down so I used all this code to dump the EKF state
        // Hopefully I will have found the bug and fixed it by the time you're reading this, and then you can remove the stuff in the catch block
        try {
            m_latencyCompensator.applyPastGlobalMeasurement(
                    Nat.N3(),
                    m_observer, m_nominalDt,
                    VecBuilder.fill(
                            visionRobotPoseMeters.getTranslation().getX(),
                            visionRobotPoseMeters.getTranslation().getY(),
                            visionRobotPoseMeters.getRotation().getRadians()
                    ),
                    m_visionCorrect,
                    timestampSeconds
            )
        } catch (e: Exception) {
            try {
                PrintWriter(File("/home/lvuser/out.txt")).use { writer ->
                    writer.println("y:")
                    writer.println(VecBuilder.fill(
                            visionRobotPoseMeters.getTranslation().getX(),
                            visionRobotPoseMeters.getTranslation().getY(),
                            visionRobotPoseMeters.getRotation().getRadians()
                    ).toString())
                    writer.println("Timestamp:")
                    writer.println(timestampSeconds)
                    writer.println("xHat old:")
                    writer.println(oldXHat)
                    writer.println("P old:")
                    writer.println(oldP)
                    writer.println("xHat:")
                    writer.println(m_observer.getXhat())
                    writer.println("P:")
                    writer.println(m_observer.getP())
                    writer.println("Snapshots old:")
                    oldSnapshots.forEach { it ->
                        val snap: `var` = it.getValue()
                        writer.println(snap.xHat)
                        writer.println(snap.errorCovariances)
                        writer.println(snap.inputs)
                        writer.println(snap.localMeasurements)
                        writer.println("=======")
                    }
                    writer.println("Snapshots:")
                    writer.println(m_latencyCompensator.m_pastObserverSnapshots.size())
                    m_latencyCompensator.m_pastObserverSnapshots.forEach { it ->
                        val snap: `var` = it.getValue()
                        writer.println(snap.xHat)
                        writer.println(snap.errorCovariances)
                        writer.println(snap.inputs)
                        writer.println(snap.localMeasurements)
                        writer.println("=======")
                    }
                    writer.println("done")
                    writer.flush()
                }
            } catch (ex: FileNotFoundException) {
                ex.printStackTrace()
            }
        }
    }
     */

    override fun update() {
        TODO("Not yet implemented")
    }
}