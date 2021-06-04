package org.firstinspires.ftc.teamcode.subsystems.localization

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.Localizer
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer
import org.firstinspires.ftc.teamcode.Constants.opMode
import org.firstinspires.ftc.teamcode.subsystems.driving.MecanumDrive
import org.firstinspires.ftc.teamcode.util.Pose2d
import org.firstinspires.ftc.teamcode.util.inchesToMm
import java.util.*

@Suppress("Unused", "MemberVisibilityCanBePrivate")
object VuforiaLocalizer : Localizer {
    override var poseEstimate: Pose2d
        get() = Pose2d(lastLocation) + offset
        // set should almost never be used
        set(value) {
            offset = value - poseEstimate
        }
    override var poseVelocity: Pose2d? = null

    private var offset = Pose2d()

    // Since ImageTarget trackables use mm to specify their dimensions, we must use mm for all the physical dimension.
    private val mmTargetHeight = 6.0.inchesToMm.toFloat()

    private val HALF_FIELD = 72.0.inchesToMm.toFloat()
    private val QUARTER_FIELD = 36.0.inchesToMm.toFloat()

    private var lastLocation: OpenGLMatrix = OpenGLMatrix()
    private lateinit var vuforia: VuforiaLocalizer

    private var targetVisible = false
    private lateinit var allTrackables: MutableList<VuforiaTrackable>

    fun initialize() {
        /*
         * Retrieve the camera we are to use.
         */
        val webcam = opMode.hardwareMap.get(WebcamName::class.java, "Webcam 1")

        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC screen);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         * Note: A preview window is required if you want to view the camera stream on the Driver Station Phone.
         */
        val cameraMonitorViewId: Int = opMode.hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorViewId)

        parameters.vuforiaLicenseKey = MecanumDrive.Constants.VUFORIA_KEY

        /*
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcam

        // Make sure extended tracking is disabled for this program.
        parameters.useExtendedTracking = false

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        val targetsUltimateGoal: VuforiaTrackables = vuforia.loadTrackablesFromAsset("UltimateGoal")
        val blueTowerGoalTarget = targetsUltimateGoal[0]
        blueTowerGoalTarget.name = "Blue Tower Goal Target"
        val redTowerGoalTarget = targetsUltimateGoal[1]
        redTowerGoalTarget.name = "Red Tower Goal Target"
        val redAllianceTarget = targetsUltimateGoal[2]
        redAllianceTarget.name = "Red Alliance Target"
        val blueAllianceTarget = targetsUltimateGoal[3]
        blueAllianceTarget.name = "Blue Alliance Target"
        val frontWallTarget = targetsUltimateGoal[4]
        frontWallTarget.name = "Front Wall Target"

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        allTrackables = ArrayList()
        allTrackables.addAll(targetsUltimateGoal)

        //Set the position of the perimeter targets with relation to origin (center of field)
        /*
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of *transformation matrices.*
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See [Transformation Matrix](https://en.wikipedia.org/wiki/Transformation_matrix)
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the [OpenGLMatrix] class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         * - The X axis runs from your left to the right. (positive from the center to the right)
         * - The Y axis runs from the Red Alliance Station towards the other side of the field
         * where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         * - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         * coordinate system (the center of the field), facing up.
         */

        //Set the position of the perimeter targets with relation to origin (center of field)
        redAllianceTarget.location = OpenGLMatrix
                .translation(0f, -HALF_FIELD, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 180f))

        blueAllianceTarget.location = OpenGLMatrix
                .translation(0f, HALF_FIELD, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 0f))
        frontWallTarget.location = OpenGLMatrix
                .translation(-HALF_FIELD, 0f, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 90f))

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.location = OpenGLMatrix
                .translation(HALF_FIELD, QUARTER_FIELD, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, -90f))
        redTowerGoalTarget.location = OpenGLMatrix
                .translation(HALF_FIELD, -QUARTER_FIELD, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, -90f))

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // For a WebCam, the default starting orientation of the camera is looking UP (pointing in the Z direction),
        // with the wide (horizontal) axis of the camera aligned with the X axis, and
        // the Narrow (vertical) axis of the camera aligned with the Y axis
        //
        // But, this program assumes that the camera is actually facing forward out the front of the robot.
        // So, the "default" camera position requires two rotations to get it oriented correctly.
        // 1) First it must be rotated +90 degrees around the X axis to get it horizontal (it's now facing out the right side of the robot)
        // 2) Next it must be be rotated +90 degrees (counter-clockwise) around the Z axis to face forward.
        //
        // Finally the camera can be translated to its actual mounting position on the robot.

        /* Let all the trackable listeners know where the phone is.  */
        for (trackable in allTrackables) {
            (trackable.listener as VuforiaTrackableDefaultListener).setCameraLocationOnRobot(parameters.cameraName!!, MecanumDrive.cameraLocationOnRobot)
        }

        // Note: To use the remote camera preview:
        // AFTER you hit Init on the Driver Station, use the "options menu" to select "Camera Stream"
        // Tap the preview window to receive a fresh image.
        targetsUltimateGoal.activate()
    }

    override fun update() {
        // check all the trackable targets to see which one (if any) is visible.
        targetVisible = false
        for (trackable in allTrackables) {
            if ((trackable.listener as VuforiaTrackableDefaultListener).isVisible) {
                MecanumDrive.telemetry.addData("Visible Target", trackable.name)
                targetVisible = true

                // getUpdatedRobotLocation() will return null if no new information is available since
                // the last time that call was made, or if the trackable is not currently visible.
                val robotLocationTransform = (trackable.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform
                }
                break
            }
        }
    }
}