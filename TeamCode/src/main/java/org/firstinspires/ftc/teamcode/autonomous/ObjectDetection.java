package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.hardware.BaseMecanumDrive;

import java.util.List;

public class ObjectDetection {
    public enum StackSize {
        NONE,
        ONE,
        FOUR
    }private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private ElapsedTime runtime = new ElapsedTime();
    StackSize quantity;
    List<Recognition> recognitions;


    private static final String VUFORIA_KEY =
            " AZ2jk6P/////AAABmck8NCyjWkCGvLdpx9HZ1kxI2vQPDlzN9vJnqy69nXRjvoXgBCEWZasRnd1hFjBpRiSXw4G4JwDFsk3kNSVko2UkuCgbi/RsiODF76MtldIi6YZGfrRMZTICMKwTanuOysh4Cn9Xd9nZzCpDiLAPLsUtKoj/DdBUn0gJuARMglUPW7/qirgtk0xI232ttZpXhgh9ya8R8LxnH+UTCCFtEaQft2ru0Tv+30Un82gG1uEzcrMc/8F3lefedcOTrelPQx8xUD8cME9dj99b5oZWfM60b36/xdswhYF7pygskPtXCS28j81xWKHGNhr5s8xL91cbKOovDzdJYdfVIILZnL1sjdbtN8zW4mULOYHwO4ur ";


    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    //Tuning Variables
    int ring_width = 120;
    int q_height = 98;
    int s_height = 63;

    double screen_fraction = 0.5;

    HardwareMap hardwareMap;
    LinearOpMode opMode;

    public void init(LinearOpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        this.opMode = opMode;

        initVuforia();
        initTfod();



        if (tfod != null) {
            tfod.activate();
        }

        runtime.reset();
    }

    public StackSize detect() {
       if (!opMode.isStopRequested()) {
            recognitions = tfod.getRecognitions();
            if (tfod != null) {
                if (recognitions.size() != 0) {
                    for (Recognition new_recognition : recognitions) {
                        double screen_middle = new_recognition.getImageWidth() * screen_fraction;
                        double object_position = (new_recognition.getRight()-new_recognition.getLeft())/2 + new_recognition.getLeft();
                        String check = measure(new_recognition.getHeight(), new_recognition.getWidth());
                        //if (object_position<(screen_middle+35) && object_position>(screen_middle-35)){
                            if (check.equals("Single")){
                                quantity = StackSize.ONE;
                                break;
                            }
                            else {
                                quantity = StackSize.FOUR;
                                break;
                            }
                        }
                        //else{
                        //    quantity = StackSize.NONE;
                        //}
                    //}
                }
                else {
                    quantity = StackSize.NONE;
                }
                return quantity;
            }

        }
        return StackSize.NONE;
    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.55f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private String measure (double boxHeight, double boxWidth){


        String answer = "Zero";

        if(boxWidth * 0.8 > boxHeight) {
            answer = "Single";
        }
        else {
            answer = "Quad";
        }
        return answer;
    }
}
