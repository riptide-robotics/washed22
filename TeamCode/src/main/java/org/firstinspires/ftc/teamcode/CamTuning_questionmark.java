package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//aight lets try and never use this k

@TeleOp(name="camtunine?", group="Demos")
public class CamTuning_questionmark extends LinearOpMode {

    public static double threshold = 0;

    public static double close = 0;
    public static double open = 1;

    //Servo claw = hardwareMap.servo.get("4_servo5claw");

    OpenCvWebcam webcam = null;


    @Override
    public void runOpMode() throws InterruptedException {
        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam 1");
        // Acquire the camera ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );
        //set the cam name and id to the webcam.
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);

        //set webcam Pipeline
        webcam.setPipeline(new ContourPipeline());

        //You're creating an instance of the AsyncCameraOpenListener class. The class contains the two methods, onOpened and onError, which you are overriding with your code. That instance is passed to the openCameraDeviceAsync method as a parameter.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Webcam not working");
            }
        });
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double Largest_Contour = ContourPipeline.getLargestSize();

            if (Largest_Contour > threshold)
            {
                //claw.setPosition(close);
                telemetry.addLine("Webcam not working");
            }
            else
            {
                //claw.setPosition(open);
                telemetry.addLine("Webcam not working");
            }
        }
        telemetry.update();
    }
}


