package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionPipelines.ActuallyContourPipeline;
import org.firstinspires.ftc.teamcode.VisionPipelines.ContourPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "WEBCAMTESTINGBOOOOOOO", group = "wecam shit")
public class AutonoomousWebcamTesting extends OpMode
{

    // why does Android studio have a spell checker
    // webcamdude is the webcam. I'm not naming it anything else
    OpenCvWebcam Webcam = null;

    Servo claw;

    public double claw_close_thresh = 0;
    boolean clawOn = true;
    ElapsedTime quick_timer = new ElapsedTime();

    @Override // This stuff happens when you click the init button
    public void init()
    {
        claw = hardwareMap.servo.get("claw");



        WebcamName webcamname = hardwareMap.get(WebcamName.class, "Webcam");
        // Acquire the camera ID
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName() );
        //set the cam name and id to the webcam.
        Webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamname, cameraMonitorViewId);

        //set webcam Pipeline
        Webcam.setPipeline(new ActuallyContourPipeline());

        // 0 for red team, 1 for blue team
        ActuallyContourPipeline.setTeamFilter(0);

        //You're creating an instance of the AsyncCameraOpenListener class. The class contains the two methods, onOpened and onError, which you are overriding with your code. That instance is passed to the openCameraDeviceAsync method as a parameter.
        Webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                Webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("Webcam not working");
            }
        });
    }

    @Override // This stuff happens when you click the play button
    public void loop()
    {
        double current_largest_contour = ActuallyContourPipeline.getLargestSize();
        telemetry.addData("Contour Area", current_largest_contour);

        if (current_largest_contour > claw_close_thresh && clawOn)
        {
            claw.setPosition(0.8);
            clawOn = false;
        }
        else
        {
            claw.setPosition(0.4);
        }
    }
}

