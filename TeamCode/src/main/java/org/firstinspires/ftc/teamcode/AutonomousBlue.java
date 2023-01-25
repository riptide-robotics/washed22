/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.*;
//import com.qualcomm.robotcore.hardware.HardwareMap.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

@Autonomous(name="meet 3 auto", group="Cool Stuffs")
public class AutonomousBlue extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;


    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    //double y = 0; // Remember, this is reversed!
    //double x = 0; // Counteract imperfect strafing
    //double rx = 0;
    int tagNum = -1;
    //double denominator;
    //double frontLeftPower;
    //double backLeftPower;
    //double frontRightPower;
    //double backRightPower;
    // UNITS ARE METERS
    double tagsize = 0.047;
    // old tagsize - 0.166

    //int tag1 = 18;
    //int tag2 = 13;
    //int tag3 = 0;
    // Tag ID 18 from the 36h11 family

    //AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        //motor mapping and setmodes.
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        Servo servo0 = hardwareMap.servo.get("servo0");
        Servo servo1 = hardwareMap.servo.get("servo1");
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //cam shit
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            servo0.setPosition(0);
            servo1.setPosition(1);

            if (currentDetections.size() != 0) {




                for (AprilTagDetection tag : currentDetections) {
                    tagNum = tag.id;
                }

                if (tagNum == 1) {
                    // pos 1
                    telemetry.addLine("tag 1 detected yo, you a real one");



                }
                if (tagNum == 2) {
                    // pos 2
                    telemetry.addLine("tag 2 detected yo, doing well fam");

                }
                if (tagNum == 3) {
                    // pos 3
                    telemetry.addLine("tag 3 detected yo, hope you bussing g");

                }
                telemetry.update();
                sleep(20);
            }
        }

        if (opModeIsActive()) {
            if (tagNum == 1) {
                servo0.setPosition(0.4);
                servo1.setPosition(0.45);
                motorBackLeft.setPower(-0.25);
                motorFrontRight.setPower(-0.25);
                motorBackRight.setPower(0.25);
                motorFrontLeft.setPower(0.25);
                sleep(2250);
                motorBackLeft.setPower(-0.25);
                motorFrontRight.setPower(-0.25);
                motorBackRight.setPower(-0.25);
                motorFrontLeft.setPower(-0.25);
                sleep(2200);

                // pos 1
                /*
                motorBackRight.setTargetPosition(200);
                motorBackLeft.setTargetPosition(200);
                motorFrontRight.setTargetPosition(200);
                motorFrontLeft.setTargetPosition(200);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setPower(0.2);
                motorFrontRight.setPower(0.2);
                motorBackRight.setPower(0.2);
                motorFrontLeft.setPower(0.2);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //sleep(1876);
                motorBackLeft.setTargetPosition(1000);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorBackRight.setTargetPosition(1000);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorFrontRight.setTargetPosition(1000);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorFrontLeft.setTargetPosition(1000);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                motorFrontLeft.setPower(0.1);
                motorFrontRight.setPower(0.1);
                motorBackLeft.setPower(0.1);
                motorBackRight.setPower(0.1);
                */

            }
            else if (tagNum == 2) {
                // pos 2
                servo0.setPosition(0.4);
                servo1.setPosition(0.45);
                motorBackLeft.setPower(-0.25);
                motorFrontRight.setPower(-0.25);
                motorBackRight.setPower(-0.25);
                motorFrontLeft.setPower(-0.25);
                telemetry.update();
                sleep(2200);

            }
            else if (tagNum == 3) {
                // pos 3
                servo0.setPosition(0.4);
                servo1.setPosition(0.45);
                motorBackLeft.setPower(0.25);
                motorFrontRight.setPower(0.25);
                motorBackRight.setPower(-0.25);
                motorFrontLeft.setPower(-0.25);
                sleep(2200);
                motorBackLeft.setPower(-0.25);
                motorFrontRight.setPower(-0.25);
                motorBackRight.setPower(-0.25);
                motorFrontLeft.setPower(-0.25);
                sleep(2250);





            }
            else
            {
                motorBackLeft.setPower(-0.25);
                motorFrontRight.setPower(-0.25);
                motorBackRight.setPower(-0.25);
                motorFrontLeft.setPower(-0.25);
                sleep(2200);

            }
        }
        }
            /*
             * The START command just came in: now work off the latest snapshot acquired
             * during the init loop.
             */

            /* Update the telemetry */

            /* Actually do something useful */



            /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */

        }



