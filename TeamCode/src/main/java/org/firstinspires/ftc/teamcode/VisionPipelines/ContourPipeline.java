package org.firstinspires.ftc.teamcode.VisionPipelines;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
//import org.opencv.core.MatOfPoint2f;
//import org.opencv.core.Point;
//import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);


    private final Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat grayscaleMat   = new Mat();

    // Contour Vars

    List<MatOfPoint> contours = new ArrayList<>();
    public double lowerContourThreshold = 0;
    public double upperContourThreshold = 300;
    public Scalar contourColors = new Scalar(0,0,0);
    public double noiseSensitivity = 0;

    private Mat edgeDetectorFrame = new Mat();
    //private double contourPerim;




    @Override
    public Mat processFrame(Mat input) {
         //Takes our "input" mat as an input, and outputs to a separate Mat buffer "ycrcbMat"
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);

         //Order is source, lowerbound, upperbound, dst.
        Core.inRange(ycrcbMat, lower, upper, binaryMat);

        /*
         * Release the reusable Mat so that old data doesn't
         * affect the next step in the current processing
         */
        maskedInputMat.release();

         //Order: src1, src2, dst, mask.
        Core.bitwise_and(input, input, maskedInputMat, binaryMat);

        // now the masked input mat is the filtered image with colors and shit.
        // Sorry Alok I ripped u off ;P

        // filter maskedinputmat to grey.
        Imgproc.cvtColor(maskedInputMat, grayscaleMat, Imgproc.COLOR_RGB2GRAY);

        // Order: input image, output edges(Array), lowerthreshold, upperthreshold
        Imgproc.Canny(grayscaleMat, edgeDetectorFrame, lowerContourThreshold, upperContourThreshold);

        contours.clear();
        //Order : input image, the contours from output, hierarchy, mode, and method

        Imgproc.findContours(edgeDetectorFrame, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        //MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        //Rect[] boundingRect = new Rect[contours.size()];

        for (int i = 0; i < contours.size() ; i++) {

            MatOfPoint contour = contours.get(i);

            List<MatOfPoint> singularContour = new ArrayList<>();
            singularContour.clear();
            singularContour.add(contour);

            // Noise remover :)
            if (Imgproc.contourArea(contour) > noiseSensitivity) {
                Imgproc.drawContours(maskedInputMat, singularContour, -1, contourColors, 2);
                telemetry.addData("Contour Area", Imgproc.contourArea(contour));
                //MatOfPoint2f mop2f = new MatOfPoint2f(contour.toArray());
                //contourPerim = Imgproc.arcLength(mop2f, true);
                //Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 0.02 * contourPerim, true);
                //boundingRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

            }
        }

/*
            Rect[] boundRect = new Rect[contours.size()];

            Point[] centers = new Point[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();

                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                centers[i] = new Point();
            }
 */
        return maskedInputMat;
    }



    @Override
    public void onViewportTapped()
    {

    }



}