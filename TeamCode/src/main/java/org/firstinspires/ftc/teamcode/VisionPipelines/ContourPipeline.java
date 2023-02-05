package org.firstinspires.ftc.teamcode.VisionPipelines;

import android.graphics.MaskFilter;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ContourPipeline extends OpenCvPipeline {

    public Scalar lower = new Scalar(0, 0, 0);
    public Scalar upper = new Scalar(255, 255, 255);


    private Mat ycrcbMat       = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat grayscaleMat   = new Mat();

    // Contour Vars

    List<MatOfPoint> contours = new ArrayList<>();
    public double lowerContourThreshold = 0;
    public double upperContourThreshold = 300;
    public Scalar contourColors = new Scalar(0,0,0);

    private Mat edgeDetectorFrame = new Mat();


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
        Imgproc.drawContours(maskedInputMat, contours, -1, contourColors );
        return maskedInputMat;
    }



    @Override
    public void onViewportTapped()
    {

    }



}