package org.firstinspires.ftc.teamcode.VisionPipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.robotcore.external.Telemetry;






public class ContourPipeline extends OpenCvPipeline {

    Telemetry telemetry;

    public ContourPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Scalar lower = new Scalar(0, 160.1, 0);
    public Scalar upper = new Scalar(255, 255, 255);


    private final Mat ycrcbMat = new Mat();
    private Mat binaryMat      = new Mat();
    private Mat maskedInputMat = new Mat();
    private Mat grayscaleMat   = new Mat();

    // Contour Vars

    List<MatOfPoint> contours = new ArrayList<>();
    public double lowerContourThreshold = 0;
    public double upperContourThreshold = 300;
    public Scalar contourColors = new Scalar(0,255,0);
    public double noiseSensitivity = 153;
    public int contourSize = 1;

    private Mat edgeDetectorFrame = new Mat();
    private int onlycontours = 1;

    private double currentLargest = 0;



    @Override
    public Mat processFrame(Mat input) {

        telemetry.addData("[greeting]", "Hello!");

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

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];

        Rect[] boundRect = new Rect[contours.size()];
        //Might be useful later, idk so I am going to leave this here
        Point[] centers = new Point[contours.size()];


        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(contours.get(i));
            centers[i] = new Point();
        }


        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            //Just to make sure that no problems really come up.
            if(poly == null)
            {
                break;
            }

            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }


        Mat activeMat;
        Mat emptyMat = Mat.zeros(edgeDetectorFrame.size(), CvType.CV_8UC3);
        int index = 0;
        double contourArea;
        double largestArea = 0;


        if (onlycontours == 1)
        {
            activeMat = maskedInputMat;
        }
        else if (onlycontours == 2)
        {
            activeMat = emptyMat;
        }
        else {
            activeMat = input;
        }

        for (int i = 0; i < contours.size(); i++) {

            MatOfPoint contour = contours.get(i);

            contourArea = Imgproc.contourArea(contour);

            // A simple filter I cam up with ;P
            //
            if (contourArea < noiseSensitivity)
            {
                continue;
            }

            if (contourArea > largestArea)
            {
                largestArea = contourArea;
                index = i;
            }

        }

        currentLargest = largestArea;

        if (contours.size() != 0 && largestArea != 0)
        {
            // Adding Text3
            Imgproc.putText (
                    activeMat,                          // Matrix obj of the image
                    largestArea + "",          // Text to be added
                    new Point(boundRect[index].tl().x, boundRect[index].tl().y),               // point
                    Imgproc.FONT_HERSHEY_SIMPLEX ,      // front face
                    1,                               // front scale
                    contourColors,             // Scalar object for color
                    2                               // Thickness
            );
            Imgproc.drawContours(activeMat, contoursPolyList, index, contourColors, contourSize);
            Imgproc.rectangle(activeMat, boundRect[index].tl(), boundRect[index].br(), contourColors, 2);

        }

        telemetry.update();

        return activeMat;
    }

    public double getLargestSize()
    {
        return currentLargest;
    }


    @Override
    public void onViewportTapped()
    {
        if (onlycontours == 1 || onlycontours == 2)
        {
            onlycontours++;
        }
        else
        {
            onlycontours = 1;
        }
    }

}