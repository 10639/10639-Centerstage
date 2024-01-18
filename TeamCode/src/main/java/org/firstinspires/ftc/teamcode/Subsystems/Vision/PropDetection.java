package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.firstinspires.ftc.teamcode.Subsystems.Scoring.Constants;

public class PropDetection extends OpenCvPipeline {

    Telemetry telemetry;
    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    boolean Blue = true;
    boolean isRed;
    public PropDetection(Telemetry t, boolean red) {
        telemetry = t;
        if(red) Blue = false;
    } // constructor for the class to set up telemetry


    Mat mat = new Mat(), mat1 = new Mat();
    Rect leftRect = new Rect(140, 270, 200, 100); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(570, 260, 530, 100);
    double PERCENT_THRESHOLD = Constants.CONFIDENCE; // define our threshold
    private Location location;

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        isRed = !Blue;
        Scalar color = isRed ? new Scalar(0, 0, 255) : new Scalar(255, 0, 0); // Blue
        Scalar lowColorBound = new Scalar(100, 40, 40); // set lower and upper bounds for the color we want to recognize (blue in this case)
        Scalar highColorBound = new Scalar(125, 500, 500);
        Scalar lowColorBound_Range = new Scalar(0, 0, 0); //Place Holders for Red
        Scalar highColorBound_Range = new Scalar(0, 0, 0); // Place Holders for Red
        Mat hold = new Mat();
        Mat cone = new Mat();

        if(isRed) {
            lowColorBound = new Scalar(0, 60, 60);
            highColorBound = new Scalar(20, 500, 500);
            lowColorBound_Range = new Scalar(160, 60, 60);
            highColorBound_Range = new Scalar(180, 255, 255);
            }

        Core.inRange(mat, lowColorBound, highColorBound, mat);
        if(isRed) {
            Core.inRange(mat, lowColorBound_Range, highColorBound_Range, mat1);
            Core.add(hold, mat1, cone);
        }
        Mat left = isRed ? cone.submat(leftRect) : mat.submat(leftRect);
        Mat middle = isRed ? cone.submat(midRect) : mat.submat(midRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255;
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;

        telemetry.addData("Left Value", (int) Core.sumElems(left).val[0]); // display the raw values to the driver hub
        telemetry.addData("Middle Value", (int) Core.sumElems(middle).val[0]);

        releaseMats(left, middle, mat1, cone, hold);

        Imgproc.rectangle(isRed ? cone : mat, leftRect, color, 2); // draw the rectangles on the output matrix
        Imgproc.rectangle(isRed ? cone : mat, midRect, color, 2);

        if(leftValue > PERCENT_THRESHOLD || midValue > PERCENT_THRESHOLD){
            if(leftValue > midValue) {
                location = Location.LEFT;
            } else {
                location = Location.MIDDLE;
            }
            Imgproc.rectangle(isRed ? cone : mat, midRect, new Scalar(0, 255, 0), 2);
        } else {
            location = Location.RIGHT;
            Imgproc.rectangle(isRed ? cone : mat, new Rect(100, 100, 50, 50), new Scalar(0, 255, 0), 2);
        }
        telemetry.addData("Location: ", location);
        return mat;
    }

    public Location getLocation() {
        return location;
    }

    public void stop(OpenCvCamera camera) {
        camera.stopStreaming();
    }

    public void releaseMats(Mat... mats) {
        for (Mat mat : mats) {
            if (mat != null) {
                mat.release();
            }
        }
    }
}