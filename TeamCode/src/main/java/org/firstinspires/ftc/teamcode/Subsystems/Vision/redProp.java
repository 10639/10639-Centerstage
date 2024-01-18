package org.firstinspires.ftc.teamcode.Subsystems.Vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class redProp extends OpenCvPipeline {

    Telemetry telemetry;

    public redProp(Telemetry t) {
        telemetry = t;
    } // constructor for the class to set up telemetry

    Mat mat = new Mat(); // declare a new matrix (computer representation of an image)
    Rect leftRect = new Rect(0, 150, 100, 100); // define our regions of interest (where the algorithm is focusing on) as rectangles
    Rect midRect = new Rect(280, 125, 100, 100);
    Rect rightRect = new Rect(520, 150, 100, 100);
    final double PERCENT_THRESHOLD = 0.19; // define our threshold
    public int finalAnswer;
    Scalar red = new Scalar(255, 0, 0); // define what the color of the rectangle outline is that appears on the output (red)

    public enum Location {
        RIGHT,
        MIDDLE,
        LEFT
    }

    private Location location;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV); // change the color space from rgb to HSV (Hue, Saturation, Value)
        Scalar lowRedBound = new Scalar(0, 100, 20); // set lower and upper bounds for the color we want to recognize (red in this case)
        Scalar highRedBound = new Scalar(10, 255, 255);

//        Scalar lowRedBound2 = new Scalar(160, 100, 35);
//        Scalar highRedBound2 = new Scalar(180, 255, 255);
//        technically red is also found in the 160-180 hue range for HSV, but let's see if it'll work with just the hue range of 0-10

        Core.inRange(mat, lowRedBound, highRedBound, mat); // see which pixels are in our range, convert the pixels we're looking for into white, store it back to mat i think
        Mat left = mat.submat(leftRect); // create sub-matrices for our regions of interest
        Mat middle = mat.submat(midRect);
        Mat right = mat.submat(rightRect);

        double leftValue = Core.sumElems(left).val[0] / leftRect.area() / 255; // get the percentage of white pixels that are present
        double midValue = Core.sumElems(middle).val[0] / midRect.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / rightRect.area() / 255;

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]); // display the raw values to the driver hub
        telemetry.addData("Center raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);

        left.release();
        right.release();
        middle.release();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, leftRect, red, 2); // draw the rectangles on the output matrix
        Imgproc.rectangle(mat, rightRect, red, 2);
        Imgproc.rectangle(mat, midRect, red, 2);

        if (leftValue < PERCENT_THRESHOLD && midValue < PERCENT_THRESHOLD && rightValue < PERCENT_THRESHOLD) {
            finalAnswer = 0;
        } else {
            if (leftValue > midValue && leftValue > rightValue) {
                finalAnswer = 1;
            } else if (midValue > leftValue && midValue > rightValue) {
                finalAnswer = 2;
            } else if (rightValue > leftValue && rightValue > midValue) {
                finalAnswer = 3;
            } else if (leftValue == midValue && leftValue == rightValue && midValue == rightValue) {
                finalAnswer = 4;
            } else {
                finalAnswer = 5;
            }
        }

        if (finalAnswer == 0) {
            telemetry.addData("Position", "Fix Percentage Threshold ");
            if (leftValue > midValue && leftValue > rightValue) {
                telemetry.addData("Position", "[Estimate] Left Marker");
                location = Location.LEFT;
                Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green
            } else if (midValue > leftValue && midValue > rightValue) {
                telemetry.addData("Position", "[Estimate] Middle Marker");
                location = Location.MIDDLE;
                Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);
            } else if (rightValue > leftValue && rightValue > midValue) {
                telemetry.addData("Position", "[Estimate] Right Marker");
                location = Location.RIGHT;
                Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);
            }

        } else if (finalAnswer == 1) {

            telemetry.addData("Position", "[Confident] Left Marker");
            location = Location.LEFT;
            Imgproc.rectangle(mat, leftRect, new Scalar(0, 255, 0), 2); // change the rectangle of the detected position to green

        } else if (finalAnswer == 3) {

            telemetry.addData("Position", "[Confident] Right Marker");
            location = Location.RIGHT;
            Imgproc.rectangle(mat, rightRect, new Scalar(0, 255, 0), 2);

        } else if (finalAnswer == 2) {

            telemetry.addData("Position", "[Confident] Center Marker");
            location = Location.MIDDLE;
            Imgproc.rectangle(mat, midRect, new Scalar(0, 255, 0), 2);

        } else {
            telemetry.addData("Debugging", "Camera Error");
        }
        telemetry.update();


        return mat;
    }

    public Location getLocation() {
        return location;
    }
}