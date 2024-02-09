package org.firstinspires.ftc.teamcode.Detection;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BluePosDetect extends OpenCvPipeline {

    Telemetry telemetry;

    Mat mat = new Mat();
    public enum Location {
        LEFT,
        MID,
        RIGHT
    }

    private volatile Location location = Location.LEFT;

    //dreapta
    static final Rect LEFT_ROI = new Rect(
            new Point(160, 170),
            new Point(180, 190));
    //stanga
    static final Rect RIGHT_ROI = new Rect(
            new Point(50, 170),
            new Point(70, 190));
    static double PERCENT_COLOR_THRESHOLD = 0.4;

    //187 207


    public BluePosDetect(Telemetry t)
    {
        telemetry=t;
    }

    @Override
    public Mat processFrame(Mat input) {


//170 230

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        //Scalar lowHSV = new Scalar(168, 158, 154);
        //Scalar highHSV = new Scalar(179, 236, 255);

        Scalar lowHSV = new Scalar(87, 188, 36);
        Scalar highHSV = new Scalar(179, 255, 255);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);



        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;

        left.release();
        right.release();


        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(rightValue * 100) + "%");

        boolean midCase = rightValue > PERCENT_COLOR_THRESHOLD;
        boolean rightCase = leftValue > PERCENT_COLOR_THRESHOLD;

        if (!midCase && !rightCase) {
            location = Location.LEFT;
            telemetry.addData("Nivel rata", location);
        }
        else if (rightCase) {
            location = Location.RIGHT;
            telemetry.addData("Nivel rata", location);
        }
        else{
            location = Location.MID;
            telemetry.addData("Nivel rata", location);
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.RIGHT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.MID? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }

}
