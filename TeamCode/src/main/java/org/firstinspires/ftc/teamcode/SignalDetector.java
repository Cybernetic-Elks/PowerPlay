package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SignalDetector extends OpenCvPipeline {
    /**
     * Programmer:    Sean Pakros
     * Date Created:  12/15/21
     * Purpose: Code for detecting our shipping element using openCV.
     * This is based off of another teams code (Wolf Corp Robotics 12525)
     * for detecting skystones in a previous season which I then modified to work for this season and our shipping element.
     */
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Side {
        PURPLE,
        YELLOW,
        GREEN
    }
    private Side side;

    static final Rect ROI = new Rect(
            new Point(150, 70),       //180,70
            new Point(120, 140));     //140,140
    static double PERCENT_COLOR_THRESHOLD = 0.20;

    public SignalDetector(Telemetry t) { telemetry = t; }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSVYellow = new Scalar(23, 50, 70);
        Scalar highHSVYellow = new Scalar(32, 255, 255);

        Core.inRange(mat, lowHSVYellow, highHSVYellow, mat);

        Mat yellow = mat.submat(ROI);

        double yellowValue = Core.sumElems(yellow).val[0] / ROI.area() / 255;

        yellow.release();

        telemetry.addData("Yellow raw value", (int) Core.sumElems(yellow).val[0]);
        telemetry.addData("Yellow percentage", Math.round(yellowValue * 100) + "%");

        boolean sideYellow = yellowValue > PERCENT_COLOR_THRESHOLD;

        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVPurple = new Scalar(77, 20, 56);
        Scalar highHSVPurple = new Scalar(235, 149, 204);

        Core.inRange(mat, lowHSVPurple, highHSVPurple, mat);

        Mat purple = mat.submat(ROI);

        double purpleValue = Core.sumElems(purple).val[0] / ROI.area() / 255;

        purple.release();

        telemetry.addData("purple raw value", (int) Core.sumElems(purple).val[0]);
        telemetry.addData("purple percentage", Math.round(purpleValue * 100) + "%");
        boolean sidePurple = purpleValue > PERCENT_COLOR_THRESHOLD;

        if (sidePurple) {
            side = Side.PURPLE;
            telemetry.addData("Side:", "PURPLE");
        } else if (sideYellow) {
            side = Side.YELLOW;
            telemetry.addData("Side:", "YELLOW");
        } else {
            side = Side.GREEN;
            telemetry.addData("Side", "GREEN");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorYellow = new Scalar(238, 255, 0);
        Scalar colorPurple = new Scalar(158, 0, 255);
        Scalar colorGreen = new Scalar(0, 255, 0);

        if (side == Side.PURPLE) {Imgproc.rectangle(mat, ROI, colorPurple);}
        else if (side == Side.YELLOW) {Imgproc.rectangle(mat, ROI, colorYellow);}
        else {Imgproc.rectangle(mat, ROI, colorGreen);}

        return mat;
    }

    public Side getSide() {
        return side;
    }
}