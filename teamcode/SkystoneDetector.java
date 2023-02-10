package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SkystoneDetector extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    public enum Location {
        LEFT,
        RIGHT,
        MIDDLE
    }
    private Location location;

    static final Rect LEFT_ROI = new Rect(
            new Point(230, 50),
            new Point(310, 200));
    static final Rect RIGHT_ROI = new Rect(
            new Point(1, 50),
            new Point(70, 200));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(90, 50),
            new Point(200, 200));
    static double PERCENT_COLOR_THRESHOLD = 0.1;

    public SkystoneDetector(Telemetry t) { telemetry = t; }

    @Config
    public static class RobotConstants {
        public static int HLow = 3;
        public static int SLow = 100;
        public static int vLow = 110;
        public static int HHigh = 40;
        public static int SHigh= 130;
        public  static int vHigh = 140;
        //public static PIDCoefficients TURNING_PID = new PIDCoefficients();
        // other constants
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
        Scalar lowHSV = new Scalar(RobotConstants.HLow, RobotConstants.SLow, RobotConstants.vLow);
        Scalar highHSV = new Scalar(RobotConstants.HHigh, RobotConstants.SHigh, RobotConstants.vHigh);

        Core.inRange(mat, lowHSV, highHSV, mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat right = mat.submat(RIGHT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] / LEFT_ROI.area() / 255;
        double rightValue = Core.sumElems(right).val[0] / RIGHT_ROI.area() / 255;
        double middleValue = Core.sumElems(middle).val[0] / MIDDLE_ROI.area() / 255;

        left.release();
        right.release();
        middle.release();

//        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
//        telemetry.addData("Middle raw value", (int) Core.sumElems(middle).val[0]);
//        telemetry.addData("Right raw value", (int) Core.sumElems(right).val[0]);
        telemetry.addData("Right percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");
        telemetry.addData("Left percentage", Math.round(rightValue * 100) + "%");

        boolean stoneLeft = leftValue > PERCENT_COLOR_THRESHOLD;
        boolean stonemiddle = middleValue > PERCENT_COLOR_THRESHOLD;
        boolean stoneRight = rightValue > PERCENT_COLOR_THRESHOLD;

        if (stoneLeft) {
            location = Location.LEFT;
            telemetry.addData("Marker Location", "left");
             }
        else if (stoneRight) {
            location = Location.RIGHT;
            telemetry.addData("Marker Location", "right");
        }
        else if (stonemiddle){
            location = Location.MIDDLE;
            telemetry.addData("Marker Location", "middle");
        }
        telemetry.update();

        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);

        Scalar colorStone = new Scalar(255, 0, 0);
        Scalar colorSkystone = new Scalar(0, 255, 0);

        Imgproc.rectangle(mat, LEFT_ROI, location == Location.LEFT? colorSkystone:colorStone);
        Imgproc.rectangle(mat, MIDDLE_ROI, location == Location.MIDDLE? colorSkystone:colorStone);
        Imgproc.rectangle(mat, RIGHT_ROI, location == Location.RIGHT? colorSkystone:colorStone);

        return mat;
    }

    public Location getLocation() {
        return location;
    }
}