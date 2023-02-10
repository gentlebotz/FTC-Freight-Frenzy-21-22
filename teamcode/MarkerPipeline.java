package org.firstinspires.ftc.teamcode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.slf4j.IMarkerFactory;
import org.slf4j.Marker;

import java.util.ArrayList;
import java.util.List;

public class MarkerPipeline extends OpenCvPipeline {

    enum MarkerLocation{
        Left,
        Middle,
        Right,
        None
    }

    private int width;
    MarkerLocation location;
    Mat grey = new Mat();

    public MarkerPipeline(int width) {
        this.width = width;
    }

    Scalar Black = new Scalar(20,20,20);

    // Pink, the default color                         Y      Cr     Cb    (Do not change Y)
    public static Scalar scalarLowerYCrCb = new Scalar(  0.0, 150.0, 120.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 255.0, 255.0);


    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
        return grey;
    }

    public MarkerLocation getLocation() {
        return this.location;
    }
}
