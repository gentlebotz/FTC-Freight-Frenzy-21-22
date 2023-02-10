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
import java.util.ArrayList;
import java.util.List;

public class PipeLine extends OpenCvPipeline {

    Scalar BLACK = new Scalar(30, 30, 30);

    public static Scalar scalarLowerYCrCb = new Scalar(0, 123, 124);
    public static Scalar scalarUpperYCrCb = new Scalar(70, 135, 135);

    public boolean error = false;
    public Exception debug;

    private int borderLeftX   = 0;   //amount of pixels from the left side of the cam to skip
    private int borderRightX  = 0;   //amount of pixels from the right of the cam to skip
    private int borderTopY    = 0;   //amount of pixels from the top of the cam to skip
    private int borderBottomY = 0;   //amount of pixels from the bottom of the cam to skip

    private int CAMERA_WIDTH = 640;
    private int CAMERA_HEIGHT = 360;

    private int loopcounter = 0;
    private int ploopcounter = 0;

    private Mat mat = new Mat();
    private Mat processed = new Mat();
    private Mat output = new Mat();

    private Rect maxRect = new Rect(600,1,1,1);
    private Rect rect = new Rect(600,1,1,1);

    private double maxArea = 0;
    private boolean first = false;

    private final Object sync = new Object();

    public void ConfigurePipeline(int borderLeftX, int borderRightX, int borderTopY, int borderBottomY, int CAMERA_WIDTH, int CAMERA_HEIGHT)
    {
        this.borderLeftX = borderLeftX;
        this.borderRightX = borderRightX;
        this.borderTopY = borderTopY;
        this.borderBottomY = borderBottomY;
    }

    public void ConfigureScalarLower(double Y, double Cr, double Cb) { scalarLowerYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarUpper(double Y, double Cr, double Cb) { scalarUpperYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarLower(int Y, int Cr, int Cb) { scalarLowerYCrCb = new Scalar(Y, Cr, Cb); }
    public void ConfigureScalarUpper(int Y, int Cr, int Cb) { scalarUpperYCrCb = new Scalar(Y, Cr, Cb); }

    @Override
    public Mat processFrame(Mat input)
    {
        CAMERA_WIDTH = input.width();
        CAMERA_HEIGHT = input.height();
        //output = input.clone();
        try{
            // Process Image
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2YCrCb);
            Core.inRange(mat, scalarLowerYCrCb, scalarUpperYCrCb, processed);

            // Remove Noise
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_OPEN, new Mat());
            Imgproc.morphologyEx(processed, processed, Imgproc.MORPH_CLOSE, new Mat());

            // GaussianBlur
            Imgproc.GaussianBlur(processed, processed, new Size(5.0, 15.0), 0.00);

            // Find Contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(processed, contours, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

            // Draw Contours
            Imgproc.drawContours(output, contours, -1, new Scalar(255, 0, 0));


            synchronized (sync) {
                // Loop Through Contours
                for (MatOfPoint contour : contours) {
                    Point[] contourArray = contour.toArray();

                    // Bound Rectangle if Contour is Large Enough
                    if (contourArray.length >= 15) {
                        MatOfPoint2f areaPoints = new MatOfPoint2f(contourArray);
                        rect = Imgproc.boundingRect(areaPoints);

                        // if rectangle is larger than previous cycle or if rectangle is not larger than previous 6 cycles > then replace
                        if (rect.area() > maxArea
                                && rect.x + rect.width / 2 > borderLeftX && rect.x + rect.width / 2 < CAMERA_WIDTH - borderRightX
                                && rect.y + rect.width / 2 > borderTopY && rect.y + rect.height / 2 < CAMERA_HEIGHT - borderBottomY
                                || loopcounter - ploopcounter > 6
                                && rect.x + rect.width / 2 > borderLeftX && rect.x + rect.width / 2 < CAMERA_WIDTH - borderRightX
                                && rect.y + rect.width / 2 > borderTopY && rect.y + rect.height / 2 < CAMERA_HEIGHT - borderBottomY) {
                            maxArea = rect.area();
                            maxRect = rect;
                            ploopcounter++;
                            loopcounter = ploopcounter;
                            first = true;
                        }
                        areaPoints.release();
                    }
                    contour.release();
                }
                mat.release();
                processed.release();
                if (contours.isEmpty()) {
                    maxRect = new Rect();
                }
            }
            // Draw Rectangles If Area Is At Least 500
            if (first && maxRect.area() > 500)
            {
                Imgproc.rectangle(output, maxRect, new Scalar(0, 255, 0), 2);
            }
            // Draw Borders
            Imgproc.rectangle(output, new Rect(borderLeftX, borderTopY, CAMERA_WIDTH - borderRightX - borderLeftX, CAMERA_HEIGHT - borderBottomY - borderTopY), BLACK, 2);
            // Display Data
            Imgproc.putText(output, "Area: " + getRectArea() + " Midpoint: " + getRectMidpointXY().x + " , " + getRectMidpointXY().y, new Point(5, CAMERA_HEIGHT - 5), 0, 0.6, new Scalar(255, 255, 255), 2);

            loopcounter++;

        }

        catch(Exception e){
            debug = e;
            error = true;
        }
        return input;

    }

    public int getRectHeight(){return maxRect.height;}
    public int getRectWidth(){ return maxRect.width; }
    public int getRectX(){ return maxRect.x; }
    public int getRectY(){ return maxRect.y; }
    public double getRectMidpointX(){ return getRectX() + (getRectWidth()/2.0); }
    public double getRectMidpointY(){ return getRectY() + (getRectHeight()/2.0); }
    public Point getRectMidpointXY(){ return new Point(getRectMidpointX(), getRectMidpointY());}
    public double getAspectRatio(){ return getRectArea()/(CAMERA_HEIGHT*CAMERA_WIDTH); }
    public double getRectArea(){ return maxRect.area(); }
}