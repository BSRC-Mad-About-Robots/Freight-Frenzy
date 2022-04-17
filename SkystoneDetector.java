package org.firstinspires.ftc.teamcode;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class SkystoneDetector extends OpenCvPipeline {

    private int width; // width of the image
    Telemetry telemetry;
    Mat mat =new Mat();
    int x;
    int y;
    
    int height;
    int maxwidth =0;
    int maxheight =0;
    int full_location=0;
    private int Integer;
    private float floating;

    /**
     *
//     * @param width The width of the image (check your camera)
     */
    public SkystoneDetector(Telemetry t) {
        telemetry=t;
//        this.telemetry=telemetry;
    }
    int length=0;
        boolean left=false;
        boolean right=false;
        boolean middle=false;
    @Override
    public Mat processFrame(Mat input)
    {
        
        
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
        // the function will return a matrix to be drawn on your phone's screen

        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking

        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
            // location = SkystoneLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(28,30,0); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(100,355, 355); // higher bound HSV for yellow
        Mat thresh = new Mat();

        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        Mat edges = new Mat();
        Imgproc.Canny(thresh, edges, 100, 300);

        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
        // Oftentimes the edges are disconnected. findContours connects these edges.
        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }

        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 360;
        double right_x = 780;
        
        
        
        
            // sleep(300);
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well
            Imgproc.rectangle(
                    mat,
                    new Point(
                            1250,
                            450),
                    new Point(
                            125,
                            640),
                    new Scalar(0, 255, 0), 4);
                    Imgproc.rectangle(
                    mat,
                    new Point(
                            320,
                            570),
                    new Point(
                            200,
                            640),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(
                    mat,
                    new Point(
                            680,
                            570),
                    new Point(
                            560,
                            640),
                    new Scalar(0, 255, 0), 4);
            Imgproc.rectangle(
                    mat,
                    new Point(
                            950,
                            570),
                    new Point(
                            1070,
                            640),
                    new Scalar(0, 255, 0), 4);
            
        for (int i = 0; i != boundRect.length; i++) {
            
        if(boundRect[i].width>50&&boundRect[i].height>60&&boundRect[i].y>450&&boundRect[i].x>115&&boundRect[i].x<1230&&(!(boundRect[i].y>640)))
        {
            
            // telemetry.addData("boundrect[i]",boundRect[i].height);
            // telemetry.addData("boundrect",boundRect[i]);
            
            
            Imgproc.rectangle(mat, boundRect[i], new Scalar(175, 255, 255), 4);
            if(boundRect[i].x<left_x)
            {
                left=true;
                // telemetry.addData("left",left);
                // telemetry.update();
            }
            else if(boundRect[i].x>left_x&&boundRect[i].x<right_x)
            {
                middle=true;
            }
            else if(boundRect[i].x>right_x)
            {
                right=true;
            }
        }
        
        
        
    }
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

    return thresh;
    
}

        // if there is no yellow regions on a side
        // that side should be a Skystone
        

         // return the mat with rectangles drawn



//    public int detection(){
//        return this.length;
//
//    }
    public double getArea(){return this.width*this.height;}
    public String getPostitionCapstone()
    {   if(left)
    {
        return "left";
    }
    else if(middle)
    {
        return "middle";
    }
    else if(right)
    {
        
        // telemetry.addData("left",left);
        // telemetry.update();
        return "right";
    }
    else
    {
        return "None";
    }
    
    }
}
