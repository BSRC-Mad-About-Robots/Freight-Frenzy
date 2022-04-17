package org.firstinspires.ftc.teamcode.Tests;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
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


import java.util.ArrayList;
import java.util.List;


public class OpenCV_Auto_Blue extends OpenCvPipeline
{
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
    boolean isAlignment;
    int length=0;
        boolean left=false;
        boolean right=false;
        boolean middle=false;
     Mat returnMat = new Mat();
    public OpenCV_Auto_Blue(Telemetry t) { telemetry=t; }
    @Override
    public Mat processFrame(Mat input) 
    {
        if (isAlignment)
        {
        // "Mat" stands for matrix, which is basically the image that the detector will process
        // the input matrix is the image coming from the camera
            // the function will return a matrix to be draw`11`
        // The detector detects regular stones. The camera fits two stones.
        // If it finds one regular stone then the other must be the skystone.
        // If both are regular stones, it returns NONE to tell the robot to keep looking
//        Imgproc.GaussianBlur();
        // Make a working copy of the input matrix in HSV
        Mat mat = new Mat();
        Mat mat1 = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        Mat dst = new Mat(mat.rows(),   mat.cols(),mat.type());
        // Imgproc.GaussianBlur(mat,dst,new Size(5,5),1);
//        Imgproc.(mat, input,new Size(3,3));
        // if something is wrong, we assume there's no skystone
        if (mat.empty()) {
//            location = SkystoneLocation.NONE;
            return input;
        }

        // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
            Scalar lowHSV = new Scalar(105,120,0); // lower bound HSV for yellow
            Scalar highHSV = new Scalar(123,355, 355);  // higher bound HSV for yellow
        Mat thresh = new Mat();
        // We'll get a black and white image. The white regions represent the regular stones.
        // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range
        Mat edges = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        // Use Canny Edge Detection to find edges
        // you might have to tune the thresholds for hysteresis
        // Imgproc.Canny(mat, edges, 300, 400);
//        // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
//        // Oftentimes the edges are disconnected. findContours connects these edges.
//        // We then find the bounding rectangles of those contours
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];


        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();


            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 1, false);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));

        }
//        JsonArray cannyOutput;
        Mat drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }


        // Iterate and check whether the bounding boxes
        // cover left and/or right side of the image
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        boolean left = false; // true if regular stone found on the left side
        boolean right = false; // "" "" on the right side
        this.length=boundRect.length;
//        if (length>=0)
//        {
//            telemetry.addData(String.format("  right,bottom (%d)", length),0);
//            telemetry.update();
//        }


//        boundRect.();
        this.x=0;
        this.height=0;
        this.width=0;
        this.y=0;
        this.maxwidth=7;
        this.maxheight=50;
//        this.full_location=0;
        for (int i = 0; i != boundRect.length; i++) {
            // draw red bounding rectangles on mat
            // the mat has been converted to HSV so we need to use HSV as well

//            Imgproc.drawContours(drawing, contoursPolyList, i, new Scalar(175, 255, 255));

//            telemetry.addData("X",boundRect[i].x);
//            telemetry.update();
//            telemetry.addData("",boundRect[i].width);
//            telemetry.update();
//            this.width=boundRect[i].width;
//            this.height=boundRect[i].height;
//            if(this.maxwidth < this.width && maxheight < height)
//            {
//                this.x=boundRect[i].x;
//                this.y=boundRect[i].y;
//                this.maxwidth =boundRect[i].width;
//                this.maxheight =boundRect[i].height;
//                this.full_location=i;
//
//
//            }

//            telemetry.addData("Height:",boundRect[i].height);
//            telemetry.update();


            if (boundRect[i].width >20 && boundRect[i].height >190 &&boundRect[i].width < 1000&&boundRect[i].height <1000)
            {
                Imgproc.rectangle(mat, boundRect[i], new Scalar(175, 255, 255), 4);
                this.x = boundRect[i].x;
                this.y = boundRect[i].y;
                this.width = boundRect[i].width;
                this.height = boundRect[i].height;

                if (full_location != 1) {
                    telemetry.addData("Width:", boundRect[i].width);
                    telemetry.addData("Height:", boundRect[i].height);
                    telemetry.addData("Y:", this.y);
                    telemetry.addData("x:", this.x);
                    telemetry.update();
                }

            }
 
//            telemetry.addData("Area:",width1*height);
//            telemetry.update();
//            if (i==1){
//                telemetry.addData(String.format("label (%d)", i),i);
//                telemetry.update();
//                sleep(250);
//            }
        }


        // if there is no yellow regions on a side
        // that side should be a Skystone

        // if both are true, then there's no Skystone in front.
        // since our team's camera can only detect two at a time
        // we will need to scan the next 2 stones
//        else location = SkystoneLocation.NONE;

//        return mat;`
//        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
        returnMat = mat;// return the mat with rectangles drawn
    }
    else
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
                right=false;
                middle=false;

                // telemetry.addData("left",left);
                // telemetry.update();
            }
            else if(boundRect[i].x>left_x&&boundRect[i].x<right_x)
            {
                middle=true;
                left=false;
                right=false;
            }
            else if(boundRect[i].x>right_x)
            {
                right=true;
                middle=false;
                left=false;


            }
        }
        
        
        
    }
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);

    returnMat = thresh; 
    }
    return returnMat;
    }
//    public int detection(){
//        return this.length;
//
//    }
    public double getAreaCap(){
        full_location=1;
        return this.height;}
    public double getPostitionY()
    {
        full_location=1;
    // telemetry.addData("this.x", this.x);
    // telemetry.update();
        return (this.x);
    }
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