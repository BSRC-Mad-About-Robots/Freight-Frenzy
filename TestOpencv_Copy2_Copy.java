package org.firstinspires.ftc.teamcode.Tests;
import org.openftc.easyopencv.OpenCvPipeline;
import java.io.FileOutputStream;
import java.io.File;

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
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;
import java.util.List;
import android.graphics.Bitmap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class TestOpencv_Copy2_Copy extends OpenCvPipeline {

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
    public TestOpencv_Copy2_Copy(Telemetry t) {
        telemetry=t;
//        this.telemetry=telemetry;
    }
    int length=0;

    @Override
    public Mat processFrame(Mat input) {

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
                // We create a HSV range for yellow to detect regular stones
        // NOTE: In OpenCV's implementation,
        // Hue values are half the real value
        Scalar lowHSV = new Scalar(15,80,0); // lower bound HSV for yellow
        Scalar highHSV = new Scalar(26,355, 355);  // higher bound HSV for yellow
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


            if (boundRect[i].width >50 && boundRect[i].height >50&&boundRect[i].y>350)
            {
                Imgproc.rectangle(mat, boundRect[i], new Scalar(175, 255, 255), 4);
                this.x = boundRect[i].x;
                this.y = boundRect[i].y;
                this.width = boundRect[i].width;
                this.height = boundRect[i].height;

                
                    telemetry.addData("Width:", boundRect[i].width);
                    telemetry.addData("Height:", boundRect[i].height);
                    telemetry.addData("Y:", this.y);
                    telemetry.addData("x:", this.x);
                    telemetry.update();
                    
            //         Bitmap bitmap = Bitmap.createBitmap(1280, 960, Bitmap.Config.RGB_565);
            //         bitmap.copyPixelsFromBuffer(mat.getPixels());
            //         bitmap = createBitmap(bitmap, 0, 0, 1280, 960); //Cropped Bitmap to show only stones

            //         FileOutputStream out = null;
            //         File file = new File(path, "bitmapName");
            //         out = new FileOutputStream(file);
            //         bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
            // bitmap = createBitmap(bitmap, 0, 0, 1280, 960);//Cropped Bitmap to show only stones

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
        // Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2BGR);
                    
                    Imgcodecs.imwrite(Environment.getExternalStorageDirectory().toString()+"/"+y+"mat.jpg",mat);
                    Imgcodecs.imwrite(Environment.getExternalStorageDirectory().toString()+"/"+y+"thresh.jpg",thresh);

                    
        return mat;// return the mat with rectangles drawn
    }

//    public int detection(){
//        return this.length;
//
//    }
    public double getArea(){
        full_location=1;
        return this.height;}
    public double getPostitionY()
    {   full_location=1;
    // telemetry.addData("this.x", this.x);
    // telemetry.update();
        return (this.x);
    }
}
