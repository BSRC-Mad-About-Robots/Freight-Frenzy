package org.firstinspires.ftc.teamcode;
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
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.imgcodecs.Imgcodecs;
import java.util.ArrayList;
import java.util.List;
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
import org.opencv.core.Point;

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

public class FinalOpenCV extends OpenCvPipeline {
    private int width; 
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
    boolean AlighnmentStart=false; 
    boolean TSE_detection=false;
    boolean left=false;
    boolean right=false;
    boolean middle=false;
    boolean blue=false;
    boolean red=false;
    double pixelshift=0;
    boolean detected=false;
    public FinalOpenCV(Telemetry t) {
        telemetry=t;
    }
    int length=0;

    @Override
    public Mat processFrame(Mat input) {

        
        
        Mat mat = new Mat();
        Mat mat1 = new Mat();
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        
        
        
        if(AlighnmentStart)
        {
            Scalar lowHSV = new Scalar(105,120,0);
        Scalar highHSV = new Scalar(123,355, 355); 
       if(blue)
    {
    lowHSV = new Scalar(105,120,0);
    highHSV = new Scalar(123,355, 355);
    }
    else if(red){
     lowHSV = new Scalar(0,80,0);
     highHSV = new Scalar(6,355, 355); 
    }
        
        Mat thresh = new Mat();

      
        Mat edges = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

    mat=thresh;
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

        Mat drawing = Mat.zeros(edges.size(), CvType.CV_8UC3);
        List<MatOfPoint> contoursPolyList = new ArrayList<>(contoursPoly.length);
        for (MatOfPoint2f poly : contoursPoly) {
            contoursPolyList.add(new MatOfPoint(poly.toArray()));
        }


        
        double left_x = 0.25 * width;
        double right_x = 0.75 * width;
        this.length=boundRect.length;

       
        this.x=0;
        this.height=0;
        this.width=0;
        this.y=0;
        this.maxwidth=7;
        this.maxheight=50;
        boolean heighttop=false;
        boolean heightbot=false;
        double heightbott=0.0;
        double ytop=0.0;
        this.detected=false;
        for (int i = 0; i != boundRect.length; i++) {
            if (boundRect[i].width >20 &&boundRect[i].width < 1000&&boundRect[i].height <1000&&boundRect[i].height>80&&boundRect[i].y<500)
            {
                this.detected=true;
            //     Point position1 = new Point(boundRect[i].x+boundRect[i].width,boundRect[i].y+boundRect[i].height);
            //                     Point position = new Point(boundRect[i].x,boundRect[i].y);
            //                     Point position2 = new Point(boundRect[i].x-290,boundRect[i].y+boundRect[i].height);
            //                     Point position3 = new Point(boundRect[i].x-290,124);
            //                     Point position4 = new Point(boundRect[i].x-50,124+boundRect[i].height);

            //                     Imgproc.rectangle(mat, boundRect[i], new Scalar(175, 255, 255), 4);
            // Imgproc.putText(mat, boundRect[i].x + "", position, Imgproc  .FONT_HERSHEY_SIMPLEX, 1, new Scalar(175, 200, 255), 5);
            //             Imgproc.putText(mat, boundRect[i].y + "", position1, Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(5, 200, 255), 5);
            // Imgproc.line(mat,position3,position2, new Scalar(175, 200, 255),12);
            //             Imgproc.line(mat,position2,position4, new Scalar(175, 200, 255),12);
            //             Imgproc.line(mat,position4,position3, new Scalar(175, 200, 255),12);

                if(boundRect[i].height>210)
                    {
               this.x =boundRect[i].x;
                this.y = boundRect[i].y;
                this.width = boundRect[i].width;
                this.height = (boundRect[i].height);
                    
                        heighttop=true;
                        ytop=boundRect[i].y;
                        
                    }
                   if(heighttop&&boundRect[i].y>200)
                    {
                         
                        this.height +=96;
                        heighttop=false;
                        heightbot=true;
                        heightbott=boundRect[i].height;
                        
                    }
            
                if (full_location != 1) {
                    telemetry.addData("Width:", boundRect[i].height);
                    telemetry.addData("Height:", this.height);
                    telemetry.addData("Y:", this.y);
                    telemetry.addData("x:", this.x);
                    telemetry.update();
                }
                

            }
            

        }
        
        }
        else if(TSE_detection)
{
    
    Scalar lowHSV = new Scalar(0,80,0);
        Scalar highHSV = new Scalar(8,355, 355); 
    if(blue)
    {
    lowHSV = new Scalar(90,80,0);
    highHSV = new Scalar(140,355, 355);
    pixelshift=0.0;
    }
    else if(red){
     lowHSV = new Scalar(0,10,0);
     highHSV = new Scalar(8,355, 355); 
    pixelshift=50;
    }
        Mat thresh = new Mat();
        Core.inRange(mat, lowHSV, highHSV, thresh);

        
        Mat edges = new Mat();
        
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(thresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
        Rect[] boundRect = new Rect[contours.size()];
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
        }
        double left_x = 495;
        double right_x = 650;
        double val_median=0.0;
        left = false;
        right = false; 
        middle=false;
        Imgproc.rectangle(
                mat,
                new Point(
                        1110+pixelshift,
                        450),
                new Point(
                        125+pixelshift,
                        960),
                new Scalar(0, 255, 0), 4);
                Imgproc.rectangle(
                mat,
                new Point(
                        290+pixelshift,
                        590),
                new Point(
                        170+pixelshift,
                        960),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                mat,
                new Point(
                        680+pixelshift,
                        570),
                new Point(
                        560+pixelshift,
                        960),
                new Scalar(0, 255, 0), 4);
        Imgproc.rectangle(
                mat,
                new Point(
                        950+pixelshift,
                        570),
                new Point(
                        1070+pixelshift,
                        960),
                new Scalar(0, 255, 0), 4);
        int counter=0;
        for (int i = 0; i != boundRect.length; i++) {
            
        if(boundRect[i].width>40&&boundRect[i].height>40&&boundRect[i].y>450&&boundRect[i].x>115&&boundRect[i].x<(1060+pixelshift))
        {
            val_median+=boundRect[i].x;
            Imgproc.rectangle(mat, boundRect[i], new Scalar(175,255,255), 4);
            counter++;
        }
    }
    val_median=val_median/counter;
    // telemetry.addData("y",val_median);
    //             telemetry.update();
     if(val_median>left_x&&val_median>right_x)
            {
                left=true;
                right=false;
                middle=false;
            }
            else if(val_median>left_x&&val_median<right_x)
            {
                middle=true;
                left=false;
                right=false;
            }
            else if(val_median<right_x)
            {
                right=true;
                middle=false;
                left=false;


            }
    
    Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2BGR);
    }
// Imgproc.cvtColor(mat, mat, Imgproc.COLOR_HSV2BGR);
    // Imgcodecs.imwrite(Environment.getExternalStorageDirectory().toString()+"/"+y+"shipping hub thresh.jpg",mat);
    return mat;
}
    public double getArea(){
        full_location=1;
        return this.height;}
    public double getPostitionY()
    {   full_location=1;
        if (this.detected){return (this.x);}
        else {return 0.0;}
    }
    public String getPostitionCapstone(){
    if(left) {return "left";}
    
    else if(middle) {return "middle";}
    
    else if(right) {return "right";}
    
    else {return "None";}
    
    }
}
