package org.firstinspires.ftc.teamcode.drive.Detectio;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

//for dashboard
@Config
public class DetectionPipeLine extends OpenCvPipeline {

    //backlog of frames to average out to reduce noise
    ArrayList<double[]> frameList;
    //these are public static to be tuned in dashboard
    public static double strictLowS = 100;
    public static double strictHighS = 255;

    public DetectionPipeLine() {
        frameList = new ArrayList<>();
    }

    public static double redLowHue = 0;
    public static double redHighHue = 15;

    @Override
    public Mat processFrame(Mat input) {
        Mat mat = new Mat();

        //mat turns into HSV value
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        if (mat.empty()) {
            return input;
        }

        Scalar highRedLowHSV = new Scalar(redLowHue, 40, 60);
        Scalar highRedHighHSV = new Scalar(redHighHue, 255, 255);
        Scalar lowRedLowHSV = new Scalar(165, 40, 60);
        Scalar lowRedHighHSV = new Scalar(180, 255, 255);

        Mat redThreshHigh = new Mat();
        Mat redThreshLow = new Mat();

        Core.inRange(mat, highRedLowHSV, highRedHighHSV, redThreshHigh);
        Core.inRange(mat, lowRedLowHSV, lowRedHighHSV, redThreshLow);
//        return redThreshHigh;

        Mat redMasked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
        Core.bitwise_and(mat, mat, redMasked, redThreshHigh);
        Core.bitwise_or(mat, redMasked, redMasked, redThreshLow);

        //calculate average HSV values of the white thresh values
        Scalar redAverage = Core.mean(redMasked, redThreshHigh);

        Mat redScaledMask = new Mat();
        //scale the average saturation to 150
        redMasked.convertTo(redScaledMask, -1, 150 / redAverage.val[1], 0);

        Mat redScaledThresh = new Mat();
        //you probably want to tune this
        Scalar strictLowHSV = new Scalar(0, strictLowS, 0);
        Scalar strictHighHSV = new Scalar(255, strictHighS, 255);

        Core.inRange(redScaledMask, strictLowHSV, strictHighHSV, redScaledThresh);

        Mat redEdges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(redScaledThresh, redEdges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> redContours = new ArrayList<>();
        Mat redHierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(redScaledThresh, redContours, redHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double curRedArea = 0;

        for (MatOfPoint contour : redContours)
        {
            curRedArea = Math.max(curRedArea, Imgproc.contourArea(contour));
            contour.release();
        }

        redArea = curRedArea;

        Scalar highGreenLowHSV = new Scalar(45, 40, 50);
        Scalar highGreenHighHSV = new Scalar(75, 255, 255);

        Mat greenThresh = new Mat();

        Core.inRange(mat, highGreenLowHSV, highGreenHighHSV, greenThresh);
//        return redThreshHigh;

        Mat greenMasked = new Mat();
//        //color the white portion of thresh in with HSV from mat
//        //output into masked
        Core.bitwise_and(mat, mat, greenMasked, greenThresh);

        //calculate average HSV values of the white thresh values
        Scalar greenAverage = Core.mean(greenMasked, greenThresh);

        Mat greenScaledMask = new Mat();
        //scale the average saturation to 150
        greenMasked.convertTo(greenScaledMask, -1, 150 / greenAverage.val[1], 0);


        Mat greenScaledThresh = new Mat();
        //you probably want to tune this
        strictLowHSV = new Scalar(0, strictLowS, 0); //strict lower bound HSV for yellow
        strictHighHSV = new Scalar(255, strictHighS, 255); //strict higher bound HSV for yellow
        //apply strict HSV filter onto scaledMask to get rid of any yellow other than pole
        Core.inRange(greenScaledMask, strictLowHSV, strictHighHSV, greenScaledThresh);

        Mat finalMask = new Mat();
        //color in scaledThresh with HSV, output into finalMask(only useful for showing result)(you can delete)
        Core.bitwise_and(mat, mat, finalMask, greenScaledThresh);
        Imgproc.cvtColor(finalMask, finalMask, Imgproc.COLOR_HSV2RGB);

        Mat greenEdges = new Mat();
        //detect edges(only useful for showing result)(you can delete)
        Imgproc.Canny(greenScaledThresh, greenEdges, 100, 200);

        //contours, apply post processing to information
        List<MatOfPoint> greenContours = new ArrayList<>();
        Mat greenHierarchy = new Mat();
        //find contours, input scaledThresh because it has hard edges
        Imgproc.findContours(greenScaledThresh, greenContours, greenHierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        double curGreenArea = 0;

        for (MatOfPoint contour : greenContours)
        {
            curGreenArea = Math.max(curGreenArea, Imgproc.contourArea(contour));
            contour.release();
        }

        if (frameList.size() > 5) {
            frameList.remove(0);
        }

        greenArea = curGreenArea;

        input.release();
        mat.release();

        redScaledThresh.copyTo(input);
        redScaledThresh.release();
        redScaledMask.release();
        redMasked.release();
        redEdges.release();
        redThreshHigh.release();
        redThreshLow.release();
        redHierarchy.release();

        greenScaledThresh.copyTo(input);
        greenScaledThresh.release();
        greenScaledMask.release();
        greenMasked.release();
        greenEdges.release();
        greenThresh.release();
        greenHierarchy.release();

//        finalMask.release();
//        return finalMask;
        //change the return to whatever mat you want
        //for example, if I want to look at the lenient thresh:
        // return thresh;
        // note that you must not do thresh.release() if you want to return thresh
        // you also need to release the input if you return thresh(release as much as possible)
        return finalMask;
//        return input;
    }

    public static double redArea = 0;
    public static double greenArea = 0;
}