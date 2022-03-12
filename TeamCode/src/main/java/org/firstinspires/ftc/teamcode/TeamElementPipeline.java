package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

public class TeamElementPipeline extends OpenCvPipeline {
    Telemetry telemetry;
    Mat matY = new Mat();
    Mat region1;
    Mat region2;
    Mat region3;
    Mat Cb = new Mat();
    //0 is left, 1 is center, 2 is right
    int position, avg1, avg2, avg3;

    public TeamElementPipeline(Telemetry t){
        telemetry = t;
    }
    private void inputToCb(Mat input) {
        Imgproc.cvtColor(input, matY, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(matY, Cb, 2);
    }
    @Override
    public void init(Mat input) {
        inputToCb(input);
        region1 = Cb.submat(new Rect(REGION1_TLEFT, REGION1_BRIGHT));
        region2 = Cb.submat(new Rect(REGION2_TLEFT, REGION2_BRIGHT));
        region3 = Cb.submat(new Rect(REGION3_TLEFT, REGION3_BRIGHT));
    }
    @Override
    public Mat processFrame(Mat input){
        inputToCb(input);

        avg1 = (int) Core.mean(region1).val[0];
        avg2 = (int) Core.mean(region2).val[0];
        avg3 = (int) Core.mean(region3).val[0];

        //0 is left, 1 is center, 2 is right
        int diff1 = Math.abs(avg1-avg2)+Math.abs(avg1-avg3);
        int diff2 = Math.abs(avg1-avg2)+Math.abs(avg2-avg3);
        int diff3 = Math.abs(avg1-avg3)+Math.abs(avg2-avg3);

        int max = Math.max(Math.max(diff1, diff2), diff3);
        if (diff1==max){
            position = 0;
        }else if (diff2==max){
            position = 1;
        }else{
            position = 2;
        }
        Imgproc.cvtColor(input, matY, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.rectangle(input, REGION1_TLEFT, REGION1_BRIGHT, BLUE, 2);
        Imgproc.rectangle(input, REGION2_TLEFT, REGION2_BRIGHT, GREEN, 2);
        Imgproc.rectangle(input, REGION3_TLEFT, REGION3_BRIGHT, RED, 2);
        return input;
    }

    public int getAnalysis()
    {
        return position;
    }
    public int getAverage1(){
        return avg1;
    }
    public int getAverage2() {
        return avg2;
    }
    public int getAverage3(){return avg3;}
}
