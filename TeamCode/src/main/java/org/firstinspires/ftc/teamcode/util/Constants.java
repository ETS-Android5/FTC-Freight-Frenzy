package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class Constants {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688976378; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 6.5846456693; // X is the up and down direction
    public static double PARALLEL_Y = 3.1496062992; // Y is the strafe direction

    public static double PERPENDICULAR_X = 1.1811023622;
    public static double PERPENDICULAR_Y = -1.624015748;

    public static double collectionBoxPosition=0.3;
    public static double carryingBoxPosition=0.4;
    public static double droppingBoxPosition=1.0;

    public static Pose2d splineToShippingHubClose =new Pose2d(36,-18,7*Math.PI/4);
    public static double strafeCarousel=72;

    public static double elevKS = 0.268/2.54;
    public static double elevKV = 1.89/2.54;
    public static double elevKA = 0.243/2.54;
    public static double elevKG = 0.268/2.54;

    public static int targetPosition =0;
    public static int armHeight3Position=2700;
    public static int armHeight2Position=1800;
    public static int armHeight1Position=1500;
    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);

    public static final int difference = 10;

    public static final Point REGION1_TLEFT = new Point(155,45);
    public static final Point REGION1_BRIGHT = new Point(245,85);
    public static final Point REGION2_TLEFT = new Point(295,45);
    public static final Point REGION2_BRIGHT = new Point(385,85);

//    public static final Point REGION1_TLEFT = new Point(150,20);
//    public static final Point REGION1_BRIGHT = new Point(220,80);
//    public static final Point REGION2_TLEFT = new Point(280,20);
//    public static final Point REGION2_BRIGHT = new Point(350,80);

    public static enum VISUALIZATION_DETERMINED {
        LEFT, RIGHT, CENTER, UNDETERMINED
    }
}
