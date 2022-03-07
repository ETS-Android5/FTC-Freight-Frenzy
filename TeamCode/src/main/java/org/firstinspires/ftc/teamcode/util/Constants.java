package org.firstinspires.ftc.teamcode.util;

import org.opencv.core.Point;
import org.opencv.core.Scalar;

public class Constants {
    public static double collectionBoxPosition=0.3;
    public static double carryingBoxPosition=0.5;
    public static double droppingBoxPosition=1.0;

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

    public static final int threshold = 100;

    public static final Point REGION1_TLEFT = new Point(160,75);
    public static final Point REGION1_BRIGHT = new Point(230,165);
    public static final Point REGION2_TLEFT = new Point(290,75);
    public static final Point REGION2_BRIGHT = new Point(360,165);
}
