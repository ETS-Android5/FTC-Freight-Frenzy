package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class autoSecondLevelRun extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive ;
    private double collectionBoxPosition=0.0;
    private double carryingBoxPosition=0.2;
    private double droppingBoxPosition=0.6;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotor carouselSpinner, armMotor, intake;
    Servo box;

    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gearratio = 20;
    Double diameter = 4.0;
    Double cpi = (cpr * gearratio)/(Math.PI * diameter);
    Double bias = 0.8;
    Double meccyBias = 0.9; //adjust strafing
    Double conversion = cpi * bias;
    Boolean exit = false;
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    Pose2d startPose=new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselSpinner = hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(DcMotor.class, "intake");
        box = hardwareMap.get(Servo.class, "box");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        Trajectory t1=drive.trajectoryBuilder(new Pose2d(0,0,Math.toRadians(-90))).back(16).build();
        Trajectory t2=drive.trajectoryBuilder(t1.end()).forward(46).strafeLeft(24).build();
        Trajectory t3=drive.trajectoryBuilder(drive.getPoseEstimate()).forward(36).build();
        Trajectory t4 = drive.trajectoryBuilder(drive.getPoseEstimate()).forward(50).build();

        box.setPosition(carryingBoxPosition);

        waitForStart();
        if ((opModeIsActive() && !isStopRequested())) {
            drive.turn(Math.toRadians(-90));
            drive.followTrajectory(t1);
            carouselSpinner.setPower(0.2);
            sleep(4000);
            carouselSpinner.setPower(0);
            drive.followTrajectory(t2);
            drive.turn(Math.toRadians(-90));
            //move arm down
            armMotor.setPower(0.4);
            sleep(1000);
            armMotor.setPower(0.0);
            box.setPosition(droppingBoxPosition);
            drive.followTrajectory(t3);
            drive.turn(Math.toRadians(90));
            drive.followTrajectory(t4);
        }
    }
    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - move);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + move);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() - move);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() + move);
        //
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
        //
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){
            if (exit){
                rightFront.setPower(0);
                leftFront.setPower(0);
                rightRear.setPower(0);
                leftRear.setPower(0);
                return;
            }
        }
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        return;
    }
    //
    /*
 Gyro to turn x degrees
     */
    public void turnWithGyro(double degrees, double speedDirection){
        //Init
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double yaw = -angles.firstAngle;
        telemetry.addData("Speed Direction", speedDirection);
        telemetry.addData("Yaw", yaw);
        telemetry.update();
        //
        telemetry.addData("stuff", speedDirection);
        telemetry.update();
        //
        double first;
        double second;
        //
        if (speedDirection > 0){//set target positions
            if (degrees > 10){
                first = (degrees - 10) + devertify(yaw);
                second = degrees + devertify(yaw);
            }else{
                first = devertify(yaw);
                second = degrees + devertify(yaw);
            }
        }else{
            if (degrees > 10){
                first = devertify(-(degrees - 10) + devertify(yaw));
                second = devertify(-degrees + devertify(yaw));
            }else{
                first = devertify(yaw);
                second = devertify(-degrees + devertify(yaw));
            }
            //
        }
        //
        Double firsta = convertify(first - 5);//175
        Double firstb = convertify(first + 5);//-175
        //
        turnWithEncoder(speedDirection);
        //
        if (Math.abs(firsta - firstb) < 11) {
            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }else{
            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("first before", first);
                telemetry.addData("first after", convertify(first));
                telemetry.update();
            }
        }
        Double seconda = convertify(second - 5);//175
        Double secondb = convertify(second + 5);//-175
        //
        turnWithEncoder(speedDirection / 3);
        //
        if (Math.abs(seconda - secondb) < 11) {
            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
                yaw = -angles.firstAngle;
                telemetry.addData("Position", yaw);
                telemetry.addData("second before", second);
                telemetry.addData("second after", convertify(second));
                telemetry.update();
            }
            leftFront.setPower(0);
            rightFront.setPower(0);
            leftRear.setPower(0);
            rightRear.setPower(0);
        }
        //
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    //
    /*
    Encoders for strafing
    Negative = left strafe
     */
    public void strafeToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches * cpi * meccyBias));
        //
        leftRear.setTargetPosition(leftRear.getCurrentPosition() - move);
        leftFront.setTargetPosition(leftFront.getCurrentPosition() + move);
        rightRear.setTargetPosition(rightRear.getCurrentPosition() + move);
        rightFront.setTargetPosition(rightFront.getCurrentPosition() - move);
        //
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        leftFront.setPower(speed);
        leftRear.setPower(speed);
        rightFront.setPower(speed);
        rightRear.setPower(speed);
        //
        while (leftFront.isBusy() && rightFront.isBusy() && leftRear.isBusy() && rightRear.isBusy()){}
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        return;
    }

    public void waitForStartify(){
        waitForStart();
    }
    //

    public double devertify(double degrees){
        if (degrees < 0){
            degrees = degrees + 360;
        }
        return degrees;
    }
    public double convertify(double degrees){
        if (degrees > 179){
            degrees = -(360 - degrees);
        } else if(degrees < -180){
            degrees = 360 + degrees;
        } else if(degrees > 360){
            degrees = degrees - 360;
        }
        return degrees;
    }
    //

    public void initGyro(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
    //

    public void turnWithEncoder(double input){
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        leftFront.setPower(input);
        rightRear.setPower(input);
        rightFront.setPower(-input);
        rightRear.setPower(-input);
    }
}
