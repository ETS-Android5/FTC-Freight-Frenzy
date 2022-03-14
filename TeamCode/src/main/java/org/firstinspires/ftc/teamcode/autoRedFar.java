package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import static org.firstinspires.ftc.teamcode.util.Constants.*;


@Autonomous(name = "AutoRedFar", group = "teleop")
public class autoRedFar extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    OpenCvWebcam webcam;

    private SampleMecanumDrive drive;
    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotor carouselSpinner1, carouselSpinner2, intake;
    DcMotorEx armMotor;
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
    VISUALIZATION_DETERMINED teamMarkerState=VISUALIZATION_DETERMINED.UNDETERMINED;

    int position=0;
    Pose2d startPose=new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
//intake
        intake=hardwareMap.get(DcMotor.class, "intake");
//carouselSpinners
        carouselSpinner1 =hardwareMap.get(DcMotor.class, "carouselSpinner1");
        carouselSpinner2 =hardwareMap.get(DcMotor.class, "carouselSpinner2");
//arm
        armMotor=hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setTargetPosition(0);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setTargetPositionTolerance(20);
//box Servo
        box=hardwareMap.get(Servo.class, "box");

        initGyro();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        box.setPosition(carryingBoxPosition);

        Trajectory t1=drive.trajectoryBuilder(startPoseRedFar)
                .lineToLinearHeading(lineToShippingHubRedFar)
                .build();

        Trajectory t2=drive.trajectoryBuilder(t1.end())
                .lineToSplineHeading(startPoseRedFar)
                .build();

        Trajectory t3=drive.trajectoryBuilder(t2.end())
                .lineToConstantHeading(depoLineRedFar)
                .build();

        Trajectory t4=drive.trajectoryBuilder(t3.end())
                .lineToConstantHeading(startVectorRedFar)
                .build();

        Trajectory t5=drive.trajectoryBuilder(t4.end())
                .lineToSplineHeading(lineToShippingHubRedFar)
                .build();

        Trajectory t6=drive.trajectoryBuilder(t5.end())
                .lineToSplineHeading(startPoseRedFar)
                .build();

        Trajectory t7=drive.trajectoryBuilder(t6.end())
                .lineToConstantHeading(depoLineRedFar)
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamElementPipeline element = new TeamElementPipeline(telemetry, true);
        webcam.setPipeline(element);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("COMEON", "NOPE");
                telemetry.update();
            }
        });

        while (!isStarted()){
            teamMarkerState = element.getAnalysis();
            if (teamMarkerState== Constants.VISUALIZATION_DETERMINED.LEFT){
                telemetry.addData("Element pos: ", "left");
            }else if (teamMarkerState== Constants.VISUALIZATION_DETERMINED.CENTER){
                telemetry.addData("Element pos: ", "center");
            }else{
                telemetry.addData("Element pos: ", "right");
            }
            telemetry.update();
            sleep(50);
        }

        waitForStart();
        if ((opModeIsActive() && !isStopRequested())) {
            switch(teamMarkerState) {
                case LEFT: position=(armHeight1Position);
                case CENTER: position=(armHeight2Position);
                case RIGHT: position=(armHeight3Position);
                case UNDETERMINED: position=(armHeight3Position);
            }
            armMotor.setTargetPosition(position);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectory(t1);
            while(drive.isBusy() || armMotor.isBusy()) sleep(50);
            box.setPosition(droppingBoxPosition);
            armMotor.setTargetPosition(0);
            armMotor.setPower(-1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectory(t2);
            intake.setPower(0.5);
            drive.followTrajectory(t3);
            sleep(750);
            intake.setPower(0);
            drive.followTrajectory(t4);
            armMotor.setTargetPosition(position);
            armMotor.setPower(1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectory(t5);
            while(drive.isBusy() || armMotor.isBusy()) ;
            box.setPosition(droppingBoxPosition);
            armMotor.setTargetPosition(0);
            armMotor.setPower(-1);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            drive.followTrajectory(t6);
            intake.setPower(0.5);
            drive.followTrajectory(t7);
            sleep(750);
            intake.setPower(0);
        }
    }

    public void spinCarousel() {
        carouselSpinner1.setPower(0.70);
        carouselSpinner2.setPower(0.70);
    }

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
}