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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.util.Constants.*;


@Autonomous(name = "AutoFirstLevelRunRedFar", group = "teleop")
public class autoFirstLevelRunRedFar extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();


    private SampleMecanumDrive drive ;
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
    VISUALIZATION_DETERMINED teamMarkerState;

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

        Trajectory t1=drive.trajectoryBuilder(new Pose2d(0, 0,0))
                .splineToSplineHeading(splineToShippingHubClose, 0.7)
                .build();

        Trajectory t2=drive.trajectoryBuilder(t1.end())
                .strafeLeft(strafeCarousel)
                .build();

        waitForStart();
        if ((opModeIsActive() && !isStopRequested())) {
            drive.followTrajectory(t1);
            switch(teamMarkerState) {
                case LEFT: armMotor.setTargetPosition(armHeight1Position);
                case CENTER: armMotor.setTargetPosition(armHeight2Position);
                case RIGHT: armMotor.setTargetPosition(armHeight3Position);
                case UNDETERMINED: armMotor.setTargetPosition(armHeight3Position);
            }
            spinCarousel();

        }
    }

    public void spinCarousel() {
        carouselSpinner1.setPower(0.70);
        carouselSpinner2.setPower(-0.70);
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
