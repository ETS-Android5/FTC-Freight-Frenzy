package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class autoThirdLevelRun extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftRear;
    DcMotorEx rightRear;
    DcMotor carouselSpinner, armMotor;
    CRServo intake;

    Pose2d startPose=new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselSpinner = hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intake");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if ((opModeIsActive() && !isStopRequested())) {

        }
    }
}