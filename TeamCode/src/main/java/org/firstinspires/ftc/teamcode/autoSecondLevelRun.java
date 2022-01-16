package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class autoSecondLevelRun extends LinearOpMode {
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;

    DcMotorEx frontleft;
    DcMotorEx frontright;
    DcMotorEx backleft;
    DcMotorEx backright;
    DcMotor carouselSpinner, armMotor;
    CRServo intake;

    Pose2d startPose=new Pose2d();

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontright = hardwareMap.get(DcMotorEx.class, "rightFront");
        backleft = hardwareMap.get(DcMotorEx.class, "leftRear");
        backright = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselSpinner = hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intake = hardwareMap.get(CRServo.class, "intake");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        if ((opModeIsActive() && !isStopRequested())) {

        }
    }
}
