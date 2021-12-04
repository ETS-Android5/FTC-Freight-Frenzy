package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="FirstLevelRoadrunner", group="Auto")
public class AutoFirstLevelRoadrunner extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;

    DcMotorEx frontleft;
    DcMotorEx frontright;
    DcMotorEx backleft;
    DcMotorEx backright;
    DcMotor carouselSpinner, armMotor;
    CRServo intake;

    Pose2d startPose=new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        frontleft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontright = hardwareMap.get(DcMotorEx.class, "rightFront");
        backleft  = hardwareMap.get(DcMotorEx.class, "leftRear");
        backright = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselSpinner=hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor=hardwareMap.get(DcMotor.class, "armMotor");
        intake=hardwareMap.get(CRServo.class, "intake");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        waitForStart();
        if(opModeIsActive()) {
            drive.turn(Math.toRadians(-90));
            drive.trajectoryBuilder(startPose).forward(16);
            carouselSpinner.setPower(0.4);
            sleep(2000);
            carouselSpinner.setPower(0);
            drive.trajectoryBuilder(startPose).back(-48);
            sleep(1000);
            drive.turn(Math.toRadians(90));
            drive.trajectoryBuilder(startPose).splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), 0);
            //move arm down
            armMotor.setPower(0.6);
            sleep(750);
            intake.setPower(0.0);
            sleep(750);
            armMotor.setPower(0);
            drive.turn(Math.toRadians(135));
            drive.trajectoryBuilder(startPose).forward(40);
        }
    }
}
