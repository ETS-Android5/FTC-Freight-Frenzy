package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name="AutoRoadrunner", group="TeleopRobot")
public class AutoFirstLevelRoadrunner extends LinearOpMode {

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
        frontleft  = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontright = hardwareMap.get(DcMotorEx.class, "rightFront");
        backleft  = hardwareMap.get(DcMotorEx.class, "leftRear");
        backright = hardwareMap.get(DcMotorEx.class, "rightRear");
        carouselSpinner=hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor=hardwareMap.get(DcMotor.class, "armMotor");
        intake=hardwareMap.get(CRServo.class, "intake");

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        Trajectory t1=drive.trajectoryBuilder(startPose).back(16).build();
        Trajectory t2=drive.trajectoryBuilder(t1.end()).forward(48).build();
        Trajectory t3=drive.trajectoryBuilder(t2.end()).splineToLinearHeading(new Pose2d(24, 0, Math.toRadians(90)), 0).build();
        Trajectory t4=drive.trajectoryBuilder(t3.end()).forward(40).build();
    }
}
