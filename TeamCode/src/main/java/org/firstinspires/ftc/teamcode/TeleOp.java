package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ElevatorFeedforward;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp G", group="Iterative Opmode")
public class TeleOp extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;

    public DcMotor carouselSpinner1;

    public DcMotor carouselSpinner2;

    public DcMotorEx armMotor;

    public DcMotor intake;

    public ElevatorFeedforward kj=new ElevatorFeedforward(elevKS,elevKV,elevKA,elevKG);

    Servo box;

    @Override
    public void runOpMode() throws InterruptedException {
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

        if (RUN_USING_ENCODER) {
            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
                    "when using the built-in drive motor velocity PID.");
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        NanoClock clock = NanoClock.system();

        waitForStart();
        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.right_stick_x,
                            -gamepad1.left_stick_x
                    )
            );
            //arm
            if(gamepad2.right_stick_y!=0) {
                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                armMotor.setPower(-gamepad2.right_stick_y);
            }
            else if(gamepad2.b) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPosition=armHeight3Position;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(targetPosition <armMotor.getCurrentPosition()?1:-1);
            }
            else if(gamepad2.a) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPosition =armHeight2Position;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(targetPosition <armMotor.getCurrentPosition()?1:-1);
            }
            else if(gamepad2.x) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPosition =armHeight1Position;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(targetPosition <armMotor.getCurrentPosition()?1:-1);
            }
            else if(gamepad2.y) {
                armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                targetPosition =0;
                armMotor.setTargetPosition(targetPosition);
                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(targetPosition <armMotor.getCurrentPosition()?1:-1);
            }
            else if(gamepad2.right_stick_y==0&&armMotor.getMode()== DcMotor.RunMode.RUN_WITHOUT_ENCODER) armMotor.setPower(0);

            //carousel spinner
            if(gamepad2.right_trigger>0) {
                carouselSpinner1.setPower(0.70);
                carouselSpinner2.setPower(-0.70);
            }
            else if(gamepad2.left_trigger>0) {
                carouselSpinner1.setPower(-0.70);
                carouselSpinner2.setPower(0.70);
            }
            else {
                carouselSpinner1.setPower(0.00);
                carouselSpinner2.setPower(0.00);
            }
            //intake
            intake.setPower(-gamepad2.left_stick_y/2);
            //box servo
            if(gamepad2.dpad_down) box.setPosition(collectionBoxPosition);
            if(gamepad2.dpad_right) box.setPosition(carryingBoxPosition);
            if(gamepad2.dpad_up) box.setPosition(droppingBoxPosition);

            if(armMotor.getCurrentPosition()<20) {
                box.setPosition(collectionBoxPosition);
            }
            else if(armMotor.getCurrentPosition()<2480) {
                box.setPosition(carryingBoxPosition);
            }

            telemetry.addData("Current Position: ", armMotor.getCurrentPosition());
            telemetry.addData("Target Position: ", armMotor.getTargetPosition());
            telemetry.update();


//            if(gamepad2.right_stick_y!=0) box.setPosition(carryingBoxPosition);
        }
    }
}
