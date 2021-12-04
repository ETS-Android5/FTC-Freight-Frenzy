package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.opmode.ManualFeedforwardTuner;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp Gay", group="Iterative Opmode")
public class TeleOp extends LinearOpMode {

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private SampleMecanumDrive drive;

    public DcMotor carouselSpinner;

    public DcMotor armMotor;

    public CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake=hardwareMap.get(CRServo.class, "intake");
        carouselSpinner=hardwareMap.get(DcMotor.class, "carouselSpinner");
        armMotor=hardwareMap.get(DcMotor.class, "armMotor");

        if (RUN_USING_ENCODER) {
//            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
//                    "when using the built-in drive motor velocity PID.");
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        NanoClock clock = NanoClock.system();

        telemetry.addLine("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();
        while(opModeIsActive()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            //intake
            intake.setPower(gamepad2.left_stick_y);
            //arm
            armMotor.setPower(gamepad2.right_stick_y*0.8);
            //carousel spinner
            if(gamepad2.dpad_right) carouselSpinner.setPower(1.00);
            if(gamepad2.dpad_left) carouselSpinner.setPower(-1.00);
            if(gamepad2.dpad_up) carouselSpinner.setPower(0.00);
        }
    }
}
