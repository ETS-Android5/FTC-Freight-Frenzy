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

    public CRServo intake;

    @Override
    public void runOpMode() throws InterruptedException {
        intake=hardwareMap.get(CRServo.class, "intake");
        if (RUN_USING_ENCODER) {
//            RobotLog.setGlobalErrorMsg("Feedforward constants usually don't need to be tuned " +
//                    "when using the built-in drive motor velocity PID.");
        }

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new SampleMecanumDrive(hardwareMap);

        intake.setPower(0.5*gamepad2.left_stick_y+0.5);

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
        }
    }
}
