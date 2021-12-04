package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Encoder Test", group="OpMode")
public class EncoderArmTest extends LinearOpMode {
    public DcMotor armMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                break;
            }
            armMotor.setTargetPosition(armMotor.getCurrentPosition()+10);
            telemetry.addData("Encoder count: ", armMotor.getCurrentPosition());
            telemetry.update();
            telemetry.clearAll();
        }
    }
}
