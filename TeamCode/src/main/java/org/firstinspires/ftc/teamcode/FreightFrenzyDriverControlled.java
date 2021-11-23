/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *0
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOp Final Killabytez0︎︎︎", group="Iterative Opmode")

public class FreightFrenzyDriverControlled extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx spinner;
    private Servo wobbleGoalTres;
    private Servo wobbleGoalUno;
    private DcMotorEx frontleft = null;
    private DcMotorEx frontright = null;
    private DcMotorEx backleft = null;
    private DcMotorEx backright = null;
    private DcMotorEx outake2 = null;
    private DcMotorEx intake = null;
    private DcMotorEx arm = null;
    //private static final double velocity = 2000;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontleft  = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontright = hardwareMap.get(DcMotorEx.class, "frontright");
        backleft  = hardwareMap.get(DcMotorEx.class, "backleft");
        backright = hardwareMap.get(DcMotorEx.class, "backright");
        outake2 = hardwareMap.get(DcMotorEx.class, "outake2");
        //too1 = hardwareMap.get(DcMotor.class, "too1");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        wobbleGoalTres = hardwareMap.get(Servo.class, "wobbleGoalTres");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        spinner = hardwareMap.get(DcMotorEx.class, "spinner");
        wobbleGoalUno = hardwareMap.get(Servo.class, "wobbleGoalUno");

        //outake1.setDirection(DcMotor.Direction.REVERSE);
        outake2.setDirection(DcMotorEx.Direction.FORWARD);
        //too1.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);

        frontleft.setDirection(DcMotorEx.Direction.REVERSE);
        frontright.setDirection(DcMotorEx.Direction.FORWARD);
        backleft.setDirection(DcMotorEx.Direction.FORWARD);
        backright.setDirection(DcMotorEx.Direction.REVERSE);

//        frontleft.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
//        backleft.setDirection(DcMotor.Direction.REVERSE);
//        backright.setDirection(DcMotor.Direction.FORWARD);

        outake2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(50, 0, 2, 14));

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");

        outake2.setVelocity(0);

        wobbleGoalTres.setPosition(0.9);
        wobbleGoalUno.setPosition(0.2);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
//        outake2.setMotorEnable();
//        outake2.setVelocity(velocity);
        //.setPower(0.63569);

        double in  =  gamepad2.left_stick_y;
        double o = gamepad2.right_stick_y;
        double sped = gamepad2.right_stick_x;

        //too1.setPower(o/2);
        intake.setPower(in);
        spinner.setPower(in);
        arm.setPower(sped/2);

        if(gamepad2.a) {
            wobbleGoalTres.setPosition(0.65);

        }
        if(gamepad2.b) {
            wobbleGoalTres.setPosition(0.9);
        }

        if(gamepad2.x) {
            outake2.setVelocity(1240);
        }

        if(gamepad2.y) {
            outake2.setVelocity(0);
        }

        if(gamepad2.right_bumper) {
            wobbleGoalUno.setPosition(0.2);
        }

        if(gamepad2.left_bumper) {
            wobbleGoalUno.setPosition(0.8);
        }

        float y;
        float x;
        float z;
        y = gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        z = gamepad1.right_stick_y;
        frontright.setPower((y+x-z));
        frontleft.setPower((y-x+z));
        backright.setPower((y-x-z));
        backleft.setPower((y+x+z));

        // Send calculated powerea to wheels
        // Show the elapsed game time and wheel power.

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Velocity", ": " + outake2.getVelocity());
        telemetry.addData("Power", ": " + outake2.getPower());
        telemetry.addData("PIDFCoeffs", ":" + outake2.getPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER));
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
