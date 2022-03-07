package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.TeamElementPipeline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="TryingCV", group="teleop")
public class OpenCVPracticeTwo extends LinearOpMode {
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        TeamElementPipeline element = new TeamElementPipeline(telemetry);
        webcam.setPipeline(element);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("COMEON", "NOPE");
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("Average for left: ", element.getAverage1());
            telemetry.addData("Average for right: ", element.getAverage2());
            int col = element.getAnalysis();
            String curColor = "red";
            if (col==1){
                curColor = "yellow";
            }
            int pos = element.getAnalysis();
            if (pos==0){
                telemetry.addData("Element pos: ", "left");
            }else if (pos==1){
                telemetry.addData("Element pos: ", "right");
            }else{
                telemetry.addData("Element pos: ", "not in frame");
            }
            telemetry.addData("Color: ", curColor);
            telemetry.update();
            sleep(50);
        }
    }
}
