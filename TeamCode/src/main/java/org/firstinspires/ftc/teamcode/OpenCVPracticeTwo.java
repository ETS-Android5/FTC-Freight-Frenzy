package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="TryingCV", group="teleop")
public class OpenCVPracticeTwo extends LinearOpMode {
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        TeamElementPipeline element = new TeamElementPipeline(telemetry);
        camera.setPipeline(element);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
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
            int pos = element.getAnalysis();
            if (pos==0){
                telemetry.addData("Element pos: ", "left");
            }else if (pos==1){
                telemetry.addData("Element pos: ", "right");
            }else{
                telemetry.addData("Element pos: ", "not in frame");
            }
            telemetry.update();
            sleep(50);
        }
    }
}
