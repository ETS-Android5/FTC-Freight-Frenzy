package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="NorcalOpenCV", group="teleop")
public class NorcalCV extends LinearOpMode {
    OpenCvInternalCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException{
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        TeamElementPipeline element = new TeamElementPipeline(telemetry, true);
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

        while (!isStarted()){
            Constants.VISUALIZATION_DETERMINED pos = element.getAnalysis();
            if (pos== Constants.VISUALIZATION_DETERMINED.LEFT){
                telemetry.addData("Element pos: ", "left");
            }else if (pos== Constants.VISUALIZATION_DETERMINED.CENTER){
                telemetry.addData("Element pos: ", "center");
            }else{
                telemetry.addData("Element pos: ", "right");
            }
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            sleep(50);
        }
    }
}
