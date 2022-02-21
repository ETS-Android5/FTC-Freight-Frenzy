package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "ObjectDetectionOpenCVuforia", group = "teleop")
public class ObjectDetectionAuto extends LinearOpMode {
    VuforiaLocalizer vuforia = null;
    OpenCvCamera camera;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        int[] viewportContainerIds = OpenCvCameraFactory.getInstance().splitLayoutForMultipleViewports(cameraMonitorViewId, 2, OpenCvCameraFactory.ViewportSplitMethod.VERTICALLY);

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(viewportContainerIds[0]);
        parameters.vuforiaLicenseKey = "AQdCgD3/////AAABmbb936QBr0JtryxLPUieTtUP2lCtx0XyKZmdtdtn8xWs3BUCGcsfcn5ZWYNNAYos6J0FMzH8Ic1a+jwqyR65mSpTR0fzQqrx4tSEx0SXJZu1796H4wFTXfGBtP0sIGpL8zgMgcocU4gJm3kkco+Rz4vxlPXAtRf5av0tyr/nYjxVKiyuwS0nRhUlHKd+d0zeL0vM6rHv6dONPV3lCbYRkrMOaFfwSZPGO05fNv+dr+4Idl0uoxNEI3WvylfKKiGFxgsYZRbrvtuKyj3WHYVo2U3rGfpM2EysGYI9ytFDVD2Q5dEWaHGGgKrz7RUo/cnXC6YM6JzhaXjlqHGjYXWVPrab4XTDCl+H/dDM/lsgNtNP";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        camera = OpenCvCameraFactory.getInstance().createVuforiaPassthrough(vuforia, parameters, viewportContainerIds[1]);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.setPipeline(new HopefulPipeline());

                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("BRUH COME ON", "YA WE AINT DOIN IT OUT HERE");
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Passthrough FPS", camera.getFps());
            telemetry.addData("Frame count", camera.getFrameCount());
            telemetry.update();

            sleep(100);
        }
    }
}
