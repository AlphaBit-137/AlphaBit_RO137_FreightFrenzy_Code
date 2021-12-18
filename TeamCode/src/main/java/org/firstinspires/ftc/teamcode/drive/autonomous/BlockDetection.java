package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class BlockDetection extends LinearOpMode {
    public static Telemetry telemetry;
    private ElapsedTime runtime = new ElapsedTime();
    public static double Left_percent, Right_percent;

    OpenCvWebcam webcam;
    HardwareMap hwMap = null;

    @Override
    public void runOpMode() throws InterruptedException {

    }
    public void init(HardwareMap ahwMAp){
        hwMap = ahwMAp;
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDevice();
        webcam.setPipeline(new BlockDetection.StagePipeline());
        webcam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
    }
    public void stopCamera(HardwareMap ahwMap){
        webcam.stopStreaming();
    }

    static class StagePipeline extends OpenCvPipeline {

        // Working Mat variables
        Mat YCrCb = new Mat(); // This will store the whole YCrCb channel
        Mat Cb = new Mat(); // This will store the Cb Channel (part from YCrCb)
        Mat tholdMat = new Mat(); // This will store the threshold

        public Point LeftSquare1 = new Point(150, 155);
        public Point LeftSquare2 = new Point(190, 110);

        public Point RightSquare1 = new Point(75, 155);
        public Point RightSquare2 = new Point(115, 110);

        static final Rect LEFT_ROI = new Rect(
                new Point (150, 155),
                new Point (190, 110)
        );

        static final Rect RIGHT_ROI = new Rect(
                new Point (75, 155),
                new Point (115, 110)
        );

        Scalar BLACK = new Scalar(0, 0, 0);

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.threshold(Cb, tholdMat, 87, 255, Imgproc.THRESH_BINARY_INV);


            //   double[] bigSquarePointValues = tholdMat.get(BigSquarePointY, BigSquarePointX);
            //   double[] smallSquarePointValues = tholdMat.get(SmallSquarePointY, SmallSquarePointX);

            //   ring_4 = (int) bigSquarePointValues[0];
            //   ring_1 = (int) smallSquarePointValues[0];

            Mat Left = tholdMat.submat(LEFT_ROI);
            Mat Right = tholdMat.submat(RIGHT_ROI);

            double big_value = Core.sumElems(Left).val[0] / LEFT_ROI.area() / 255;
            double small_value = Core.sumElems(Right).val[0] / RIGHT_ROI.area() / 255;

            Left.release();
            Right.release();

            Left_percent = Math.round(big_value*100);
            Right_percent = Math.round(small_value*100);

            // Imgproc.rectangle(input, BigSquare1, BigSquare2, BLACK);
            // Imgproc.rectangle(input, SmallSquare1, SmallSquare2, BLACK);

            telemetry.addData("Valoarea stanga:", Left_percent);
            telemetry.addData("Valoarea dreapta:", Right_percent);
            telemetry.update();

            return input;
        }
    }

}
