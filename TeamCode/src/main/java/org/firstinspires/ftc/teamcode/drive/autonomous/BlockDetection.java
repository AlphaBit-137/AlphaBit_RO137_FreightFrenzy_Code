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

        public Point LeftSquare1 = new Point(37, 231);
        public Point LeftSquare2 = new Point(67, 189);

        public Point RightSquare1 = new Point(245, 217);
        public Point RightSquare2 = new Point(289, 166);

        static final Rect LEFT_ROI = new Rect(
                new Point (0, 239),
                new Point (160, 0)
        );

        static final Rect RIGHT_ROI = new Rect(
                new Point (160, 0),
                new Point (319, 239)
        );

        Scalar BLACK = new Scalar(0, 0, 0);

        public void BlockDetection(Telemetry t) { telemetry=t;}

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_BGR2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
            Imgproc.threshold(Cb, tholdMat, 170, 255, Imgproc.THRESH_BINARY_INV);


            //   double[] bigSquarePointValues = tholdMat.get(BigSquarePointY, BigSquarePointX);
            //   double[] smallSquarePointValues = tholdMat.get(SmallSquarePointY, SmallSquarePointX);

            Mat Left = tholdMat.submat(LEFT_ROI);
            Mat Right = tholdMat.submat(RIGHT_ROI);

            double big_value = Core.sumElems(Left).val[0] / LEFT_ROI.area() / 255;
            double small_value = Core.sumElems(Right).val[0] / RIGHT_ROI.area() / 255;

            Left.release();
            Right.release();

            Left_percent = Math.round(big_value*100.0);
            Right_percent = Math.round(small_value*100.0);

            Imgproc.rectangle(input, LeftSquare1, LeftSquare2, BLACK);
            Imgproc.rectangle(input, RightSquare1, RightSquare2, BLACK);

            // telemetry.addData("Valoarea stanga:",(int)Core.sumElems(Left).val[0]);
            //telemetry.addData("Valoarea dreapta:", (int)Core.sumElems(Right).val[0]);
            //  telemetry.addData("Procentaj stanga:", Math.round(big_value*100));
            //telemetry.addData("Procentaj dreapta:", Math.round(small_value*100));
            // telemetry.update();
//             Imgproc.rectangle(tholdMat, LeftSquare1, LeftSquare2, BLACK);
//             Imgproc.rectangle(tholdMat, RightSquare1, RightSquare2, BLACK);

            return tholdMat;
        }
    }
}
