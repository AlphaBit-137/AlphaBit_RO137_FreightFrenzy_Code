package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.ArmSlider;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;

@Autonomous
public class Auto extends LinearOpMode {
    EncoderMovement mvmt = new EncoderMovement();
    ColorSensor senzor;
    Carusel rotitza = new Carusel();
    BlockDetection bloc = new BlockDetection();
    ArmSlider slide = new ArmSlider();
    Intake inteic = new Intake();

    ElapsedTime runtime = new ElapsedTime();
    int caz=1;
    @Override
    public void runOpMode() throws InterruptedException {
        mvmt.init(hardwareMap);
        senzor = hardwareMap.get(ColorSensor.class,"sensor");
        rotitza.init(hardwareMap);
        bloc.init(hardwareMap);
        slide.init(hardwareMap);
        inteic.init(hardwareMap);
        while(!isStarted()){
            if(bloc.Right_percent <= 96 && bloc.Left_percent > bloc.Right_percent){
                caz = 1;
            }else if(bloc.Left_percent <= 96   && bloc.Right_percent > bloc.Left_percent){
                caz = 2;
            }else{
                caz = 3;
            }
            telemetry.addData("caz =",caz);
            telemetry.update();
        }
        if(opModeIsActive()){
            runtime.reset();
            mvmt.encoderDriveLinear(0.5,10,1,-1);
            sleep(10);
            mvmt.encoderDriveRotate(0.5,46.5,1,-1);
            sleep(10);
            mvmt.encoderDriveStrafe(0.5,10,1,-1);
            sleep(10);
            mvmt.encoderDriveLinear(0.5,33,2,-1);
            sleep(10);
            rotitza.switchToIN();
            rotitza.update();
            sleep(3200);
            rotitza.switchToSTOP();
            rotitza.update();
            mvmt.encoderDriveStrafe(0.5,10,1,1);
            sleep(10);
            mvmt.encoderDriveLinear(0.5,98,3,1);
            sleep(10);

            mvmt.encoderDriveRotate(0.5,46.7,2,1);
            sleep(10);


            if(caz == 3){
                mvmt.encoderDriveLinear(0.5,19,2,-1);
                slide.switchTo1();
                slide.update();
                sleep(10);
                mvmt.encoderDriveRotate(0.5,47,2,-1);
                sleep(10);
                mvmt.encoderDriveStrafe(0.3,45,3,-1);
                sleep(10);
            }else if(caz == 2){
                mvmt.encoderDriveLinear(0.5,28,2,-1);
                slide.switchTo1();
                slide.update();
                sleep(10);
                mvmt.encoderDriveRotate(0.5,47,2,-1);
                sleep(10);
                mvmt.encoderDriveStrafe(0.25,54,3,-1);
                sleep(10);
            }else{
                mvmt.encoderDriveLinear(0.5,35,2,-1);
                slide.switchTo1();
                slide.update();
                sleep(10);
                mvmt.encoderDriveLinear(0.2, 5, 3, 1);
                mvmt.encoderDriveRotate(0.5,47,2,-1);
                sleep(10);
                mvmt.encoderDriveStrafe(0.3,59 ,6,-1);
                sleep(10);
            }



            mvmt.encoderDriveLinear(0.5,130,5,1);
            /*if(runtime.seconds() < 15) {
                inteic.switchToOUT();
                inteic.update();
                while (senzor.alpha() < 140) {

                }

                inteic.switchToSTOP();
                inteic.update();
                sleep(100);
                mvmt.encoderDriveLinear(0.5, 135, 5, -1);
                sleep(100);
                mvmt.encoderDriveStrafe(0.5, 60, 3, 1);
                sleep(100);
                mvmt.encoderDriveRotate(0.5, 46.5, 2, 1);
                sleep(100);
               // slide.switchTo1();
               // slide.update();
                mvmt.encoderDriveRotate(0.5, 46.5, 2, -1);
                sleep(100);
                mvmt.encoderDriveStrafe(0.5, 60, 3, -1);
                sleep(100);
                mvmt.encoderDriveLinear(0.5, 130, 5, 1);
            }
*/
        }

    }
}
