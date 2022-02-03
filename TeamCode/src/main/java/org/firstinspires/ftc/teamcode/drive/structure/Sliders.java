package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sliders extends LinearOpMode {
    public DcMotor Slider = null;
    public boolean IsUp =true;
    public boolean IsDown= false;

    public SlidersModes RobotSlider = SlidersModes.STOP;

    @Override
    public void runOpMode() throws InterruptedException {

    }


    public enum SlidersModes {
        UP,
        DOWN,
        STOP,
    }

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        Slider = hwMap.get(DcMotor.class, "Slider");
        // Slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setPower(0);
    }

    public void update(){
        switch (RobotSlider){
            case UP:{

                Slider.setTargetPosition(110);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider.setPower(0.3);
                while(Slider.isBusy()){

                }
                Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Slider.setPower(0);

                break;
            }
            case DOWN:{

                Slider.setTargetPosition(-10);
                Slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slider.setPower(-0.3);
                while(Slider.isBusy()){

                }
                Slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Slider.setPower(0);

                break;
            }
            case STOP:{
                Slider.setPower(0);
                break;
            }
        }
    }

    public void switchToUP() {RobotSlider = SlidersModes.UP;}

    public void switchToDOWN() {RobotSlider = SlidersModes.DOWN;}

    public void switchToSTOP() {RobotSlider = SlidersModes.STOP;}

    public void RobotUp() {Slider.setPower(0.6);}
    public void RobotDown() {Slider.setPower(-0.6);}
    public void RobotStop() {Slider.setPower(0);}
//
}
