package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Sliders extends LinearOpMode {
    public DcMotor Slider = null;

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
        Slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slider.setPower(0);
    }

    public void update(){
        switch (RobotSlider){
            case UP:{
                Slider.setPower(-0.6);
                break;
            }
            case DOWN:{
                Slider.setPower(0.6);
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

}
