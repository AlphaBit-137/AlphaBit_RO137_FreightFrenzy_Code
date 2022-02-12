package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class SliderStructure {

    public DcMotor slider;

    ElapsedTime runtime = new ElapsedTime();
    Positions SliderPosition = Positions.STOP;
    HardwareMap hwMap = null;

    public enum Positions{
        UP,
        DOWN,
        STOP,
        RESET
    }

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;
        // Define and Initialize Motors
        slider = hwMap.get(DcMotor.class,"Slider");
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slider.setDirection(DcMotor.Direction.FORWARD);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setPower(0);
    }


    public void SliderMovement(int position, double power)
    {
        slider.setTargetPosition(position);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slider.setPower(power);
    }

    public void update(){
        switch (SliderPosition){
            case UP:{
                slider.setPower(-0.5);
                break;
            }
            case DOWN:{
                slider.setPower(0.5);
                break;
            }
            case STOP:{
                slider.setPower(0);
                break;
            }
            case RESET:{
                slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    public void switchToSliderUp() {SliderPosition = Positions.UP;}

    public void switchToSliderDown() {SliderPosition = Positions.DOWN;}

    public void switchToSliderSTOP() {SliderPosition = Positions.STOP;}

    public void switchToSliderRESET() {SliderPosition = Positions.RESET;}

    public boolean SliderBUSY() {return slider.isBusy();}

}
