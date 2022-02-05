package org.firstinspires.ftc.teamcode.drive.structure;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class ArmAssist extends LinearOpMode {
    public DcMotor arm;
    public DcMotor slider;
    public boolean manual=false;

    @Override
    public void runOpMode() throws InterruptedException {

    }

    HardwareMap hwMap = null;
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        slider = hwMap.get(DcMotor.class,"Slider");
        arm = hwMap.get(DcMotor.class, "Arm");


        slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        arm.setDirection(DcMotor.Direction.FORWARD);
        slider.setDirection(DcMotor.Direction.FORWARD);

        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        arm.setPower(0);
        slider.setPower(0);
    }


    public enum Armpos{
        UP,
        MEDIUM,
        DOWN,
        STOP,
        COMPLEX,

    }

    public void reset()
    {
        manual = false;
    }

    public Armpos pos = Armpos.STOP;

    public void Assist(){

        slider.setTargetPosition(1000);
        slider.setPower(0.5);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (slider.getCurrentPosition() < 1000 && manual == false) {

        }
        if (manual == false)
            slider.setPower(0);

        arm.setTargetPosition(-1076);
        arm.setPower(0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while (arm.isBusy() && manual == false) {

        }
        if (manual == false)
            arm.setPower(0);

        arm.setTargetPosition(0);
        arm.setPower(0.6);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(arm.isBusy() && manual==false){

        }
        arm.setPower(0);

        slider.setTargetPosition(10);
        slider.setPower(0.5);
        slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(slider.getCurrentPosition() > 10 &&  manual==false){

        }
        slider.setPower(0);
    }
        public void SlowAssist()
        {
            slider.setTargetPosition(1000);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slider.getCurrentPosition() < 1000 && manual == false) {

            }
            if (manual == false)
                slider.setPower(0);

            arm.setTargetPosition(-1076);
            arm.setPower(0.4);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (arm.isBusy() && manual == false) {

            }
            if (manual == false)
                arm.setPower(0);

            arm.setTargetPosition(0);
            arm.setPower(0.1);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(arm.isBusy() && manual==false){

            }
            arm.setPower(0);

            slider.setTargetPosition(10);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slider.getCurrentPosition() > 10 &&  manual==false){

            }
            slider.setPower(0);
        }
        public void Complex(){

            slider.setTargetPosition(850);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slider.getCurrentPosition() < 800){

            }
            slider.setPower(0);

            arm.setTargetPosition(-500);
            arm.setPower(0.6);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(arm.isBusy()){

            }
            arm.setPower(0);

            slider.setTargetPosition(10);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slider.getCurrentPosition() > 10){

            }
            slider.setPower(0);

            arm.setTargetPosition(-1076);
            arm.setPower(0.6);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(arm.isBusy()){

            }
            arm.setPower(0);

            slider.setTargetPosition(1000);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (slider.getCurrentPosition() < 1000){

            }
            slider.setPower(0);

            arm.setTargetPosition(0);
            arm.setPower(0.6);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(arm.isBusy()){

            }
            arm.setPower(0);

            slider.setTargetPosition(10);
            slider.setPower(0.5);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(slider.getCurrentPosition() > 10){

            }
            slider.setPower(0);

    }
}












































