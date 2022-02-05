package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmSlider extends LinearOpMode {

    public DcMotor arm;
    public DcMotor slider;
    ElapsedTime runtime = new ElapsedTime();
    public Armpos RobotPos = Armpos.STOP;
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
        STRONG,
        WEAK,
        STOP,
    }

    @Override
    public void runOpMode() throws InterruptedException {
        while(opModeIsActive()){
            switch(RobotPos){
                case STRONG:{
                    slider.setTargetPosition(1000);
                    slider.setPower(0.5);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    slider.setPower(0);


                    arm.setTargetPosition(-1076);
                    arm.setPower(0.6);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    arm.setPower(0);

                    arm.setTargetPosition(0);
                    arm.setPower(0.6);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    arm.setPower(0);

                    slider.setTargetPosition(10);
                    slider.setPower(0.5);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0);
                    arm.setPower(0);
                    break;
                }
                case WEAK:{
                    slider.setTargetPosition(1000);

                    slider.setPower(0.5);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    slider.setPower(0);


                    arm.setTargetPosition(-1076);
                    arm.setPower(0.4);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


                    arm.setPower(0);

                    arm.setTargetPosition(0);
                    arm.setPower(0.4);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(0);

                    slider.setTargetPosition(10);
                    slider.setPower(0.5);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(0);
                    arm.setPower(0);
                    break;
                }
                case STOP:{
                    arm.setPower(0);
                    slider.setPower(0);
                    break;
                }
            }
        }
    }

}
