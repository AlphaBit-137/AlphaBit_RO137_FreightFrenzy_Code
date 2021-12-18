package org.firstinspires.ftc.teamcode.drive.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends LinearOpMode {
    public DcMotor arm;


    @Override
    public void runOpMode() throws InterruptedException {

    }


    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        arm = hwMap.get(DcMotor.class, "Arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);
    }

    public void RobotDown(double power) {
        arm.setPower(power);
    }

    public void RobotUP(double power) {
        arm.setPower(-power);
    }

    public void RobotStop() {
        arm.setPower(0);
    }

}