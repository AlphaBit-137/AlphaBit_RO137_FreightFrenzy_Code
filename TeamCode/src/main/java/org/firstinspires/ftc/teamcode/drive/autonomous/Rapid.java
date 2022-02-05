package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;

@Autonomous
public class Rapid extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    public DcMotor BackLeft = null;
    public DcMotor FrontRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor BackRight = null;

    ArmAssist assist = new ArmAssist();

    @Override
    public void runOpMode() throws InterruptedException {

        BackLeft = hardwareMap.get(DcMotor.class, "Back_Left");
        FrontRight = hardwareMap.get(DcMotor.class, "Front_Right");
        FrontLeft = hardwareMap.get(DcMotor.class, "Front_Left");
        BackRight = hardwareMap.get(DcMotor.class, "Back_Right");


        BackLeft.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.FORWARD);
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.FORWARD);

        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        waitForStart();

        BackLeft.setPower(-0.7);
        FrontRight.setPower(-0.7);
        FrontLeft.setPower(0.7);
        BackRight.setPower(0.7);
        runtime.reset();
        while(runtime.seconds() <= 0.4) {


        }
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        assist.Complex();

        BackLeft.setPower(0.7);
        FrontRight.setPower(0.7);
        FrontLeft.setPower(-0.7);
        BackRight.setPower(-0.7);
        runtime.reset();
        while(runtime.seconds() <= 0.4) {


        }
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);

        BackLeft.setPower(0.7);
        FrontRight.setPower(0.7);
        FrontLeft.setPower(0.7);
        BackRight.setPower(0.7);
        runtime.reset();
        while(runtime.seconds() <= 0.8) {


        }
        BackLeft.setPower(0);
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);


    }
}
