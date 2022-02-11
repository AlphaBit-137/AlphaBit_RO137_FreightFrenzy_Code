package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;
//import org.firstinspires.ftc.teamcode.drive.structure.ArmSlider;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;



@TeleOp
public class First extends LinearOpMode {
    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;
    public double Limit=0.5;
    public boolean Chose;
    public boolean Chose2;
    ArmAssist assist = new ArmAssist();
    //  ArmSlider works = new ArmSlider();
    Intake intake = new Intake();
    Carusel duck = new Carusel();

    private ElapsedTime runtime = new ElapsedTime();

    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    public Status IntakeStatus = Status.STATIC;

    //Status structuri autonoame
    public enum Status{

        MOVING,
        INITIALIZING,
        STATIC

    }

    @Override
    public void runOpMode() {

        BackLeftMotor = hardwareMap.get(DcMotor.class, "Back_Left");
        FrontRightMotor = hardwareMap.get(DcMotor.class, "Front_Right");
        FrontLeftMotor = hardwareMap.get(DcMotor.class, "Front_Left");
        BackRightMotor = hardwareMap.get(DcMotor.class, "Back_Right");

        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);

        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.init(hardwareMap);
        duck.init(hardwareMap);
        assist.init(hardwareMap);
        //  works.init(hardwareMap);
        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {

            //Initiallizam variabilele
            double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

            Front = Range.clip(gamepad1.left_stick_y, -Limit, Limit);
            Turn = Range.clip(gamepad1.right_stick_x, -Limit, Limit);
            Side = Range.clip(gamepad1.left_stick_x, -Limit, Limit);

            //Speed Modes
            if(gamepad1.right_bumper) {
                if(Chose)Limit=Limit+0.1;
                if(Limit>1)Limit=1;
                Chose = false;
            } else Chose=true;

            if(gamepad1.left_bumper) {
                if(Chose2)Limit=Limit-0.1;
                if(Limit<0.3)Limit=0.3;
                Chose2 = false;
            } else {Chose2=true; }


            double armup = Range.clip(gamepad2.right_trigger, 0,0.3);
            double armdown = Range.clip(gamepad2.left_trigger,0,0.3);

            //Calcularea puterii redate motoarelor
            Sum = Range.clip(Front + Side, -1.0, 1.0);
            Diff = Range.clip(Front - Side, -1.0, 1.0);

            Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
            Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
            Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
            Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);

            /*if(gamepad2.right_bumper){
                intake.TimeIntake();
            }else if(gamepad2.left_bumper){
                //
                // intake.Reset2();
            }*/

            if(gamepad2.x){
                duck.switchToIN();
            } else {duck.switchToSTOP();}

            if (gamepad2.dpad_up) {
                assist.switchTo3();
            } else {
                assist.switchToSTOP();
            }

            if (gamepad2.dpad_right) {
                assist.switchTo2();
            } else {
                assist.switchToSTOP();
            }

            if (gamepad2.right_bumper)
            {
                assist.switchToArmUp();
            }
            else
            {
                assist.switchToSTOP();
            }

            //assist.arm.setPower(armup);
            //assist.arm.setPower(armdown);

            if(gamepad2.a){
                intake.switchToIN();
            }else if(gamepad2.b || IntakeStatus == Status.MOVING){
                intake.switchToOUT();
            }else{
                intake.switchToSTOP();
            }

            if (Math.abs(intake.intakewing.getCurrentPosition()) >= 2850)
                intake.switchToRESET();

            if (gamepad2.dpad_left)
            {
                intake.ResetPosition();
            }

            if (intake.IntakeBUSY())
                IntakeStatus = Status.MOVING;
            else
                IntakeStatus = Status.INITIALIZING;
            if (IntakeStatus == Status.INITIALIZING)
            {
                IntakeStatus = Status.STATIC;
                intake.switchToRESET();
            }

            MS(Drive1, Drive2, Drive3, Drive4);

            telemetry.addData("Motors", "BackLeft (%.2f), FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f)", Drive1, Drive2, Drive3, Drive4);
            telemetry.addData("Power Limit:", Limit + "%");
            telemetry.addData("Poz Carusel",duck.Duck.getCurrentPosition());
            telemetry.addData("Test Intake", IntakeStatus);

            intake.update();
            telemetry.update();
            assist.update();
            duck.update();

        }
    }

    void MS(double x1, double x2, double x3, double x4){
        BackLeftMotor.setPower(x1);
        FrontRightMotor.setPower(x2);
        FrontLeftMotor.setPower(x3);
        BackRightMotor.setPower(x4);

    }

}
