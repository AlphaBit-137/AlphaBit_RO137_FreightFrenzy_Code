package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;
//import org.firstinspires.ftc.teamcode.drive.structure.ArmSlider;
import org.firstinspires.ftc.teamcode.drive.structure.ArmPrototype;
import org.firstinspires.ftc.teamcode.drive.structure.ArmStructure;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.ServoStructure;
import org.firstinspires.ftc.teamcode.drive.structure.SliderStructure;


@TeleOp
public class First extends LinearOpMode {
    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;
    public double Limit=0.5;
    public boolean Chose;
    public boolean Chose2;

    boolean reset = false;

    ArmStructure arm = new ArmStructure();
    SliderStructure slider = new SliderStructure();
    ServoStructure servo = new ServoStructure();
    Intake intake = new Intake();
    Carusel duck = new Carusel();

    private double pas = 0;
    private int level = 0;

    private ElapsedTime runtime = new ElapsedTime();

    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    Status IntakeStatus = Status.STATIC;
    Status SliderStatus = Status.STATIC;
    Status ArmStatus = Status.STATIC;

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
        slider.init(hardwareMap);
        arm.init(hardwareMap);
        servo.init(hardwareMap);
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

            //Carusel
            if(gamepad2.x){
                duck.switchToIN();
            } else {duck.switchToSTOP();}

            //Intake
            if (gamepad2.a || IntakeStatus == Status.MOVING){
                intake.switchToIN();
            }
            else{
                if (gamepad2.b){
                    intake.switchToOUT();
                }
                else{

                    intake.switchToSTOP();
                }
            }

            if (gamepad2.y && intake.intakewing.getCurrentPosition()!=0)
                intake.ResetPosition();

            if (intake.IntakeBUSY())
                IntakeStatus = Status.MOVING;
            else{
                if (IntakeStatus == Status.MOVING)
                    IntakeStatus = Status.INITIALIZING;
            }
            if (IntakeStatus == Status.INITIALIZING)
            {
                IntakeStatus = Status.STATIC;
                intake.switchToRESET();
            }

            if (Math.abs(intake.intakewing.getCurrentPosition()) >= 2850)
                intake.switchToRESET();

            //ARM PROTOTYPE

            //Slider
            if (gamepad2.right_bumper || slider.SliderBUSY()){
                slider.switchToSliderUp();
            }else{
                if (gamepad2.left_bumper){
                    slider.switchToSliderDown();
                }
                else{
                    slider.switchToSliderSTOP();
                }
            }
/*
            if (gamepad2.dpad_up && SliderStatus == Status.STATIC){
                slider.SliderMovement(-1400, 0.5);
            }
            if (gamepad2.dpad_down && SliderStatus == Status.STATIC) {
                slider.SliderMovement(1400, 0.5);
            }
*/
            if (slider.SliderBUSY())
            {
                SliderStatus = Status.MOVING;
            }
            else if (SliderStatus == Status.MOVING)
                SliderStatus = Status.INITIALIZING;
            if (SliderStatus == Status.INITIALIZING)
            {
                slider.switchToSliderRESET();
                SliderStatus = Status.STATIC;
            }

            //Arm
            if(gamepad2.right_trigger!=0 && gamepad2.left_trigger==0 || arm.ArmBUSY()){
                arm.switchToArmUp();
            }else{
                if(gamepad2.right_trigger==0 && gamepad2.left_trigger!=0){
                    arm.switchToArmDown();
                }else arm.switchToArmSTOP();
            }
/*
            if (gamepad2.dpad_right && ArmStatus == Status.STATIC)
                arm.ArmMovement(1000, 0.5);
            if (gamepad2.dpad_left && ArmStatus == Status.STATIC)
                arm.ArmMovement(-1000, 0.5);
*/

            if (arm.ArmBUSY())
            {
                ArmStatus = Status.MOVING;
            }
            else if (ArmStatus == Status.MOVING)
                ArmStatus = Status.INITIALIZING;
            if (ArmStatus == Status.INITIALIZING)
            {
                arm.switchToArmRESET();
                ArmStatus = Status.STATIC;
            }

            //Arm assist

            //Level 3
            if((gamepad2.dpad_up || pas!=0) && (level == -3 || level == 0))
            {
                if (level == 0){
                    servo.Closed();
                    intake.ResetPosition();
                    level = -3;
                }
                MovingUp();
            }

            if((gamepad2.dpad_right || pas!=0) && (level == -2 || level == 0))
            {
                if (level == 0) {
                    servo.Closed();
                    intake.ResetPosition();
                    level = -2;
                }
                MovingUp();
            }

            if (gamepad2.dpad_left || pas!=0 || reset == true){
                switch (level){
                    case 3:{
                        if (reset == false)
                            MovingPoz3();
                        else
                            MovingResetPoz3();
                        break;
                    }
                    case 2:{
                        if (reset == false)
                            MovingPoz2();
                        else
                            MovingResetPoz2();
                        break;
                    }
                    default:{
                        break;
                    }
                }
            }

            if (gamepad2.right_stick_button)
            {
                arm.switchToArmRESET();
                slider.switchToSliderRESET();
            }

            MS(Drive1, Drive2, Drive3, Drive4);

            telemetry.addData("Motors", "BackLeft (%.2f), FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f)", Drive1, Drive2, Drive3, Drive4);
            telemetry.addData("Power Limit:", Limit + "%");
            telemetry.addData("Pozitia Slider", slider.slider.getCurrentPosition());
            telemetry.addData("Status Slider", SliderStatus);
            telemetry.addData("Pozitia Arm", arm.arm.getCurrentPosition());
            telemetry.addData("Status Arm", ArmStatus);
            telemetry.addData("Pas", pas);
            telemetry.addData("Pozitia Intake:", intake.intakewing.getCurrentPosition());
            telemetry.addData("Status Intake", IntakeStatus);
            telemetry.addData("Reset", reset);
            telemetry.update();

            intake.update();
            slider.update();
            arm.update();
            duck.update();
        }
    }

    void MS(double x1, double x2, double x3, double x4){
        BackLeftMotor.setPower(x1);
        FrontRightMotor.setPower(x2);
        FrontLeftMotor.setPower(x3);
        BackRightMotor.setPower(x4);

    }

    public void MovingUp()
    {
        if (pas == 0){
            pas=1;
        }
        if (pas == 1)
        {
            pas=1.5;
            slider.SliderMovement(1350, 0.5);
        }
        if (pas == 1.5 && !slider.SliderBUSY()){
            pas = 2;
        }

        if (pas == 2)
        {
            pas=2.5;
            arm.ArmMovement(-1000, 0.5);
        }
        if (pas == 2.5 && !arm.ArmBUSY()){
            pas = 0;
            if (level == -2)
                level = 2;
            if (level == -3)
                level = 3;
        }

    }

    public void MovingPoz3()
    {
        if (pas==0){
            pas=1;
        }

        if(pas == 1)
        {
            pas = 1.5;
            arm.ArmMovement(-160, 0.5);
        }
        if (pas == 1.5 && !arm.ArmBUSY()){
            pas = 2;
        }

        if (pas == 2)
        {
            servo.Open();
            pas = 2.5;
            sleep(1000);
        }

        if (pas == 2.5)
        {
            reset = true;
            pas = 0;
        }


    }

    public void MovingResetPoz3(){
        if (pas == 0){
            pas = 1.5;
            arm.ArmMovement(1170, 0.5);
        }
        if (pas == 1.5 && !arm.ArmBUSY()){
            pas = 2;
        }

        if (pas == 2)
        {
            pas = 2.5;
            slider.SliderMovement(-1315, 0.5);
        }
        if (pas == 2.5 && !slider.SliderBUSY()){

            level = 0;
            reset = false;
            pas = 0;
            slider.switchToSliderRESET();
            arm.switchToArmRESET();
        }
    }

    public void MovingPoz2()
    {
//-200, -1315
        if (pas==0){
            pas=1;
        }

        if(pas == 1)
        {
            pas = 1.5;
            arm.ArmMovement(-210, 0.5);
        }
        if (pas == 1.5 && !arm.ArmBUSY()){
            pas = 2;
        }

        if (pas == 2){
            pas = 2.5;
            slider.SliderMovement(-1000, 0.5);
        }

        if (pas == 2.5 && !slider.SliderBUSY()){
            pas = 3;
        }

        if (pas == 3)
        {
            servo.Open();
            pas = 3.5;
            sleep(2000);
        }

        if (pas == 3.5)
        {
            reset = true;
            pas = 0;
        }
    }

    public void MovingResetPoz2(){
        if (pas == 0 && !slider.SliderBUSY())
        {
            pas = 1;
        }

        if (pas == 1)
        {
            slider.SliderMovement(1120, 0.5);
            pas = 1.5;
        }
        if (pas == 1.5 && !slider.SliderBUSY())
        {
            pas = 2;
        }

        if (pas == 2)
        {
            arm.ArmMovement(1240, 0.5);
            pas = 2.5;
        }
        if (pas == 2.5 && !arm.ArmBUSY()){
            pas =3;
        }

        if (pas == 3)
        {
            slider.SliderMovement(-1200, 0.5);
            pas=3.5;
        }
        if (pas == 3.5 && !slider.SliderBUSY()){

            level = 0;
            reset = false;
            pas = 0;
            slider.switchToSliderRESET();
            arm.switchToArmRESET();
        }
//1460, 1225
    }

}
