package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
//import org.firstinspires.ftc.teamcode.drive.structure.Lift;
import org.firstinspires.ftc.teamcode.drive.structure.Sliders;

@TeleOp
public class First extends LinearOpMode {
    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;
    public double Limit=0.3;
    public boolean Chose;
    public boolean Chose2;


    Intake intake = new Intake();
    Arm arm = new Arm();
    Sliders slider = new Sliders();
    Carusel duck = new Carusel();

    int ArmModes;

    private ElapsedTime runtime = new ElapsedTime();

    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

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
        arm.init(hardwareMap);
        slider.init(hardwareMap);
        duck.init(hardwareMap);


        runtime.reset();
        waitForStart();

        while (opModeIsActive()) {

            //Initiallizam variabilele
            double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

            //Speed Modes
            if(gamepad1.dpad_up) {
                if(Chose)Limit=Limit+0.1;
                if(Limit>1)Limit=1;
                Chose = false;

            } else Chose=true;

            if(gamepad1.dpad_down) {
                if(Chose2)Limit=Limit-0.1;
                if(Limit<0.3)Limit=0.3;
                Chose2 = false;

            } else {Chose2=true; }
            Front = Range.clip(gamepad1.left_stick_y, -Limit, Limit);
            Turn = Range.clip(-gamepad1.right_stick_x, -Limit, Limit);
            Side = Range.clip(gamepad1.left_stick_x, -Limit, Limit);

            double armup = Range.clip(gamepad2.right_trigger, 0,0.3);
            double armdown = Range.clip(gamepad2.left_trigger,0,0.3);

            //Calcularea puterii redate motoarelor
            Sum = Range.clip(Front + Side, -1.0, 1.0);
            Diff = Range.clip(Front - Side, -1.0, 1.0);

            Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
            Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
            Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
            Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);

            if(gamepad2.x){
                duck.switchToIN();
            }else if(gamepad2.y){
                duck.switchToOUT();
            }else {duck.switchToSTOP();}

            if(gamepad2.dpad_right){
                slider.RobotUp();
            }else if(gamepad1.dpad_left){
                slider.RobotDown();
            }else { slider.RobotStop();}

// Verificarea individuala a motoarelor
            /*if(gamepad1.a){
                BackLeftMotor.setPower(0.5);
            }else{
                BackLeftMotor.setPower(0);
            }

            if(gamepad1.b){
                BackRightMotor.setPower(0.5);
            }else{
                BackRightMotor.setPower(0);
            }

            if(gamepad1.x){
                FrontLeftMotor.setPower(0.5);
            }else{
                FrontLeftMotor.setPower(0);
            }

            if(gamepad1.y){
                FrontRightMotor.setPower(0.5);
            }else{
                FrontRightMotor.setPower(0);
            }
*/

            if(gamepad2.right_bumper){
                slider.switchToUP();
            }else if(gamepad2.left_bumper){
                slider.switchToDOWN();
            }else{
                slider.switchToSTOP();
            }

            if(armup!=0 && armdown!=0){
                arm.RobotStop();
                ArmModes=0;
            }else {
                if(armdown!=0 || armup!=0) {
                    if (armdown<armup)
                    {
                        arm.RobotUP(armup);
                        ArmModes=1;
                    }
                    else
                    {
                        arm.RobotDown(armdown);
                        ArmModes=-1;
                    }
                }
                else
                {
                    arm.RobotStop();
                    ArmModes=0;
                }

            }

            if(gamepad2.a){
                intake.switchToIN();
            }else if(gamepad2.b){
                intake.switchToOUT();
            }else{
                intake.switchToSTOP();
            }
            MS(Drive1, Drive2, Drive3, Drive4);

            telemetry.addData("Motors", "BackLeft (%.2f), FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f)", Drive1, Drive2, Drive3, Drive4);
            //telemetry.addData("Informatie:", "Atentie! Programul a fost stins.");
            telemetry.addData("Power Limit:", Limit + "%");
            telemetry.addData("Poz slider:",slider.Slider.getCurrentPosition());
            telemetry.addData("Poz Carusel",duck.Duck.getCurrentPosition());
            telemetry.addData("Poz intake", intake.intakewing.getCurrentPosition());

//325, 82,
            intake.update();
            slider.update();
            telemetry.update();
            duck.update();
            //lift.update();
        }
    }

    void MS(double x1, double x2, double x3, double x4){
        BackLeftMotor.setPower(x1);
        FrontRightMotor.setPower(x2);
        FrontLeftMotor.setPower(x3);
        BackRightMotor.setPower(x4);


    }

}
