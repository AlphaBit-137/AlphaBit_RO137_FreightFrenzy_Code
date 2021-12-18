package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.drive.structure.Arm;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;
import org.firstinspires.ftc.teamcode.drive.structure.Sliders;

@TeleOp
public class First extends LinearOpMode {
    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;

    public DcMotor arm = null;
    public DcMotor intake = null;
    public DcMotor slider = null;
    /*Intake intake = new Intake();
    Arm arm = new Arm();
    Sliders slider = new Sliders();*/

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
        FrontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);

        BackLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        BackRightMotor.setPower(0);

        BackLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        /*intake.init(hardwareMap);
        arm.init(hardwareMap);
        slider.init(hardwareMap);*/

        arm = hardwareMap.get(DcMotor.class, "Arm");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setPower(0);

        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        slider = hardwareMap.get(DcMotor.class, "Slider");
        slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slider.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slider.setPower(0);

        runtime.reset();
        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod

        while (opModeIsActive()) {

            //Initiallizam variabilele
            double Front, Turn, Sum, Diff, Side, Drive1, Drive2, Drive3, Drive4;

            //Primirea datelor de la joystick-uri
            Front = gamepad1.left_stick_y;
            Turn = gamepad1.right_stick_x;
            Side = gamepad1.left_stick_x;


            double armup = Range.clip(gamepad2.right_trigger, 0,0.5);

            double armdown = Range.clip(gamepad2.left_trigger,0,0.5);

            //Calcularea puterii redate motoarelor
            Sum = Range.clip(Front + Side, -1.0, 1.0);
            Diff = Range.clip(Front - Side, -1.0, 1.0);

            Drive1 = Range.clip(Sum - 2*Turn, -1.0, 1.0);
            Drive2 = Range.clip(Sum + 2*Turn, -1.0, 1.0);
            Drive3 = Range.clip(Diff - 2*Turn, -1.0, 1.0);
            Drive4 = Range.clip(Diff + 2*Turn, -1.0, 1.0);


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
            //Repara codil sa mearga cu structure
            /*
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
                }*/
            MS(Drive1, Drive2, Drive3, Drive4);

            /*if(intake.RobotIntake == Intake.IntakeModes.IN){
                telemetry.addData("Intake", "IN");
            }
            if(intake.RobotIntake == Intake.IntakeModes.OUT){
                telemetry.addData("Intake", "OUT");
            }
            if(intake.RobotIntake == Intake.IntakeModes.STOP){
                telemetry.addData("Intake", "STOP");
            }

            if(slider.RobotSlider == Sliders.SlidersModes.DOWN){
                telemetry.addData("Slider", "DOWn");
            }
            if(slider.RobotSlider == Sliders.SlidersModes.UP){
                telemetry.addData("Slider", "UP");
            }
            if(slider.RobotSlider == Sliders.SlidersModes.STOP){
                telemetry.addData("Slider", "STOP");
            }

            if(ArmModes==1){
                telemetry.addData("Arm", "IN");
            }
            if(ArmModes==-1){
                telemetry.addData("Arm", "OUT");
            }
            if(ArmModes==0){
                telemetry.addData("Arm", "STOP");
            }*/

            if(gamepad2.right_bumper){
                slider.setPower(-0.5);
            }else if(gamepad2.left_bumper){
                slider.setPower(0.5);
            }else{
                slider.setPower(0);
            }

            if(gamepad2.a){
                intake.setPower(-0.8);
            }else if(gamepad2.b){
                intake.setPower(0.8);
            }else{
                intake.setPower(0);
            }

            if(armup!=0 && armdown!=0){
                arm.setPower(0);
                ArmModes=0;
            }else {
                if(armdown!=0 || armup!=0) {
                    if (armdown<armup)
                    {
                        arm.setPower(-armup);
                        ArmModes=1;
                    }
                    else
                    {
                        arm.setPower(armdown);
                        ArmModes=-1;
                    }
                }
                else
                {
                    arm.setPower(0);
                    ArmModes=0;
                }

            }

            telemetry.addData("Motors", "BackLeft (%.2f), FrontRight (%.2f), FrontLeft (%.2f), BackRight (%.2f)", Drive1, Drive2, Drive3, Drive4);
            telemetry.addData("Informatie:", "Atentie! Programul a fost sting.");

            /*intake.update();
            slider.update();*/
            telemetry.update();
        }
    }

    void MS(double x1, double x2, double x3, double x4){
        BackLeftMotor.setPower(x1);
        FrontRightMotor.setPower(x2);
        FrontLeftMotor.setPower(x3);
        BackRightMotor.setPower(x4);


    }

}
