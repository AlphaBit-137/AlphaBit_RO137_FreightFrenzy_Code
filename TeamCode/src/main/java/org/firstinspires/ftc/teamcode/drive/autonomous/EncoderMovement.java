package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;

public class EncoderMovement {

    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;

    ArmAssist Assist = new ArmAssist();
    Intake intake = new Intake();
    Carusel duck = new Carusel();

    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {

        hwMap = ahwMap;

        BackLeftMotor = hwMap.get(DcMotor.class, "Back_Left");
        FrontRightMotor = hwMap.get(DcMotor.class, "Front_Right");
        FrontLeftMotor = hwMap.get(DcMotor.class, "Front_Left");
        BackRightMotor = hwMap.get(DcMotor.class, "Back_Right");

        BackLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        FrontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        BackRightMotor.setDirection(DcMotor.Direction.FORWARD);

        BackLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

        public void encoderDriveLinear(double speed,
        double centimeters,
        double timeoutS,
        int direction) {

            int newFrontLeftTarget;
            int newFrontRightTarget;
            int newBackRightTarget;
            int newBackLeftTarget;

            // Ensure that the opmode is still active

                // Determine new target position, and pass to motor controller
                newBackLeftTarget = BackLeftMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
                newBackRightTarget = BackRightMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
                newFrontRightTarget = FrontRightMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
                newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);

                BackRightMotor.setTargetPosition(newBackRightTarget);
                BackLeftMotor.setTargetPosition(newBackLeftTarget);
                FrontRightMotor.setTargetPosition(newFrontRightTarget);
                FrontLeftMotor.setTargetPosition(newFrontLeftTarget);


                // Turn On RUN_TO_POSITION
                BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // reset the timeout time and start motion.
                runtime.reset();

                BackRightMotor.setPower(Math.abs(speed));
                BackLeftMotor.setPower(Math.abs(speed));
                FrontRightMotor.setPower(Math.abs(speed));
                FrontLeftMotor.setPower(Math.abs(speed));

                // keep looping while we are still active, and there is time left, and both motors are running.
                // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
                // its target position, the motion will stop.  This is "safer" in the event that the robot will
                // always end the motion as soon as possible.
                // However, if you require that BOTH motors have finished their moves before the robot continues
                // onto the next step, use (isBusy() || isBusy()) in the loop test.
                while ((runtime.seconds() < timeoutS) &&
                        (BackLeftMotor.isBusy() && BackRightMotor.isBusy()&&FrontLeftMotor.isBusy() && FrontRightMotor.isBusy())) {

                /*// Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();*/
                }

                // Stop all motion;
                BackRightMotor.setPower(0);
                BackLeftMotor.setPower(0);
                FrontRightMotor.setPower(0);
                FrontLeftMotor.setPower(0);

                // Turn off RUN_TO_POSITION
                BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

    public void encoderDriveStrafe(double speed,
                                 double centimeters,
                                 double timeoutS,
                                 int direction) {


        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeftMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newBackRightTarget = BackRightMotor.getCurrentPosition() + 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() + 1 * direction * (int)(centimeters * COUNTS_PER_INCH);

            BackRightMotor.setTargetPosition(newBackRightTarget);
            BackLeftMotor.setTargetPosition(newBackLeftTarget);
            FrontRightMotor.setTargetPosition(newFrontRightTarget);
            FrontLeftMotor.setTargetPosition(newFrontLeftTarget);


            // Turn On RUN_TO_POSITION
            BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            BackRightMotor.setPower(Math.abs(speed));
            BackLeftMotor.setPower(Math.abs(speed));
            FrontRightMotor.setPower(Math.abs(speed));
            FrontLeftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (BackLeftMotor.isBusy() && BackRightMotor.isBusy()&&FrontLeftMotor.isBusy() && FrontRightMotor.isBusy())) {

                /*// Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            BackRightMotor.setPower(0);
            BackLeftMotor.setPower(0);
            FrontRightMotor.setPower(0);
            FrontLeftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void encoderDriveRotate(double speed,
                                 double centimeters,
                                 double timeoutS,
                                 int direction) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;


            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeftMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newBackRightTarget = BackRightMotor.getCurrentPosition() + 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightMotor.getCurrentPosition() + 1 * direction * (int)(centimeters * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() - 1 * direction * (int)(centimeters * COUNTS_PER_INCH);

            BackRightMotor.setTargetPosition(newBackRightTarget);
            BackLeftMotor.setTargetPosition(newBackLeftTarget);
            FrontRightMotor.setTargetPosition(newFrontRightTarget);
            FrontLeftMotor.setTargetPosition(newFrontLeftTarget);


            // Turn On RUN_TO_POSITION
            BackRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            BackRightMotor.setPower(Math.abs(speed));
            BackLeftMotor.setPower(Math.abs(speed));
            FrontRightMotor.setPower(Math.abs(speed));
            FrontLeftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ((runtime.seconds() < timeoutS) &&
                    (BackLeftMotor.isBusy() && BackRightMotor.isBusy()&&FrontLeftMotor.isBusy() && FrontRightMotor.isBusy())) {

                /*// Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                                            robot.leftDrive.getCurrentPosition(),
                                            robot.rightDrive.getCurrentPosition());
                telemetry.update();*/
            }

            // Stop all motion;
            BackRightMotor.setPower(0);
            BackLeftMotor.setPower(0);
            FrontRightMotor.setPower(0);
            FrontLeftMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}