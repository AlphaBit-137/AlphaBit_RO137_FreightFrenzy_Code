package org.firstinspires.ftc.teamcode.drive.autonomous;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.structure.ArmAssist;
import org.firstinspires.ftc.teamcode.drive.structure.Carusel;
import org.firstinspires.ftc.teamcode.drive.structure.Intake;

@Autonomous (group="Autonoame Basic")
public class BlueSmallParkOnly_Encoder extends LinearOpMode {

    public DcMotor BackLeftMotor = null;
    public DcMotor FrontRightMotor = null;
    public DcMotor FrontLeftMotor = null;
    public DcMotor BackRightMotor = null;

    ArmAssist Assist = new ArmAssist();
    Intake intake = new Intake();
    Carusel duck = new Carusel();

    int ArmModes;

    //Constante
    private static double MAX_POWER = 1.0, MIN_POWER = -1.0, NULL_POWER = 0.0;

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

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


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        BackLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDriveForward(0.4,4.5,5);
        sleep(20000);
        StrafeRight(0.2,3,5);
        encoderDriveBack(0.4,2,2);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDriveForward(double speed,
                                    double inches,
                                    double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeftMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = BackRightMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
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

            sleep(1000);   // optional pause after each move
        }
    }

    public void StrafeRight(double speed,
                            double inches,
                            double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = BackRightMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() - (int)(inches * COUNTS_PER_INCH);

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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
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

            sleep(1000);   // optional pause after each move
        }
    }

    public void encoderDriveBack(double speed,
                                 double inches,
                                 double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = BackLeftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newBackRightTarget = BackRightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontRightTarget = FrontRightMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            newFrontLeftTarget = FrontLeftMotor.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

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
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
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

            sleep(1000);   // optional pause after each move
        }
    }

}
