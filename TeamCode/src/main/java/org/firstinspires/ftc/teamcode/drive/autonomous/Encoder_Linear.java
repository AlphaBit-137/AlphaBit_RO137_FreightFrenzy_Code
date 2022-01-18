/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.teamcode.drive.structure.ChasisInit;

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

@Autonomous(name="Auto Drive By Encoder", group="Tests")
public class Encoder_Linear extends LinearOpMode {

    ChasisInit chasis = new ChasisInit();

    /* Declare OpMode members. */
    HardwarePushbot robot   = new HardwarePushbot();   // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 384.5; ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 10 ;     // For figuring circumference
    static final double     COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        chasis.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        chasis.FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chasis.FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chasis.BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        chasis.BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        chasis.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chasis.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chasis.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        chasis.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d :%7d :%7d",
                          chasis.FrontRight.getCurrentPosition(),
                          chasis.FrontLeft.getCurrentPosition(),
                          chasis.BackRight.getCurrentPosition(),
                          chasis.BackLeft.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  40,  40, 10.0);  // S1: Forward 47 Inches with 5 Sec timeout
       // encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -20, -20, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        //robot.leftClaw.setPosition(1.0);            // S4: Stop and close the claw.
        //robot.rightClaw.setPosition(0.0);
        sleep(1000);     // pause for servos to move

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
    public void encoderDrive(double speed,
                             double leftCentiMeters, double righCentiMeters,
                             double timeoutS) {
        int newLeftUpTarget;
        int newRightUpTarget;
        int newLeftDownTarget;
        int newRightDownTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftUpTarget = chasis.FrontLeft.getCurrentPosition() + (int)(leftCentiMeters * COUNTS_PER_CM);
            newRightUpTarget = chasis.FrontRight.getCurrentPosition() + (int)(righCentiMeters * COUNTS_PER_CM);
            newLeftDownTarget= chasis.BackLeft.getCurrentPosition() + (int)(leftCentiMeters * COUNTS_PER_CM);
            newRightDownTarget= chasis.BackRight.getCurrentPosition() + (int)(righCentiMeters * COUNTS_PER_CM);

            chasis.FrontLeft.setTargetPosition(newLeftUpTarget);
            chasis.FrontRight.setTargetPosition(newRightUpTarget);
            chasis.BackLeft.setTargetPosition(newLeftDownTarget);
            chasis.BackRight.setTargetPosition(newRightDownTarget);

            // Turn On RUN_TO_POSITION
            chasis.FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chasis.FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chasis.BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chasis.BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            chasis.FrontRight.setPower(Math.abs(speed));
            chasis.FrontLeft.setPower(Math.abs(speed));
            chasis.BackRight.setPower(Math.abs(speed));
            chasis.BackLeft.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (chasis.BackLeft.isBusy() && chasis.BackRight.isBusy() && chasis.FrontLeft.isBusy() && chasis.FrontRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d :%7d :%7d", newLeftUpTarget,  newRightUpTarget, newLeftDownTarget, newLeftUpTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d :%7d :%7d",
                                            chasis.FrontRight.getCurrentPosition(),
                                            chasis.FrontLeft.getCurrentPosition(),
                                            chasis.BackRight.getCurrentPosition(),
                                            chasis.BackLeft.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            chasis.FrontRight.setPower(0);
            chasis.FrontLeft.setPower(0);
            chasis.BackRight.setPower(0);
            chasis.BackLeft.setPower(0);

            // Turn off RUN_TO_POSITION
            chasis.FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chasis.FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chasis.BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            chasis.BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(5000);   // optional pause after each move
        }
    }
}
