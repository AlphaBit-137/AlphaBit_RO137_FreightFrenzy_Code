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

package org.firstinspires.ftc.teamcode.drive.sliders;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Brat_Test", group = "Linear Opmode")

public class BratTest extends LinearOpMode {

    // Declaram obiectul robot cu clasa hardware si timpul de rulare
    private ElapsedTime runtime = new ElapsedTime();

    public DcMotorEx arm = null;
    public DcMotorEx lift = null;


    //Constante

    @Override
    public void runOpMode() {

        lift = hardwareMap.get(DcMotorEx.class, "Slider" +
                "");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        lift.setVelocityPIDFCoefficients(1.174, 0.1174, 0, 11.74);
        lift.setPositionPIDFCoefficients(5.0);

        waitForStart();

        // Atata timp cat OpMode-ul este activ va rula pana la oprire urmatorul cod
        while (opModeIsActive()) {
            double power = Range.clip(gamepad2.left_stick_y, -1, 1);
            if (gamepad2.a) {
                lift.setPower(-0.8);
                runtime.reset();
                while (runtime.seconds() <= 0.5) {

                }
                lift.setPower(0);
            }
            if (gamepad2.b){
                lift.setPower(0.8);
                runtime.reset();
                while (runtime.seconds()<=0.5)
                {

                }
                lift.setPower(0);
            }

            lift.setPower(power);


            telemetry.addData("Slider", lift.getCurrentPosition());
            telemetry.addData("Power", power);

            telemetry.update();
        }
    }

}