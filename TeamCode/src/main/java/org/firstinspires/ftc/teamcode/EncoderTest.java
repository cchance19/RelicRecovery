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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorController;


import java.util.Locale;

@Autonomous(name="EncoderTest", group="Autonomous")
public class EncoderTest extends LinearOpMode {

    // Declare OpMode members.
    Hardware2858 robot = new Hardware2858();// Use a K9'shardware
    private ElapsedTime runtime = new ElapsedTime();

            @Override
            public void runOpMode() throws InterruptedException
            {

                robot.init(hardwareMap);

                // reset encoder count kept by left motor.
                robot.liftDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.liftDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // set left motor to run to target encoder position and stop with brakes on.
                robot.liftDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.liftDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                telemetry.addData("Mode", "waiting");
                telemetry.update();

                // wait for start button.

                waitForStart();

                telemetry.addData("Mode", "running");
                telemetry.update();

                // set left motor to run for 5000 encoder counts.

                robot.liftDriveLeft.setTargetPosition(1538);
                robot.liftDriveRight.setTargetPosition(1538);

                // set both motors to 25% power. Movement will start.

                robot.liftDriveLeft.setPower(1);
                robot.liftDriveRight.setPower(1);

                // wait while opmode is active and left motor is busy running to position.

                while (opModeIsActive() && robot.liftDriveLeft.isBusy())
                {
                    telemetry.addData("encoder-fwd", robot.liftDriveLeft.getCurrentPosition() + "  busy=" + robot.liftDriveLeft.isBusy());
                    telemetry.update();
                    idle();
                }
                while (opModeIsActive() && robot.liftDriveRight.isBusy())
                {
                    telemetry.addData("encoder-fwd", robot.liftDriveRight.getCurrentPosition() + "  busy=" + robot.liftDriveRight.isBusy());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to turn off motors. The motors stop on their own but
                // power is still applied so we turn off the power.

                robot.liftDriveLeft.setPower(0.0);
                robot.liftDriveRight.setPower(0.0);

                // wait 5 sec to you can observe the final encoder position.
/*
                resetStartTime();

                while (opModeIsActive() && getRuntime() < 5)
                {
                    telemetry.addData("encoder-fwd-end", robot.liftDriveLeft.getCurrentPosition() + "  busy=" + robot.liftDriveLeft.isBusy());
                    telemetry.update();
                    idle();
                }
                while (opModeIsActive() && getRuntime() < 5)
                {
                    telemetry.addData("encoder-fwd-end", robot.liftDriveRight.getCurrentPosition() + "  busy=" + robot.liftDriveRight.isBusy());
                    telemetry.update();
                    idle();
                }

                // set position for back up to starting point. In this example instead of
                // having the motor monitor the encoder we will monitor the encoder ourselves.

                robot.liftDriveLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                robot.liftDriveRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                robot.liftDriveLeft.setTargetPosition(0);
                robot.liftDriveRight.setTargetPosition(0);

                robot.liftDriveLeft.setPower(0.25);
                robot.liftDriveRight.setPower(0.25);

                while (opModeIsActive() && robot.liftDriveLeft.getCurrentPosition() < robot.liftDriveLeft.getTargetPosition())
                {
                    telemetry.addData("encoder-back", robot.liftDriveLeft.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                while (opModeIsActive() && robot.liftDriveRight.getCurrentPosition() < robot.liftDriveRight.getTargetPosition())
                {
                    telemetry.addData("encoder-back", robot.liftDriveRight.getCurrentPosition());
                    telemetry.update();
                    idle();
                }

                // set motor power to zero to stop motors.

                robot.liftDriveLeft.setPower(0.0);
                robot.liftDriveRight.setPower(0.0);

                resetStartTime();

                while (opModeIsActive() && getRuntime() < 5)
                {
                    telemetry.addData("encoder-back-end", robot.liftDriveLeft.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                while (opModeIsActive() && getRuntime() < 5)
                {
                    telemetry.addData("encoder-back-end", robot.liftDriveRight.getCurrentPosition());
                    telemetry.update();
                    idle();
                }
                */
            }
        }