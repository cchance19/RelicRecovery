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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Driver Control 2858", group="DriverControl2858")
public class DriverControl2858 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2858 robot = new Hardware2858();// Use a K9's hardware

    double teamMarkerPosition = robot.TEAMMARKER_HOME;                   // Servo safe position
    double sweeperServoLPosition = robot.SWEEPERSERVOL_HOME;
    double sweeperServoRPosition = robot.SWEEPERSERVOR_HOME;

    @Override
    public void runOpMode() {

        //Initialize the hardware variables. The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // reset encoder count kept by left motor.
        robot.liftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set left motor to run to target encoder position and stop with brakes on.
        robot.liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double left;
            double right;
            double liftUp = gamepad1.right_trigger;
            double liftDown = gamepad1.left_trigger;
            double sweeperOut = gamepad2.right_trigger;
            double sweeperIn = gamepad2.left_trigger;
            double gripper = gamepad2.right_stick_y;

            robot.teamMarker.setPosition(0.0);

            // Run wheels in tank mode
            left = gamepad1.left_stick_y;
            right = gamepad1.right_stick_y;
            robot.leftDriveOUT.setPower(-left);
            robot.leftDriveIN.setPower(-left);
            robot.rightDriveOUT.setPower(-right);
            robot.rightDriveIN.setPower(-right);
            robot.gripperDrive.setPower(gripper);

            if (gamepad2.right_trigger > gamepad2.left_trigger) {
                robot.sweeperDrive.setPower(-sweeperOut);
            }
            else if (gamepad2.right_trigger < gamepad2.left_trigger) {
                robot.sweeperDrive.setPower(sweeperIn);
            }
            else if (gamepad2.right_trigger == gamepad2.left_trigger) {
                    robot.sweeperDrive.setPower(0);
            }

            if (gamepad1.right_trigger > gamepad1.left_trigger) {
                robot.liftDrive.setPower(liftUp);
            } else if (gamepad1.right_trigger < gamepad1.left_trigger) {
                robot.liftDrive.setPower(-liftDown);
            } else if (gamepad1.right_trigger == gamepad1.left_trigger) {
                robot.liftDrive.setPower(0);
            }



/*
            if (robot.liftDrive.getCurrentPosition() > -6400) {
                if (gamepad1.right_trigger > gamepad1.left_trigger) {
                    robot.liftDrive.setPower(liftUp);
                } else if (gamepad1.right_trigger < gamepad1.left_trigger) {
                    robot.liftDrive.setPower(-liftDown);
                } else if (gamepad1.right_trigger == gamepad1.left_trigger) {
                    robot.liftDrive.setPower(0);
                }
            } else if (robot.liftDrive.getCurrentPosition() <= -6400) {
                robot.liftDrive.setPower(0);
            }
            */

            // Send telemetry message to indicate successful Encoder reset
            telemetry.addData("Path0", "Starting at %7d :",
                    robot.liftDrive.getCurrentPosition());
            telemetry.update();

                teamMarkerPosition = Range.clip(teamMarkerPosition, robot.TEAMMARKER_MIN_RANGE, robot.TEAMMARKER_MAX_RANGE);
                robot.teamMarker.setPosition(teamMarkerPosition);

                if (gamepad2.a) {
                    //sweeperServoLPosition = robot.SWEEPERSERVOL_MIN_RANGE;
                    //sweeperServoRPosition = robot.SWEEPERSERVOR_MAX_RANGE;
                    robot.sweeperServoL.setPosition(0);
                }
                else if (gamepad2.b) {
                    //sweeperServoLPosition = robot.SWEEPERSERVOL_MAX_RANGE;
                    //sweeperServoRPosition = robot.SWEEPERSERVOR_MIN_RANGE;
                    robot.sweeperServoL.setPosition(-0.9);
                }

                if (gamepad2.a) {
                //sweeperServoLPosition = robot.SWEEPERSERVOL_MIN_RANGE;
                //sweeperServoRPosition = robot.SWEEPERSERVOR_MAX_RANGE;
                    robot.sweeperServoR.setPosition(0);
                }
                else if (gamepad2.b) {
                //sweeperServoLPosition = robot.SWEEPERSERVOL_MAX_RANGE;
                //sweeperServoRPosition = robot.SWEEPERSERVOR_MIN_RANGE;
                    robot.sweeperServoR.setPosition(0.9);
                }

                sweeperServoLPosition = Range.clip(sweeperServoLPosition, robot.SWEEPERSERVOL_MIN_RANGE, robot.SWEEPERSERVOL_MAX_RANGE);
                sweeperServoRPosition = Range.clip(sweeperServoRPosition, robot.SWEEPERSERVOR_MIN_RANGE, robot.SWEEPERSERVOR_MAX_RANGE);
                robot.sweeperServoL.setPosition(sweeperServoLPosition);
                robot.sweeperServoR.setPosition(sweeperServoRPosition);

                // Send telemetry message to signify robot running;
                telemetry.addData("left", "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                telemetry.addData("liftUp", "%.2f", liftUp);
                telemetry.addData("liftDown", "%.2f", liftDown);
                telemetry.addData("sweeperOut", "%.2f", sweeperOut);
                telemetry.addData("sweeperIn", "%.2f", sweeperIn);
                telemetry.addData("gripper", "%.2f", gripper);
                telemetry.update();
            }
        }
    }




    /*                if (robot.liftDriveLeft.getCurrentPosition() > -6400 && robot.liftDriveRight.getCurrentPosition() > -6400) {
                    if (gamepad1.right_trigger > gamepad1.left_trigger) {
                        robot.liftDriveLeft.setPower(liftUp);
                        robot.liftDriveRight.setPower(liftUp);
                    } else if (gamepad1.right_trigger < gamepad1.left_trigger) {
                        robot.liftDriveLeft.setPower(-liftDown);
                        robot.liftDriveRight.setPower(-liftDown);
                    } else if (gamepad1.right_trigger == gamepad1.left_trigger) {
                        robot.liftDriveLeft.setPower(0);
                        robot.liftDriveRight.setPower(0);
                    }
                } else if (robot.liftDriveLeft.getCurrentPosition() <= -6400 && robot.liftDriveRight.getCurrentPosition() <= -6400) {
                    robot.liftDriveLeft.setPower(0);
                    robot.liftDriveRight.setPower(0);
                }

                // Send telemetry message to indicate successful Encoder reset
                telemetry.addData("Path0", "Starting at %7d :%7d",
                        robot.liftDriveLeft.getCurrentPosition(),
                        robot.liftDriveRight.getCurrentPosition());
                telemetry.update();
    */
