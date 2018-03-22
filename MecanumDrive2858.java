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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;



/**
 * This OpMode uses the common Hardware2858 class to define the devices on the robot.
 * All device access is managed through the Hardware2858 class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It opens and closes the clampLeft using the Gampad A and B buttons respectively.
 * It also opens and closes the clampRight slowly using the A and B buttons.
 */

@TeleOp(name="Mecanum Control 2858", group="MecanumControl2858")

public class MecanumDrive2858 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2858 robot = new Hardware2858();// Use a K9'shardware

    @Override
    public void runOpMode() {


        // Initialize the hardware variables. The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);                           //Gives R of Polar Coordinates
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;    //Gives THETA of Polar Coordinates
            double rightX = gamepad1.right_stick_x;                                                         //Turning
            final double frontLeft = r * Math.cos(robotAngle) + rightX;                                     //Moves in same direction as BackRight
            final double frontRight = r * Math.sin(robotAngle) - rightX;                                    //Moves in same direction as BackLeft
            final double backLeft = r * Math.sin(robotAngle) + rightX;                                      //Moves in same direction as FrontRight
            final double backRight = r * Math.cos(robotAngle) - rightX;                                     //Moves in same direction as FrontLeft

            robot.frontLeftDrive.setPower(frontLeft);
            robot.frontRightDrive.setPower(frontRight);
            robot.backLeftDrive.setPower(backLeft);
            robot.backRightDrive.setPower(backRight);

            telemetry.addData("frontLeft",  "%.2f", frontLeft);
            telemetry.addData("frontRight", "%.2f", frontRight);
            telemetry.addData("backLeft",  "%.2f", backLeft);
            telemetry.addData("backRight", "%.2f", backRight);
            telemetry.update();
        }
    }
}
