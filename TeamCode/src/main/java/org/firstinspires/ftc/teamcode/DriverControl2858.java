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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;
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

@TeleOp(name="Driver Control 2858", group="DriverControl2858")

public class DriverControl2858 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2858 robot = new Hardware2858();// Use a K9'shardware
    double          clampTopLeftPosition     = robot.CLAMPTOPLEFT_HOME;                   // Servo safe position
    double          clampBottomLeftPosition  = robot.CLAMPBOTTOMLEFT_HOME;              // Servo safe position
    double          clampTopRightPosition    = robot.CLAMPTOPRIGHT_HOME;                  // Servo safe position
    double          clampBottomRightPosition    = robot.CLAMPBOTTOMRIGHT_HOME;                  // Servo safe position
    final double    CLAMPTOPRIGHT_SPEED      = 1;                            // sets rate to move servo
    final double    CLAMPBOTTOMRIGHT_SPEED      = 1;                            // sets rate to move servo
    final double    CLAMPTOPLEFT_SPEED       = 1;                            // sets rate to move servo
    final double    CLAMPBOTTOMLEFT_SPEED      = 1;                         //sets rate to move servo

    DigitalChannel digitalTouch;  // Hardware Device Object


    @Override
    public void runOpMode() {
        double left;
        double right;
        double lift;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();


        // get a reference to our digitalTouch object.
        digitalTouch = hardwareMap.get(DigitalChannel.class, "touch_sensor");

        // set the digital channel to input.
        digitalTouch.setMode(DigitalChannel.Mode.INPUT);


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            robot.leftDrive.setPower(left);
            robot.rightDrive.setPower(right);


            if (digitalTouch.getState() == true) {
                lift = gamepad2.right_stick_y;

                robot.clampLift.setPower(lift);

                lift = Range.clip(lift, -0.5, 0.5) ;

                telemetry.addData("lift", "%.2f", lift);
            }
            else if(digitalTouch.getState() == false) {
                robot.clampLift.setPower(0);

                telemetry.addData("Say", "Limit Reached");    //
                telemetry.update();

                robot.clampLift.setPower(-0.25);
            }

            telemetry.update();

            // Use gamepad Y & B to open and close clampTopLeft
            if (gamepad2.y || gamepad2.x)
                clampTopLeftPosition += CLAMPTOPLEFT_SPEED;
            else if (gamepad2.b)
                clampTopLeftPosition -= CLAMPTOPLEFT_SPEED;

            // Use gamepad Y & B to open and close the clampTopRight
            if (gamepad2.y || gamepad2.x)
                clampTopRightPosition -= CLAMPTOPRIGHT_SPEED;
            else if (gamepad2.b)
                clampTopRightPosition += CLAMPTOPRIGHT_SPEED;


            //Use gamepad A and B to open and close ClampBottomLeft
            if(gamepad2.a || gamepad2.x)
                clampBottomLeftPosition += CLAMPBOTTOMLEFT_SPEED;
            else if (gamepad2.b)
                clampBottomLeftPosition -= CLAMPBOTTOMLEFT_SPEED;


            // Use gamepad A & B to open and close the clampBottomRight
            if (gamepad2.a || gamepad2.x)
                clampBottomRightPosition -= CLAMPBOTTOMRIGHT_SPEED;
            else if (gamepad2.b)
                clampBottomRightPosition += CLAMPBOTTOMRIGHT_SPEED;

            // Move both servos to new position.
            clampTopLeftPosition  = Range.clip(clampTopLeftPosition, 0.20, 0.50);
            robot.clampTopLeft.setPosition(clampTopLeftPosition);
            clampBottomLeftPosition = Range.clip(clampBottomLeftPosition, 0.20, 0.50);
            robot.clampBottomLeft.setPosition(clampBottomLeftPosition);
            clampTopRightPosition = Range.clip(clampTopRightPosition, 0.50, 0.80);
            robot.clampTopRight.setPosition(clampTopRightPosition);
            clampBottomRightPosition = Range.clip(clampBottomRightPosition, 0.50, 0.80);
            robot.clampBottomRight.setPosition(clampBottomRightPosition);

            // Send telemetry message to signify robot running;
            telemetry.addData("clampTopLeft",   "%.2f", clampTopLeftPosition);
            telemetry.addData("clampBottomLeft",    "%.2f", clampBottomLeftPosition);
            telemetry.addData("clampTopRight",  "%.2f", clampTopRightPosition);
            telemetry.addData("clampBottomRight",  "%.2f", clampTopRightPosition);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.update();

        }
    }
}
