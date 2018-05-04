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
import com.qualcomm.robotcore.util.Range;


/**
 * This OpMode uses the common Hardware2858 class to define the devices on the robot.
 * All device access is managed through the Hardware2858 class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It opens and closes the clampLeft using the Gamepad A and B buttons respectively.
 * It also opens and closes the clampRight slowly using the A and B buttons.
 */

@TeleOp(name="TurnerBot", group="TurnerBot")

public class TurnerBot extends LinearOpMode {

    /* Declare OpMode members. */
    TurnerHardware2858 robot = new TurnerHardware2858();// Use a K9'shardware
    double          armTTPosition = robot.ARMTT_HOME;
    final double    ARMTT_SPEED      = 1;                            // sets rate to move servo


    @Override
    public void runOpMode() {

        double left;
        double right;
        double lift;
        double string;

        // Initialize the hardware variables. The init() method of the hardware class does all the work here
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in tank mode
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            lift = gamepad2.right_stick_y;
            string = gamepad2.left_stick_y;

            robot.LeftDrive.setPower(left);
            robot.RightDrive.setPower(right);

            if(gamepad2.left_stick_y > 0)

                robot.StringMotor.setPower(string);

            else if (gamepad2.left_stick_y < 0)

                robot.StringMotor.setPower(string);

            else if (gamepad2.y)

                robot.StringMotor.setPower(0);



            if(gamepad2.right_stick_y > 0)

                robot.LiftMotor.setPower(lift);

            else if (gamepad2.right_stick_y < 0)

                robot.LiftMotor.setPower(lift);

            else if (gamepad2.x)

                robot.LiftMotor.setPower(0);

            lift = Range.clip(lift, -0.1, 0.1);
            string = Range.clip(string, -0.1, 0.1);

            //Use gamepad A and B to open and close armSN
            if(gamepad2.a)
                armTTPosition -= ARMTT_SPEED;
            else if (gamepad2.b)
                armTTPosition += ARMTT_SPEED;

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            telemetry.addData("lift",  "%.2f", lift);
            telemetry.addData("string",  "%.2f", string);
            telemetry.addData("armTT",   "%.2f", armTTPosition);
            telemetry.update();

           armTTPosition  = Range.clip(armTTPosition, 0.51, 1.00);
            robot.armTT.setPosition(armTTPosition);
        }
    }
}
