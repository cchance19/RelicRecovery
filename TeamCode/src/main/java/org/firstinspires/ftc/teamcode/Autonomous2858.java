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

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

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
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Autonomous2858", group="Autonomous")
public class Autonomous2858 extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware2858 robot = new Hardware2858();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 219.174419;    // eg: REV Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 6.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.75;
    static final double TURN_SPEED = 0.75;

    static final double LIFT_COUNTS_PER_MOTOR_REV = 4833.07692307;    // eg: REV Motor Encoder
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV) / (3.1415);
    static final double LIFT_SPEED = 1;

    public GoldAlignDetector detector;

    @Override
    public void runOpMode() {

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        // Optional Tuning
        detector.alignSize = 400; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

     //   telemetry.addData("X Pos", detector.getXPosition()); // Gold X pos.

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftDriveLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftDriveRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.liftDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.leftDrive.getCurrentPosition(),
                robot.rightDrive.getCurrentPosition(),
                robot.liftDriveLeft.getCurrentPosition(),
                robot.liftDriveRight.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        robot.teamMarker.setPosition(-0.9);

        liftEncoderDrive(LIFT_SPEED, 6, 6, 4.00);
        encoderDrive(DRIVE_SPEED, 0.5, 0.5, 4.00);
        encoderDrive(TURN_SPEED, 2.5, -2.5, 4.00);
        encoderDrive(DRIVE_SPEED, -2, -2, 4.00);
        encoderDrive(TURN_SPEED, -2.5, 2.5, 4.00);
        encoderDrive(DRIVE_SPEED, -5, -5, 4.00);

        telemetry.addData("IsAligned", detector.getAligned()); // Is the bot aligned with the gold mineral

        sleep(1000);

        if (detector.getAligned()) {
            encoderDrive(DRIVE_SPEED, -22, -22, 4.00);
            encoderDrive(DRIVE_SPEED, 19, 19, 4.00);
            encoderDrive(DRIVE_SPEED, -8, -8, 4.00);
            encoderDrive(TURN_SPEED, 14.65, -14.65, 4.00);
            encoderDrive(DRIVE_SPEED, 43, 43, 4.00);
            encoderDrive(DRIVE_SPEED, -1,-1,4.00);
            encoderDrive(TURN_SPEED, -7.5, 7.5, 4.00);
        }
        else if (!detector.getAligned()) {
            encoderDrive(DRIVE_SPEED, 7, -7, 4.00);

            sleep(1000);

            if (detector.getAligned()) {
                encoderDrive(DRIVE_SPEED, -27, -27, 4.00);
                encoderDrive(DRIVE_SPEED, 17, 17, 4.00);
                encoderDrive(TURN_SPEED, -7,7,4.00);
                encoderDrive(TURN_SPEED, 14.65, -14.65, 4.00);
                encoderDrive(DRIVE_SPEED, 46, 46, 4.00);
                encoderDrive(DRIVE_SPEED, -1,-1,4.00);
                encoderDrive(TURN_SPEED, -7.5, 7.5, 4.00);
            }
            else if (!detector.getAligned()) {
                encoderDrive(TURN_SPEED, -11,11,4.00);
                encoderDrive(DRIVE_SPEED, -27,-27,4.00);
                encoderDrive(TURN_SPEED, 5,-5,4.00);
                encoderDrive(TURN_SPEED, 11.5,-11.5,4.00);
                encoderDrive(DRIVE_SPEED, 30,30,4.00);
                encoderDrive(TURN_SPEED, -2,2,4.00);
            }
        }

        encoderDrive(DRIVE_SPEED, 80, 80, 4.00);
        encoderDrive(DRIVE_SPEED, -5,-5,4.00);

        robot.teamMarker.setPosition(0.9);

        encoderDrive(LIFT_SPEED, -120, -120, 4.00);


        telemetry.addData("Path", "Complete");
        telemetry.update();
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.leftDrive.setTargetPosition(newLeftTarget);
            robot.rightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.leftDrive.setPower(Math.abs(speed));
            robot.rightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.leftDrive.isBusy() && robot.rightDrive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.leftDrive.getCurrentPosition(),
                        robot.rightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void liftEncoderDrive(double speed,
                                 double liftLeftInches, double liftRightInches,
                                 double timeoutS) {
        int newLiftLeftTarget;
        int newLiftRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLiftLeftTarget = robot.liftDriveLeft.getCurrentPosition() + (int) (liftLeftInches * LIFT_COUNTS_PER_INCH);
            newLiftRightTarget = robot.liftDriveRight.getCurrentPosition() + (int) (liftRightInches * LIFT_COUNTS_PER_INCH);
            robot.liftDriveLeft.setTargetPosition(newLiftLeftTarget);
            robot.liftDriveRight.setTargetPosition(newLiftRightTarget);

            // Turn On RUN_TO_POSITION
            robot.liftDriveLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.liftDriveRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.liftDriveLeft.setPower(Math.abs(speed));
            robot.liftDriveRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.liftDriveLeft.isBusy() && robot.liftDriveRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path3", "Running to %7d :%7d", newLiftLeftTarget, newLiftRightTarget);
                telemetry.addData("Path4", "Running at %7d :%7d",
                        robot.liftDriveLeft.getCurrentPosition(),
                        robot.liftDriveRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.liftDriveLeft.setPower(0);
            robot.liftDriveRight.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.liftDriveLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.liftDriveRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
}