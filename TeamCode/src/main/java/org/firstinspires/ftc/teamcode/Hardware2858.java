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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 */

public class Hardware2858
{
    // Public OpMode members.
    public DcMotor  leftDriveOUT   = null;
    public DcMotor  leftDriveIN   = null;
    public DcMotor  rightDriveOUT  = null;
    public DcMotor  rightDriveIN  = null;
    public DcMotor  liftDrive       = null;
    public Servo    teamMarker      = null;
    public DcMotor  sweeperDrive    = null;
    public Servo    sweeperServoL    = null;
    public Servo    sweeperServoR    = null;
    public DcMotor  gripperDrive    = null;

    public final static double TEAMMARKER_HOME = 0;
    public final static double TEAMMARKER_MIN_RANGE = -1.0;
    public final static double TEAMMARKER_MAX_RANGE = 1.0;
    public final static double SWEEPERSERVO_HOME = 0.5;
    public final static double SWEEPERSERVOL_MIN_RANGE = 0;
    public final static double SWEEPERSERVOL_MAX_RANGE = 1;
    public final static double SWEEPERSERVOR_MIN_RANGE = 0;
    public final static double SWEEPERSERVOR_MAX_RANGE = -1;


    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    /* Constructor */
    public Hardware2858() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Mecanum Motors
        leftDriveOUT  = hwMap.get(DcMotor.class, "left_drive_out");
        leftDriveIN  = hwMap.get(DcMotor.class, "left_drive_in");
        rightDriveOUT = hwMap.get(DcMotor.class, "right_drive_out");
        rightDriveIN = hwMap.get(DcMotor.class, "right_drive_in");
        liftDrive = hwMap.get(DcMotor.class, "lift_drive");
        sweeperDrive = hwMap.get(DcMotor.class, "sweeper_drive");
        gripperDrive = hwMap.get(DcMotor.class, "gripper_drive");
        leftDriveIN.setDirection(DcMotor.Direction.REVERSE);
        rightDriveOUT.setDirection(DcMotor.Direction.REVERSE);
        gripperDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDriveOUT.setPower(0);
        leftDriveIN.setPower(0);
        rightDriveOUT.setPower(0);
        rightDriveIN.setPower(0);
        liftDrive.setPower(0);
        sweeperDrive.setPower(0);
        gripperDrive.setPower(0);

        // Set all mecanum motors to run using encoders.
        leftDriveOUT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveIN.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveOUT.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveIN.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sweeperDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gripperDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        teamMarker  = hwMap.get(Servo.class, "team_Marker");
        teamMarker.setPosition(TEAMMARKER_HOME);
        sweeperServoL = hwMap.get(Servo.class, "sweeper_Servo_L");
        sweeperServoR = hwMap.get(Servo.class, "sweeper_Servo_R");
    }
}
