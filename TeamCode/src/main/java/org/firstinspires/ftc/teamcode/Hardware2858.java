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
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to open/close clampTopLeft: "clampTopLeft"
 * Servo channel: Servo to open/close clampBottomLeft: "clampBottomLeft
 * Servo channel:  Servo to open/close clampTopRight: "clampTopRight"
 * Servo channel:  Servo to open/close clampBottomRight: "clampBottomRight"
 */
public class Hardware2858
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  clampLift   = null;
    public Servo    clampTopLeft         = null;
    public Servo    clampBottomLeft     = null;
    public Servo    clampTopRight        = null;
    public Servo    clampBottomRight      = null;
    public Servo    colorArm               = null;

    public final static double CLAMPTOPLEFT_HOME = 0.20;
    public final static double CLAMPBOTTOMLEFT_HOME = 0.20;
    public final static double CLAMPTOPRIGHT_HOME = 0.80;
    public final static double CLAMPBOTTOMRIGHT_HOME = 0.80;
    public final static double COLORARM_HOME    = 1.00;

    //Mecanum Wheels
    public DcMotor  frontLeftDrive   = null;
    public DcMotor  frontRightDrive  = null;
    public DcMotor  backLeftDrive   = null;
    public DcMotor  backRightDrive  = null;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    public double LEFT_HOME;
    public double RIGHT_HOME;
    public double LIFT_HOME;

    /* Constructor */
    public Hardware2858() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        clampLift  = hwMap.get(DcMotor.class, "clamp_lift");
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Define and Initialize Mecanum Motors
        frontLeftDrive  = hwMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hwMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hwMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hwMap.get(DcMotor.class, "back_right_drive");

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        clampLift.setPower(0);

        // Set all mecanum motors to zero power
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clampLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all mecanum motors to run without encoders.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        clampTopLeft  = hwMap.get(Servo.class, "clampTopLeft");
        clampBottomLeft  = hwMap.get(Servo.class, "clampBottomLeft");
        clampTopRight = hwMap.get(Servo.class, "clampTopRight");
        clampBottomRight = hwMap.get(Servo.class, "clampBottomRight");
        colorArm = hwMap.get(Servo.class, "colorArm");
        clampTopLeft.setPosition(0.10);
        clampBottomLeft.setPosition(0.10);
        clampTopRight.setPosition(0.90);
        clampBottomRight.setPosition(0.90);
        colorArm.setPosition(1.00);
    }
}
