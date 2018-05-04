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

public class TurnerHardware2858
{
    // Public OpMode members.

    //Arm
    public Servo armTT              = null;
    public final static double ARMTT_HOME = 1.00;

    //Mecanum Wheels
    public DcMotor  LeftDrive  = null;
    public DcMotor  RightDrive = null;
    public DcMotor  LiftMotor   = null;
    public DcMotor  StringMotor  = null;

    /* Local OpMode members. */
    HardwareMap ThwMap  = null;

    /* Constructor */
    public TurnerHardware2858() {
    }

    // Initialize standard Hardware interfaces
    public void init(HardwareMap TahwMap) {
        // save reference to HW Map
        ThwMap = TahwMap;

        // Define and Initialize Mecanum Motors
        LeftDrive  = ThwMap.get(DcMotor.class, "left_drive");
        RightDrive = ThwMap.get(DcMotor.class, "right_drive");
        LiftMotor  = ThwMap.get(DcMotor.class, "back_left_drive");
        StringMotor = ThwMap.get(DcMotor.class, "string_motor");
        RightDrive.setDirection(DcMotor.Direction.REVERSE);
        StringMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all mecanum motors to zero power
        LeftDrive.setPower(0);
        RightDrive.setPower(0);
        LiftMotor.setPower(0);
        StringMotor.setPower(0);

        // Set all mecanum motors to run without encoders.
        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        StringMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Define and initialize all installed servos.
        armTT = ThwMap.get(Servo.class, "armTT");
        armTT.setPosition(1.00);
    }
}
