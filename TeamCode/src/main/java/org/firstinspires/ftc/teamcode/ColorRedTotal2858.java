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

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */

@Autonomous(name = "ColorRedTotal2858", group = "ColorRedTotal2858")
public class ColorRedTotal2858 extends LinearOpMode {

    /* Declare OpMode Members */
    Hardware2858 robot = new Hardware2858();   // Use a Pushbot's hardware
    double colorArmPosition = robot.COLORARM_HOME;

    private ElapsedTime runtime = new ElapsedTime();

    //static final double     COUNTS_PER_MOTOR_REV    = 14 ;    // eg: TETRIX Motor Encoder
    //static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    //static final double     WHEEL_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    //static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
         //   (WHEEL_DIAMETER_INCHES * 3.1415);
    //static final double     DRIVE_SPEED             = 0.6;
    //static final double     TURN_SPEED              = 0.5;


    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    /**
     * Note that the REV Robotics Color-Distance incorporates two sensors into one device.
     * It has a light/distance (range) sensor.  It also has an RGB color sensor.
     * The light/distance sensor saturates at around 2" (5cm).  This means that targets that are 2"
     * or closer will display the same value for distance/light detected.
     * <p>
     * Although you configure a single REV Robotics Color-Distance sensor in your configuration file,
     * you can treat the sens5r as two separate sensors that share the same name in your op mode.
     * <p>
     * In this example, we represent the detected color by a hue, saturation, and value color
     * model (see https://en.wikipedia.org/wiki/HSL_and_HSV).  We change the background
     * color of the screen to match the detected color.
     * <p>
     * In this example, we  also use the distance sensor to display the distance
     * to the target object.  Note that the distance sensor saturates at around 2" (5 cm).
     */
    ColorSensor sensorColor;

    @Override
    public void runOpMode() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // get a reference to the color sensor.
        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = "ATpG7Mr/////AAAAGUhsGTaz1UNuoy/bR4iCcfJ5mifTu9M4neN3iASx8Rt/G0IWTrIIcean0W2ulGksyTai0ovlOG8vo8JfF34gSIlSI1vyatjot23Mxc9i9KHNSMtqh/XJF3YQzyzb8Eq7Rql1jn60FXEP6UIlzlVStnvxJkrxpx8Tz6eA+KdF4G28nNf0NTsVFV6wDu5I/5QMZoYxBpya+piBGhCgGignAw80R+G2a0OKYLsoRqPdk1CqTaHzIUiWrSCg8z+YNiNLBBEMppsDCBqBLkBlzKv16jYsWlPPrL/RB/VpUonZDWSD+naol++ABySTxPxkfuhxwde1xdeHIpnaiWGEKsiA2D+8d6MkfzkQJw6mFH3jq6AM";

        /*
         * We also indicate which camera on the RC that we wish to use.
         * Here we chose the back (HiRes) camera (for greater range), but
         * for a competition robot, the front camera might be more convenient.
         */
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        /*
         * Load the data set containing the VuMarks for Relic Recovery. There's only one trackable
         * in this data set: all three of the VuMarks in the game were created from this one template,
         * but differ in their instance id information.
         * @see VuMarkInstanceId
         */
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        // Send telemetry message to signify robot waiting;
        //telemetry.addData("Status", "Resetting Encoders");    //
        //telemetry.update();

        //robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0",  "Starting at %7d :%7d",
               // robot.leftDrive.getCurrentPosition(),
               // robot.rightDrive.getCurrentPosition());
       // telemetry.update();

        waitForStart();


        //Set color sensor arm to new position
        robot.colorArm.setPosition(0.33);


        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {

            runtime.reset();

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorColor.red() * SCALE_FACTOR),
                    (int) (sensorColor.green() * SCALE_FACTOR),
                    (int) (sensorColor.blue() * SCALE_FACTOR),
                    hsvValues);

            // send the info back to driver station using telemetry function.
            // Send telemetry message to signify robot or.red());
            telemetry.addData("Green", sensorColor.green());
            telemetry.addData("Red  ", sensorColor.red());
            telemetry.addData("Blue ", sensorColor.blue());
            telemetry.addData("Hue", hsvValues[0]);
            telemetry.addData("colorArm", "%.2f", colorArmPosition);

            if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                forwardJewel();

            } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                backwardJewel();



            } else if (runtime.seconds() > 5.00) {

                robot.colorArm.setPosition(1.00);

                //Reposition robot to force read
                robot.leftDrive.setPower(0.25);
                robot.rightDrive.setPower(0.25);
                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 0.25)) {
                    telemetry.addData("Path", "Leg 7: %2.5f S Elapsed", runtime.seconds());
                    telemetry.update();
                }

                robot.leftDrive.setPower(0);
                robot.rightDrive.setPower(0);

                robot.colorArm.setPosition(0.33);


                if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                    forwardJewel();


                } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                    backwardJewel();


                }
            }
                relicTrackables.activate();

                runtime.reset();
                while (opModeIsActive() && (runtime.seconds() < 30)) {
            /*
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
                    RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                    if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                        if (vuMark == RelicRecoveryVuMark.LEFT) {

                            if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                forwardJewel();

                            } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                backwardJewel();



                            } else if (runtime.seconds() > 5.00) {

                                robot.colorArm.setPosition(1.00);

                                //Reposition robot to force read
                                robot.leftDrive.setPower(0.25);
                                robot.rightDrive.setPower(0.25);
                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < 0.25)) {
                                    telemetry.addData("Path", "Leg 7: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                robot.leftDrive.setPower(0);
                                robot.rightDrive.setPower(0);

                                robot.colorArm.setPosition(0.33);


                                if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                    forwardJewel();


                                } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                    backwardJewel();


                                }
                            }


                            //Moves robot to safe zone
                            //Turns robot
                            //Moves forward to cryptobox
                            //encoderDrive(DRIVE_SPEED, 24, 24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                            //encoderDrive(TURN_SPEED, -12, 12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            //encoderDrive(DRIVE_SPEED, 18, 18, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                                telemetry.addData("Path", "Leg 2R: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Turns robot
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Drop block
                            robot.clampLift.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 14: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.clampLift.setPower(0);

                            robot.clampTopLeft.setPosition(0.1);
                            robot.clampTopRight.setPosition(0.9);
                            robot.clampBottomLeft.setPosition(0.1);
                            robot.clampBottomRight.setPosition(0.9);

                            //Back up from Block
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 15: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Push block in
                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.75)) {
                                telemetry.addData("Path", "Leg 16: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 17: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            telemetry.addData("Visible:", "LEFT", vuMark);

                            stop();
                        } else if (vuMark == RelicRecoveryVuMark.CENTER) {


                            if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                forwardJewel();

                            } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                backwardJewel();



                            } else if (runtime.seconds() > 5.00) {

                                robot.colorArm.setPosition(1.00);

                                //Reposition robot to force read
                                robot.leftDrive.setPower(0.25);
                                robot.rightDrive.setPower(0.25);
                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < 0.25)) {
                                    telemetry.addData("Path", "Leg 7: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                robot.leftDrive.setPower(0);
                                robot.rightDrive.setPower(0);

                                robot.colorArm.setPosition(0.33);


                                if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                    forwardJewel();


                                } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                    backwardJewel();


                                }
                            }

                            // encoderDrive(DRIVE_SPEED, 30, 30, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                            //encoderDrive(TURN_SPEED, -12, 12, 4.0);  // S2: Turn left 12 Inches with 4 Sec timeout
                            //encoderDrive(DRIVE_SPEED, 18, 18, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                                telemetry.addData("Path", "Leg 2R: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Turns robot
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 1.5)) {
                                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Drop block
                            robot.clampLift.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 18: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.clampLift.setPower(0);

                            robot.clampTopLeft.setPosition(0.1);
                            robot.clampTopRight.setPosition(0.9);
                            robot.clampBottomLeft.setPosition(0.1);
                            robot.clampBottomRight.setPosition(0.9);

                            //Back up from Block
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 19: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Push block in
                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.75)) {
                                telemetry.addData("Path", "Leg 20: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 21: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            telemetry.addData("Visible:", "CENTER", vuMark);

                            stop();
                        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {


                            if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                forwardJewel();

                            } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                backwardJewel();



                            } else if (runtime.seconds() > 5.00) {

                                robot.colorArm.setPosition(1.00);

                                //Reposition robot to force read
                                robot.leftDrive.setPower(0.25);
                                robot.rightDrive.setPower(0.25);
                                runtime.reset();
                                while (opModeIsActive() && (runtime.seconds() < 0.25)) {
                                    telemetry.addData("Path", "Leg 7: %2.5f S Elapsed", runtime.seconds());
                                    telemetry.update();
                                }

                                robot.leftDrive.setPower(0);
                                robot.rightDrive.setPower(0);

                                robot.colorArm.setPosition(0.33);


                                if (sensorColor.red() > sensorColor.blue() && sensorColor.red() > 20 && sensorColor.blue() < 100) {

                                    forwardJewel();


                                } else if (sensorColor.red() < sensorColor.blue() && sensorColor.blue() > 20 && sensorColor.red() < 100) {

                                    backwardJewel();


                                }
                            }

                            // encoderDrive(DRIVE_SPEED, 36, 36, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                            // encoderDrive(TURN_SPEED, -12, 12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                            // encoderDrive(DRIVE_SPEED, 18, 18, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                                telemetry.addData("Path", "Leg 2R: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Turns robot
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 1.0)) {
                                telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Drop block
                            robot.clampLift.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 22: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.clampLift.setPower(0);

                            robot.clampTopLeft.setPosition(0.1);
                            robot.clampTopRight.setPosition(0.9);
                            robot.clampBottomLeft.setPosition(0.1);
                            robot.clampBottomRight.setPosition(0.9);

                            //Back up from Block
                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 23: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            //Push block in
                            robot.leftDrive.setPower(0.5);
                            robot.rightDrive.setPower(0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.75)) {
                                telemetry.addData("Path", "Leg 24: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            robot.leftDrive.setPower(-0.5);
                            robot.rightDrive.setPower(-0.5);
                            runtime.reset();
                            while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                                telemetry.addData("Path", "Leg 25: %2.5f S Elapsed", runtime.seconds());
                                telemetry.update();
                            }

                            robot.leftDrive.setPower(0);
                            robot.rightDrive.setPower(0);

                            telemetry.addData("VuMark", "RIGHT", vuMark);

                            stop();
                        }
                    }
                }
            }


            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();

            // Set the panel back to the default color
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.WHITE);
                }
            });

            // Range clip for servo that color sensor sits on
            colorArmPosition = Range.clip(colorArmPosition, 0.50, 1.00);
            robot.colorArm.setPosition(colorArmPosition);
        }

public void forwardJewel() {

    //Picks up block
    robot.clampTopLeft.setPosition(0.8);
    robot.clampTopRight.setPosition(0.3);
    robot.clampBottomLeft.setPosition(0.8);
    robot.clampBottomRight.setPosition(0.3);

    //Picks up block
    robot.clampLift.setPower(-0.5);
    runtime.reset();
    while (opModeIsActive() && (runtime.seconds() < 0.5)) {
        telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
    }

    robot.clampLift.setPower(0);

    //Moves robot to knock off jewel
    robot.leftDrive.setPower(-0.5);
    robot.rightDrive.setPower(-0.5);
    runtime.reset();
    while (opModeIsActive() && (runtime.seconds() < 0.35)) {
        telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
    }

    robot.leftDrive.setPower(0);
    robot.rightDrive.setPower(0);
    robot.colorArm.setPosition(1.00);

    //Moves robot back
    robot.leftDrive.setPower(0.5);
    robot.rightDrive.setPower(0.5);
    runtime.reset();
    while (opModeIsActive() && (runtime.seconds() < 0.35)) {
        telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
        telemetry.update();
    }
}

    public void backwardJewel() {

        //Picks up block
        robot.clampTopLeft.setPosition(0.8);
        robot.clampTopRight.setPosition(0.3);
        robot.clampBottomLeft.setPosition(0.8);
        robot.clampBottomRight.setPosition(0.3);

        //Picks up block
        robot.clampLift.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 4: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.clampLift.setPower(0);

        //Moves robot to knock off jewel
        robot.leftDrive.setPower(0.5);
        robot.rightDrive.setPower(0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 5: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.colorArm.setPosition(1.00);

        //Moves robot back
        robot.leftDrive.setPower(-0.5);
        robot.rightDrive.setPower(-0.5);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5)) {
            telemetry.addData("Path", "Leg 6: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
/*
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.leftDrive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.rightDrive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
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
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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
    */
}
