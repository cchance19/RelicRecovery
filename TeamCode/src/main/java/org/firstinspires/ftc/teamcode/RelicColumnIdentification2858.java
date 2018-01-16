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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
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


/**
 * This OpMode illustrates the basics of using the Vuforia engine to determine
 * the identity of Vuforia VuMarks encountered on the field. The code is structured as
 * a LinearOpMode. It shares much structure with {@link ConceptVuforiaNavigation}; we do not here
 * duplicate the core Vuforia documentation found there, but rather instead focus on the
 * differences between the use of Vuforia for navigation vs VuMark identification.
 *
 * @see ConceptVuforiaNavigation
 * @see VuforiaLocalizer
 * @see VuforiaTrackableDefaultListener
 * see  ftc_app/doc/tutorial/FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained in {@link ConceptVuforiaNavigation}.
 */

@Autonomous(name="Relic Key ID", group ="Autonomous")
public class RelicColumnIdentification2858 extends LinearOpMode {

    // Declare OpMode Members.
    Hardware2858    robot               = new Hardware2858();   // Use our robot's hardware
    double          colorArmPosition    = robot.COLORARM_HOME;

    public static final String TAG = "Vuforia VuMark Sample";

    private ElapsedTime runtime = new ElapsedTime();

    OpenGLMatrix lastLocation = null;

    /*
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    VuforiaLocalizer vuforia;

    @Override public void runOpMode() {

        robot.init(hardwareMap);

        //To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
        //If no camera monitor is desired, use the parameterless constructor instead (commented out below).

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

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        waitForStart();

        relicTrackables.activate();

        while (opModeIsActive()) {

            /*
             * See if any of the instances of {@link relicTemplate} are currently visible.
             * {@link RelicRecoveryVuMark} is an enum which can have the following values:
             * UNKNOWN, LEFT, CENTER, and RIGHT. When a VuMark is visible, something other than
             * UNKNOWN will be returned by {@link RelicRecoveryVuMark#from(VuforiaTrackable)}.
             */
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                     if (vuMark == RelicRecoveryVuMark.LEFT) {

                         //Moves robot to safe zone
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(0.5);
                         while (opModeIsActive() && (runtime.seconds() < 2.0)) {
                             telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
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

                         //Moves forward to cryptobox
                         robot.leftDrive.setPower(0.5);
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         //Push block in
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         robot.leftDrive.setPower(-0.5);
                         robot.rightDrive.setPower(-0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         telemetry.addData("Visible:", "LEFT", vuMark);

                         stop();
                     }










                else if (vuMark == RelicRecoveryVuMark.CENTER) {

                         //Moves robot to safe zone
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(-0.5);
                         while (opModeIsActive() && (runtime.seconds() < 2.5)) {
                             telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
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

                         //Moves forward to cryptobox
                         robot.leftDrive.setPower(0.5);
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         //Push block in
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         robot.leftDrive.setPower(-0.5);
                         robot.rightDrive.setPower(-0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         telemetry.addData("Visible:", "CENTER", vuMark);
                     }










                else if (vuMark == RelicRecoveryVuMark.RIGHT) {

                         //Moves robot to safe zone
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(0.5);
                         while (opModeIsActive() && (runtime.seconds() < 3.0)) {
                             telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
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

                         //Moves forward to cryptobox
                         robot.leftDrive.setPower(0.5);
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
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
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         //Push block in
                         robot.leftDrive.setPower(0.5);
                         robot.rightDrive.setPower(0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         robot.leftDrive.setPower(-0.5);
                         robot.rightDrive.setPower(-0.5);
                         runtime.reset();
                         while (opModeIsActive() && (runtime.seconds() < 0.5)) {
                             telemetry.addData("Path", "Leg 1R: %2.5f S Elapsed", runtime.seconds());
                             telemetry.update();
                         }

                         robot.leftDrive.setPower(0);
                         robot.rightDrive.setPower(0);

                         telemetry.addData("VuMark", "RIGHT", vuMark);
                     }


                /* Found an instance of the template. In the actual game, you will probably
                 * loop until this condition occurs, then move on to act accordingly depending
                 * on which VuMark was visible. */
                telemetry.addData("VuMark", "%s visible", vuMark);

                /* For fun, we also exhibit the navigational pose. In the Relic Recovery game,
                 * it is perhaps unlikely that you will actually need to act on this pose information, but
                 * we illustrate it nevertheless, for completeness. */
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                telemetry.addData("Pose", format(pose));

                /* We further illustrate how to decompose the pose into useful rotational and
                 * translational components */
                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
            }
            else {
                telemetry.addData("VuMark", "not visible");
            }

            telemetry.update();
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
