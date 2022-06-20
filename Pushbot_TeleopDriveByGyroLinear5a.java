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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMU;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * This file illustrates the concept of driving a path based on Gyro heading and encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a Modern Robotics I2C gyro with the name "gyro"
 *   otherwise you would use: PushbotAutoDriveByEncoder;
 *
 *  This code requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  In order to calibrate the Gyro correctly, the robot must remain stationary during calibration.
 *  This is performed when the INIT button is pressed on the Driver Station.
 *  This code assumes that the robot is stationary when the INIT button is pressed.
 *  If this is not the case, then the INIT should be performed again.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clock Wise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * Note:  This program has been modified to use the BNO055 IMU (and not the Modern Robotics I2c Gyro)
 *
 * {@link SensorBNO055IMU} gives a short demo on how to use the BNO055 Inertial Motion Unit (IMU) from AdaFruit.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://www.adafruit.com/products/2472">Adafruit IMU</a>
 */

@TeleOp(name="Pushbot: Teleop Drive By Gyro5a", group="Pushbot")
//@Disabled   //Make sure to comment out (disable) @Disable or program wont show up in list of Autonomous programs in the driver station

public class Pushbot_TeleopDriveByGyroLinear5a extends LinearOpMode {

    /* Declare OpMode members. */
    org.firstinspires.ftc.robotcontroller.external.samples.Pushbot_Hardware robot
            = new org.firstinspires.ftc.robotcontroller.external.samples.Pushbot_Hardware();   // Use a Pushbot's hardware

    // *****Start BNO055 IMU*****
    // The IMU sensor object
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;

    // *****End BNO055 IMU*****

    /*
    REV Robotics Core Hex Motor Specifications:
        Gear Ratio: 72:1
        Encoder Counts per Revolution
            At the motor - 4 counts/revolution
            At the output - 288 counts/revolution

    REV Robotics 90MM Grip Wheel Specifications:
        Diameter: 90mm/3.54in
    */

    static final double     COUNTS_PER_MOTOR_REV    = 4 ;       // REV Robotics Core Hex Motor
    static final double     DRIVE_GEAR_REDUCTION    = 72 ;      // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 3.54 ;    // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // The can/should be tweaked to suite the specific robot drive train.
    static final double     DRIVE_SPEED             = 0.7;     // Nominal speed for better accuracy.
    static final double     TURN_SPEED              = 0.5;     // Nominal half speed for better accuracy.

    static final double     HEADING_THRESHOLD       = 1 ;      // As tight as we can make it with an integer gyro
    static final double     P_TURN_COEFF            = 0.1;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_COEFF           = 0.15;     // Larger is more responsive, but also less stable

    // The constant "heading_correction" is used to adjust the "zero degree position" when the BNO055 IMU is first initialized.
    // This adjustment is necessary to account for the placement of the Control Hub (and associated IMU) relative to the midpoint
    //  between the wheels that are used for turning the robot.
    // The current compensation is -15 degrees.  For example if you want to turn 90 degrees CW, you would specify a desired angle of
    //  -90 degrees.  Assuming a heading_correction = -15, the adjusted angle to account for the placement of the Control Hub would be:
    //      angle = desired_angle - heading correction
    //      -75 = -90 - (-15)
    static final double     heading_correction      = 0.0;

/*    double                  previous_heading;*/
    double                  previous_heading;
    double                  heading;
    double                  last_left_speed;
    double                  last_right_speed;
    int                     heading_update_count    = 0;
    int                     side                    = 1;
    int                     adjust_G_180            = 0;
    int                     adjust_LE_neg180        = 0;
    boolean                 continue_with_movement  = true;


    @Override
    public void runOpMode() {

        //
        // Initialize the standard drive system variables.
        // The init() method of the hardware class does most of the work here
        //

        robot.init(hardwareMap);

        // *****Start BNO055 IMU*****
        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "gyro");
        imu.initialize(parameters);

        // *****End BNO055 IMU*****

        // Wait until we're told to go
        waitForStart();

        // Start the logging of measured acceleration
        sleep(3000);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        previous_heading = heading;

        // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
        robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while (continue_with_movement){

            // Step through each leg of the path,
            // Notes:   Reverse movement is obtained by setting a negative distance (not speed)
            //          All angles are relative to initial IMU calivration
            //          Negative angle turns CW
            //          Positve angle turns CCW


            telemetry.addLine()
                    .addData("Status", new Func<String>() {
                        @Override public String value() {
                            return imu.getSystemStatus().toShortString();
                        }
                    })
                    .addData("Calib", new Func<String>() {
                        @Override public String value() {
                            return imu.getCalibrationStatus().toString();
                        }
                    });


            telemetry.addData("Press A", "To test heading correction");
            telemetry.update();

            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, 0.0 [0 degrees]");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, 0.0);                 // Turn  0 Degrees
            //***********


            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, 75.0 [90 degrees CW]");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -75.0);                 // Turn  CCW to 90 Degrees
            //***********

            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, 75.0 [90 degrees CW]");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, 75.0);                 // Turn  CCW to 0 Degrees
            //***********

            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, 75.0 [90 degrees CCW]");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, 75.0);                 // Turn  CW to 270 Degrees
            //***********

            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -150.0 [180 degrees CW]");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -150.0);                 // Turn  CCW to 90 Degrees
            //***********



            telemetry.addData("Press A", "To drive in an 12 inch square");
            telemetry.update();

            while (!gamepad1.a) {
            }
            while (side != 5) {
                telemetry.addData("Press A: gyroDrive(DRIVE_SPEED, 12.0, 0.0) for side #","%d", side);
                telemetry.update();
                while(!gamepad1.a){
                }
                gyroDrive(DRIVE_SPEED, 12.0, 0.0);    // Drive FWD 8 inches
                //***********
                telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -75.0 [90 degrees CW]");
                telemetry.update();
                while(!gamepad1.a){
                }
                gyroTurn(TURN_SPEED, -75.0);                 // Turn  CW to 90 Degrees
                //***********
                side++;
            }
/*          //***********
            telemetry.addData("Press A", "To drive in an 8 inch square");
            telemetry.update();
            while (!gamepad1.a) {
            }
            gyroDrive(DRIVE_SPEED, 8.0, 0.0);    // Drive FWD 8 inches
             //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -60.0); #1");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -60.0);         // Turn  CCW to 90 Degrees
            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -60.0); #2");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -60.0);         // Turn  CCW to 90 Degrees
            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -60.0); #3");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -60.0);         // Turn  CCW to 90 Degrees
            //***********
            telemetry.addData("Press A", "gyroTurn(TURN_SPEED, -60.0); #4");
            telemetry.update();
            while(!gamepad1.a){
            }
            gyroTurn(TURN_SPEED, -60.0);         // Turn  CCW to 90 Degrees*/

            telemetry.addData("Press A", "To repeat movements");
            telemetry.addData("Press B", "To stop program");
            telemetry.update();
            while(!gamepad1.a && !gamepad1.b){
            }
            if (gamepad1.a){
                // Start the logging of measured acceleration
                sleep(3000);
                imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
                angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                heading = angles.firstAngle;
                previous_heading = heading;

                // Ensure the robot it stationary, then reset the encoders and calibrate the gyro.
                robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Reset side count for drawing of square
                side = 1;

                // Remove topmost status line from Telemetry data
                telemetry.clearAll();

                // Restart the robot movement again
                continue;
            }
            if (gamepad1.b){
                continue_with_movement = false;
            }
        }
    stop();
    }

   /**
    *  Method to drive on a fixed compass bearing (angle), based on encoder counts.
    *  Move will stop if either of these conditions occur:
    *  1) Move gets to the desired position
    *  2) Driver stops the opmode running.
    *
    * @param speed      Target speed for forward motion.  Should allow for _/- variance for adjusting heading
    * @param distance   Distance (in inches) to move from current position.  Negative distance means move backwards.
    * @param angle      Absolute Angle (in Degrees) relative to last gyro reset.
    *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
    *                   If a relative angle is required, add/subtract from current heading.
    */
    public void gyroDrive ( double speed, double distance, double angle) {
        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error           = 0.0;
        double  steer           = 0.0;
        double  leftSpeed       = 0.0;
        double  rightSpeed      = 0.0;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(1500);
            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1500);
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            previous_heading = heading;
            adjust_G_180 = 0;
            adjust_LE_neg180 = 0;

            /*            previous_heading = heading;*/

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * COUNTS_PER_INCH);
            newLeftTarget = robot.left_drive.getCurrentPosition() + moveCounts;
            newRightTarget = robot.right_drive.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            robot.left_drive.setTargetPosition(newLeftTarget);
            robot.right_drive.setTargetPosition(newRightTarget);

            robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            robot.left_drive.setPower(speed);
            robot.right_drive.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while (opModeIsActive() &&
                   (robot.left_drive.isBusy() && robot.right_drive.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, P_DRIVE_COEFF);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0) {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                robot.left_drive.setPower(leftSpeed);
                robot.right_drive.setPower(rightSpeed);

/*                // Display drive status for the driver.
                telemetry.addData("Error / Steer",  "%5.1f / %5.1f", error, steer);
                telemetry.addData("New_LTarget / New_RTarget",  "%7d / %7d", newLeftTarget,  newRightTarget);
                telemetry.addData("LDrive / RDrive",  "%7d / %7d", robot.left_drive.getCurrentPosition(),
                                                                                 robot.right_drive.getCurrentPosition());
                telemetry.addData("LSpeed / RSpeed",   "%5.2f: / %5.2f", leftSpeed, rightSpeed);
                telemetry.update();*/
            }

            // Display drive status for the driver.
            telemetry.addData("Error / Steer",  "%5.1f / %5.1f", error, steer);
            telemetry.addData("New_LTarget / New_RTarget",  "%7d / %7d", newLeftTarget,  newRightTarget);
            telemetry.addData("LDrive / RDrive",  "%7d / %7d", robot.left_drive.getCurrentPosition(),
                                                                             robot.right_drive.getCurrentPosition());
            telemetry.addData("LSpeed / RSpeed",   "%5.2f: / %5.2f", leftSpeed, rightSpeed);

            // Stop all motion;
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(1500);
            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1500);

            // Turn off RUN_TO_POSITION
        }
    }


    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param speed Desired speed of turn.
     * @parm begin_ramp_down  Percent of turn, when to start ramping down the motor seed.
     *                        Once the ramp-down percent is reached, the motor speed will
     *                        be reduced by 20% for every 20% decrease of the remaining amount
     *                        to be turn (error).  This reduction will continue until the
     *                        min_ramp_down_speed is reached
     * @parm min_ramp_down_speed  Minimum ramp-down speed.
     * @param angle      Relative Angle (in Degrees). Relative to the last gyroTurn or gyroDrive.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     */
/*    public void gyroTurn (  double speed, double angle, double begin_ramp_down, double min_ramp_down_speed) {*/
    public void gyroTurn (  double speed, double angle) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(1500);
            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1500);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            heading = angles.firstAngle;
            previous_heading = heading;
            adjust_G_180 = 0;
            adjust_LE_neg180 = 0;


            // keep looping while we are still active, and not on heading.
            while (opModeIsActive() && !onHeading(speed, angle, P_TURN_COEFF)) {
                // Update telemetry & Allow sleep for other processes to run.
                telemetry.update();
            }

            // Stop all motion
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            sleep(1500);
            robot.left_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(1500);
        }
    }

    /** Perform one cycle of closed loop heading control.
     *
     * @param speed     Desired speed of turn.
     * @param angle     Absolute Angle (in Degrees) relative to last gyro reset.
     *                  0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                  If a relative angle is required, add/subtract from current heading.
     * @param PCoeff    Proportional Gain coefficient
     * @return
     */


    boolean onHeading(double speed, double angle, double PCoeff) {
        double   error ;
        double   steer ;
        boolean  onTarget = false ;
        double leftSpeed;
        double rightSpeed;

        // determine turn power based on +/- error
        error = getError(angle);

        if (Math.abs(error) <= HEADING_THRESHOLD) {
            steer = 0.0;
            leftSpeed  = 0.0;
            rightSpeed = 0.0;
            onTarget = true;
        }
        else {
            steer = getSteer(error, PCoeff);
            rightSpeed  = speed * steer;
            last_right_speed = rightSpeed;
            leftSpeed   = -rightSpeed;
            last_left_speed = leftSpeed;
        }

        // Send desired speeds to motors.
        robot.left_drive.setPower(leftSpeed);
        robot.right_drive.setPower(rightSpeed);

        // Display it for the driver.
        telemetry.addData("Target", "%5.2f", angle);
/*        telemetry.addData("adjust_G_180 / adjust_LE_neg180", "%3d / %3d", adjust_G_180 , adjust_LE_neg180);*/
        telemetry.addData("P_heading / C_Heading # H_count", "%5.2f / %5.2f # %3d", previous_heading, heading, heading_update_count);
        telemetry.addData("Error / Steer", "%5.2f/%5.2f", error, steer);
        telemetry.addData("RSpeed = Speed * Steer", "%5.2f = %5.2f * %5.2f", rightSpeed, speed, steer);
        telemetry.addData("LSpeed / RSpeed", "%5.2f:%5.2f", leftSpeed, rightSpeed);
        
/*        if ((adjust_LE_neg180 > 0 || adjust_LE_neg180 > 0)){
            telemetry.addData("Press A", "Heading adjustment");
            telemetry.addData("adjust_G_180 / adjust_LE_neg180", "%3d / %3d", adjust_G_180 , adjust_LE_neg180);
            telemetry.update();
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);
            while(!gamepad1.a){
            }
            robot.left_drive.setPower(leftSpeed);
            robot.right_drive.setPower(rightSpeed);
        }*/
        return onTarget;
    }


    /**
     * getError determines the error between the target angle and the robot's current heading
     * @param   targetAngle  Desired angle (relative to global reference established at last Gyro Reset).
     * @return  error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     *          +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        heading = angles.firstAngle;
        /*robotError = targetAngle  - heading;*/
        robotError = targetAngle - heading_correction + previous_heading - heading;
        heading_update_count += 1;
        while (robotError > 180) {
            robotError -= 360;
            adjust_G_180++;
        }
        while (robotError <= -180){
            robotError += 360;
            adjust_LE_neg180++;
        }
        return robotError;
    }

    /**
     * returns desired steering force.  +/- 1 range.  +ve = steer left
     * @param error   Error angle in robot relative degrees
     * @param PCoeff  Proportional Gain Coefficient
     * @return
     */
    public double getSteer(double error, double PCoeff) {
        return Range.clip(error * PCoeff, -1, 1);
    }

}
