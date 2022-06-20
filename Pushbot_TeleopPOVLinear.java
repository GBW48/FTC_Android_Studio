package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

//package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcontroller.external.samples.Pushbot_Hardware;

/*
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list

FTC Game Controller Reference
  https://docs.google.com/document/d/12gQTTJqDPs2JtyxIzylWSq6nD5qabHyXXto0CeTLYAA/edit

"x" Returns true if X button is pressed, false otherwise.
"y" Returns true if Y button is pressed, false otherwise.
"b" Returns true if B button is pressed, false otherwise.
"a" Returns true if A button is pressed, false otherwise.

"dpad_down" Returns true if D-pad Down is pressed, false otherwise.
"dpad_left" Returns true if D-pad Left is pressed, false otherwise.
"dpad_right" Returns true if D-pad Right is pressed, false otherwise.
"dpad_up" Returns true if D-pad Up is pressed, false otherwise

"left_trigger" Returns range of value 0.0 to 1.0 as right trigger is pressed.
"right_trigger" Returns range of value 0.0 to 1.0 as right trigger is pressed.
"left_bumper" Returns true if the left bumper is pressed, false otherwise.
"right_bumper" Returns true if the right bumper is pressed, false otherwise.

"back" Returns true if Back button is pressed, false otherwise.
"start" Returns true if the Start button is pressed, false otherwise.
"guide" or "mode" Returns true if the Guide/Mode button is pressed, false otherwise.

"left_stick_x" Returns the left-right deflection of the left stick.
  Negative values represent left deflections & positive right deflections. Range is -1.0 to +1.0.

"left_stick_y" Returns the up-down deflection of the left stick.
  Negative values represent up deflections & positive values down deflections. Range is -1.0 to +1.0.

"right_stick_x" Returns the left-right deflection of the right stick.
  Negative values represent left deflections & positive right deflections. Range is -1.0 to +1.0.

"right_stick_y" Returns the up-down deflection of the right stick,
  Negative values represent up deflections & positive values down deflections. Range is -1.0 to +1.0.

"left_stick_button" Returns true if the left stick button is pressed, false otherwise.

"right_stick_button" Returns true if the right stick button is pressed, false otherwise.

"atRest" Returns true if joys sticks and triggers in neutral position, false otherwise.
  atRest is a property you can access in your programming - not actually a “button” on the controller
*/

@TeleOp(name="Pushbot: Teleop POV", group="Pushbot")
    //@Disabled
    public class Pushbot_TeleopPOVLinear extends LinearOpMode {

        /* Declare OpMode members. */
        Pushbot_Hardware robot          = new Pushbot_Hardware();   // Use a Pushbot's hardware
    final double    SERVO_Speed     = 0.02 ;                   // sets rate to move servo*/

    @Override
    public void runOpMode() {
        double left;
        double right;
        double drive;
        double turn;
        double max;
        double total_crservo;
        double servo_position = .5;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Run wheels in POV mode (note: The joystick goes negative when pushed forwards, so negate it)
            // In this mode the Left stick moves the robot fwd and back, the Right stick turns left and right.
            // This way it's also easy to just drive straight, or just turn.

            drive = -gamepad1.left_stick_y;
            turn  =  gamepad1.right_stick_x;

            // Combine drive and turn for blended motion.
            left  = drive + turn;
            right = drive - turn;

            // Normalize the values so neither exceed +/- 1.0
            max = Math.max(Math.abs(left), Math.abs(right));
            if (max > 1.0)
            {
                left /= max;
                right /= max;
            }

            // Output the safe vales to the motor drives.
            robot.left_drive.setPower(left);
            robot.right_drive.setPower(right);


            //Use gamepad left & right Triggers to control rotation (CW or CCW) of cr_servo
            total_crservo = -gamepad1.left_trigger + gamepad1.right_trigger;
            robot.cr_servo.setPower(-gamepad1.left_trigger+gamepad1.right_trigger);


            //Use gamepad left & right Bumpers to control incremental rotation of servo
            //  DpadLeft  button set servo to -90 degrees (from vertical)
            //  DpadRight button set servo to +90 degrees (from vertical)
            //  Back button set servo to 0 degrees (from vertical)
            if (gamepad1.dpad_up | gamepad1.dpad_left | gamepad1.dpad_right){
                if (gamepad1.dpad_up){
                    telemetry.addData("dpad_up", "Pressed");
                    telemetry.update();
                    servo_position = 0.5;
                }
                else if (gamepad1.dpad_left)
                    servo_position = 0.0;
                else
                    // gamepad1.dpad_right is true
                    servo_position = 1.0;
                robot.servo.setPosition(servo_position);
            }

            //Use gamepad left & right Bumpers to control fixed positioning of servo
            //  Left Bumper decrease servo position by 9 degrees
            //  Right Bumper increase servo position by 9 degrees
            if (gamepad1.left_bumper | gamepad1.right_bumper){
                if (gamepad1.left_bumper){
                    servo_position -= 0.05;
                    if (servo_position < 0.0) {
                        servo_position = 0.0;
                    }
                    // Delay until left bumper button is released
                    while (gamepad1.left_bumper) {
                    }
                }
                else {
                    // gamepad1.right_bumper is true
                    servo_position += 0.05;
                    if (servo_position > 1.0) {
                        servo_position = 1.0;
                    }
                    // Delay until right bumper button is released
                    while (gamepad1.right_bumper) {
                    }
                }
                robot.servo.setPosition(servo_position);
            }


            // Send telemetry message to signify robot running;
            if (total_crservo != 0.0)
                telemetry.addData("crservo", "%.2f", total_crservo);
            else if (left != 0.0 || right != 0.0) {
                telemetry.addData("left wheel", "%.2f", left);
                telemetry.addData("right wheel", "%.2f", right);
            }
/*            else if (servo_change) {
                telemetry.addData("crservo", "%.2f", servo_position);
                servo_change = false;
            }*/
            else {
                // Display servo position
                // Note you display text comment by using  "telemetry.addData("", "%s", " ");"
                telemetry.addData("servo", "%.2f", servo_position);
            }
            telemetry.update();

            // Pace this loop so jaw action is reasonable speed.

            sleep(50);
        }
    }
}
