/* Copyright (c) 2022 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java.
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes five motors (left_front_drive, left_back_drive, right_front_drive, right_back_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 */

public class RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode;   // gain access to methods in the calling OpMode.
    private final ElapsedTime runtime = new ElapsedTime();

    // Define Motor and Servo objects (Make them private, so they can't be accessed externally)
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive  = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo   leftHand = null;
    private Servo   rightHand = null;

    // Define Drive constants.  Make them public, so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5 ;
    public static final double HAND_SPEED = 0.02 ; // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45 ;
    public static final double ARM_DOWN_POWER = -0.45 ;

    // Define Encoder constants. Make them public, so that CAN be used by the calling OpMode
    public static final double COUNTS_PER_MOTOR_REV = 1120 ; // NeverRest Orbital 40
    public static final double DRIVE_GEAR_REDUCTION = 1.0 ; // No External Gearing.
    public static final double WHEEL_DIAMETER_INCHES = 4 ;
    public static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    public static final double AUTO_DRIVE_SPEED = 0.6;
    public static final double AUTO_TURN_SPEED = 0.5;

    // Define Vision Constants. (Make them private, so they can't be accessed externally)
    public OpenCvWebcam webcam;

    public static final Scalar BLUE = new Scalar(0, 0, 255);
    public static final Scalar GREEN = new Scalar(0, 255, 0);
    public static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150,440);
    public static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(520,400);
    public static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(920,440);
    public static final int REGION_WIDTH = 200;
    public static final int REGION_HEIGHT = 20;

    //Points which actually define a simple region rectangles, derived from above values.
    // Examples of how points A and B work to define a rectangle:
    //
    //         ------------------------------------
    //         | (0,0) Point A                    |
    //         |                                  |
    //         |                                  |
    //         |                                  |
    //         |                                  |
    //         |                                  |
    //         |                                  |
    //         |                  Point B (70,50) |
    //         ------------------------------------
    private static final Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
    private static final Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    private static final Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
    private static final Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    private static final Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
    private static final Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x +REGION_WIDTH, REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Working variables
    public static Mat region1, region2, region3;
    public static Mat YCrCb = new Mat();
    public static Mat Cb = new Mat();
    public static Mat Cr = new Mat();
    public static int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/0 synchronization
    private static volatile BluePixelDeterminationPipeline.PixelPosition bluePosition = BluePixelDeterminationPipeline.PixelPosition.LEFT;
    private static volatile RedPixelDeterminationPipeline.PixelPosition redPosition = RedPixelDeterminationPipeline.PixelPosition.LEFT;

    public static void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }
    public static void inputToCr(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cr, 1);
    }

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn in the same direction as the motor shaft).
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send a telemetry message to indicate successful Encoder reset
        myOpMode.telemetry.addData("Starting at", "%7d: %7d :%7d :%7d",
                          leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                          rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
        myOpMode.telemetry.update();


        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        // Define and initialize ALL installed cameras elements.
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 1")
                /*, cameraMonitorViewId*/
        );
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }
            @Override
            public void onError(int errorCode) {
            }
        });

        myOpMode.telemetry.addData("Status", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Combine the joystick request for each axis-motion to determine each wheel's power.
     * Robot motions: Drive (Axial motion), Strafe (Lateral motion), and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param drive     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param strafe    Right/Left driving power (-1.0 to 1.0) +ve is right
     * @param turn      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */

    public void driveRobot(double drive, double strafe, double turn) {
        double max;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = drive + strafe + turn;
        double leftBackPower = drive - strafe + turn;
        double rightFrontPower = drive - strafe - turn;
        double rightBackPower = drive + strafe - turn;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max,Math.abs(rightBackPower));

        if (max > 1.0)
        {
            leftFrontPower /= max;
            leftBackPower /= max;
            rightFrontPower /= max;
            rightBackPower /= max;
        }

        setDrivePower(leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        // Show the elapsed game time and wheel power.
        myOpMode.telemetry.addData("Status", "Run Time: " + runtime);
        myOpMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        myOpMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        myOpMode.telemetry.update();
    }
    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param leftFrontPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param leftBackPower     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightFrontPower   Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param rightBackPower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     */
    public void setDrivePower(double leftFrontPower, double leftBackPower, double rightFrontPower, double rightBackPower) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed handOffset.
     *
     * @param handOffset opposing positions (-1.0 to 1.0) +ve
     */
    public void setHandPositions(double handOffset) {
        handOffset = Range.clip(handOffset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + handOffset);
        rightHand.setPosition(MID_SERVO - handOffset);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */

    public void encoderDrive(double speed, double leftFrontInches, double leftBackInches,
                             double rightFrontInches, double rightBackInches, double timeoutS){
        int newLeftFrontTarget;
        int newLeftBackTarget;
        int newRightFrontTarget;
        int newRightBackTarget;

        // Ensure that the OpMode is still active
        if (myOpMode.opModeIsActive()){

            //Determine new target position, as pass to motor controller
            newLeftFrontTarget = leftFrontDrive.getCurrentPosition() + (int)(leftFrontInches * COUNTS_PER_INCH);
            newLeftBackTarget = leftBackDrive.getCurrentPosition() + (int)(leftBackInches * COUNTS_PER_INCH);
            newRightFrontTarget = rightFrontDrive.getCurrentPosition() + (int)(rightFrontInches * COUNTS_PER_INCH);
            newRightBackTarget = rightBackDrive.getCurrentPosition() + (int)(rightBackInches * COUNTS_PER_INCH);

            leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            leftBackDrive.setTargetPosition(newLeftBackTarget);
            rightFrontDrive.setTargetPosition(newRightFrontTarget);
            rightBackDrive.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // Keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.
            // This is "safer" in the event that the robot will always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (myOpMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftFrontDrive.isBusy() && leftBackDrive.isBusy() &&
                            rightFrontDrive.isBusy() && rightBackDrive.isBusy())) {

                // Display it for the driver.
                myOpMode.telemetry.addData("Running to", " %7d :%7d :%7d :%7d", newLeftFrontTarget, newLeftBackTarget,
                        newRightFrontTarget, newRightBackTarget);
                myOpMode.telemetry.addData("Currently at", " at %7d :%7d :%7d :%7d",
                        leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(),
                        rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
                myOpMode.telemetry.update();
            }
            // Stop all motions:
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            myOpMode.sleep(250); // Optional pause after each move
        }
    }
    public static class BluePixelDeterminationPipeline extends OpenCvPipeline {
        public enum PixelPosition {
            LEFT, CENTER, RIGHT
        }
        @Override
        public void init(Mat firstFrame) {

            // We need to call this to make sure the 'Cb' or 'Cr' object is initialized,
            // so that the submats we make will still be linked to it on the subsequent frames.
            // (If the object were to only be initialized in processFrame,
            // then the submats would become delinked because the backing buffer would be re-allocated the first time
            // a real frame was crunched)
            inputToCb(firstFrame);

            // Submats are a persistent reference to a region of the parent buffer.
            // Any changes to the child affect the parent, and the reverse also holds true.
            region1 = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2 = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3 = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            // Overview of what we're doing:
            //
            // We first convert YCrCb color space, from RGB color space.
            // Why do we do this?
            // Well, in the RGB color space,chroma and luma are intertwined.
            // In YCrCb, chroma and luma are separated.
            // YCrCb is a 3-channel color space, just like RGB.
            // TCrCb's 3 channels are Y,
            // the luma channel (which is essentially just a B&W image), the Cr channel,
            // which records the difference from red, and the Cb channel,
            // which records the difference from blue.
            // Because chroma and luma are not related in YCrCb, vision code
            // written to look for certain values in the Cr/Cb channels will not be severely affected by differing light intensity,
            // since that difference would most likely just be reflected in the Y channel.
            //
            // After we've converted to YCrCb, we extract just the 2nd channel, the Cb channel.
            //
            // We then take the average pixel value of 3 different regions on that Cb channel,
            // one positioned over each spike.
            // The brightness of the 3 regions is where we assume the starting prop to be.
            //
            // We also draw rectangles on the screen showing where the sample regions are,
            // as well as drawing a solid rectangle over the sample region
            // we believe is on top of the team prop.
            //
            // In order for this while process to work correctly,
            // each sample region should be positioned in the center of each of the first 3 prop positions,
            // and be small enough such that only the prop is sampled, and not any of the surroundings.

            // Get the Cb channel of the input from after conversion to YCrCb
            inputToCb(input);

            // Compute the average pixel of each submat region.
            // We're taking the average of a single channel buffer, so the value we need is at index 0.
            // We could have also taken the average pixel value of the 3-channel image,
            // and referenced the value at index 2 here.
            avg1 = (int) Core.mean(region1).val[0];
            avg2 = (int) Core.mean(region2).val[0];
            avg3 = (int) Core.mean(region3).val[0];

            // Draw a rectangle showing sample region 1-3 on the screen.
            // Simply visual aid.
            // Serves no functional purpose.
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE,2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE,2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE,2);

            // Find the max of the 3 averages
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            //Now that we found the max, we actually need to go and figure out which sample region that value was from.
            if (max == avg1) {
                bluePosition = PixelPosition.LEFT;
                Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);
            } else if (max == avg2) {
                bluePosition = PixelPosition.CENTER;
                Imgproc.rectangle(input, region2_pointA, region2_pointB, GREEN, -1);
            } else if (max == avg3) {
                bluePosition = PixelPosition.RIGHT;
                Imgproc.rectangle(input, region3_pointA, region3_pointB, GREEN, -1);
            }

            // Render the 'input' buffer to the viewport.
            // But note this is not simply rendering the raw camera feed,
            // because we called functions to add some annotations to this buffer earlier up.
            return input;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public PixelPosition getAnalysis() {
            return bluePosition;
        }
    }
    public static class RedPixelDeterminationPipeline extends OpenCvPipeline {
        public enum PixelPosition {
            LEFT, CENTER, RIGHT
        }
        @Override
        public void init(Mat firstFrame) {

            // We need to call this to make sure the 'Cb' or 'Cr' object is initialized,
            // so that the submats we make will still be linked to it on the subsequent frames.
            // (If the object were to only be initialized in processFrame,
            // then the submats would become delinked because the backing buffer would be re-allocated the first time
            // a real frame was crunched)
            inputToCr(firstFrame);

            // Submats are a persistent reference to a region of the parent buffer.
            // Any changes to the child affect the parent, and the reverse also holds true.
            region1 = Cr.submat(new Rect(region1_pointA, region1_pointB));
            region2 = Cr.submat(new Rect(region2_pointA, region2_pointB));
            region3 = Cr.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            // Overview of what we're doing:
            //
            // We first convert YCrCb color space, from RGB color space.
            // Why do we do this?
            // Well, in the RGB color space,chroma and luma are intertwined.
            // In YCrCb, chroma and luma are separated.
            // YCrCb is a 3-channel color space, just like RGB.
            // TCrCb's 3 channels are Y,
            // the luma channel (which is essentially just a B&W image), the Cr channel,
            // which records the difference from red, and the Cb channel,
            // which records the difference from blue.
            // Because chroma and luma are not related in YCrCb, vision code
            // written to look for certain values in the Cr/Cb channels will not be severely affected by differing light intensity,
            // since that difference would most likely just be reflected in the Y channel.
            //
            // After we've converted to YCrCb, we extract just the 3rd channel, the Cr channel.
            //
            // We then take the average pixel value of 3 different regions on that Cr channel,
            // one positioned over each spike.
            // The brightness of the 3 regions is where we assume the starting prop to be.
            //
            // We also draw rectangles on the screen showing where the sample regions are,
            // as well as drawing a solid rectangle over the sample region
            // we believe is on top of the team prop.
            //
            // In order for this while process to work correctly,
            // each sample region should be positioned in the center of each of the first 3 prop positions,
            // and be small enough such that only the prop is sampled, and not any of the surroundings.

            // Get the Cr channel of the input from after conversion to YCrCb
            inputToCr(input);

            // Compute the average pixel of each submat region.
            // We're taking the average of a single channel buffer, so the value we need is at index 0.
            // We could have also taken the average pixel value of the 3-channel image,
            // and referenced the value at index 2 here.
            avg1 = (int) Core.mean(region1).val[0];
            avg2 = (int) Core.mean(region2).val[0];
            avg3 = (int) Core.mean(region3).val[0];

            // Draw a rectangle showing sample region 1-3 on the screen.
            // Simply visual aid.
            // Serves no functional purpose.
            Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE,2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE,2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE,2);

            // Find the max of the 3 averages
            int maxOneTwo = Math.max(avg1, avg2);
            int max = Math.max(maxOneTwo, avg3);

            //Now that we found the max, we actually need to go and figure out which sample region that value was from.
            if (max == avg1) {
                redPosition = PixelPosition.LEFT;
                Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);
            } else if (max == avg2) {
                redPosition = PixelPosition.CENTER;
                Imgproc.rectangle(input, region2_pointA, region2_pointB, GREEN, -1);
            } else if (max == avg3) {
                redPosition = PixelPosition.RIGHT;
                Imgproc.rectangle(input, region3_pointA, region3_pointB, GREEN, -1);
            }

            // Render the 'input' buffer to the viewport.
            // But note this is not simply rendering the raw camera feed,
            // because we called functions to add some annotations to this buffer earlier up.
            return input;
        }

        // Call this from the OpMode thread to obtain the latest analysis
        public PixelPosition getAnalysis() {
            return redPosition;
        }
    }
}
