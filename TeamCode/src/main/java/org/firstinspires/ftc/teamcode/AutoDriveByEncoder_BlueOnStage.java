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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels, otherwise you would use: RobotAutoDriveByTime;
 *
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forward, and causes the encoders to count UP.
 *
 * The code is written using a method called: encoderDrive(speed, leftFrontInches, leftBackInches,
 * rightFrontInches, rightBackInches timeoutS) that performs the actual movement.
 * This method assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder-based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 */

@Autonomous(name="Auto Drive By Encoder (Blue On Stage)", group="Robot")

public class AutoDriveByEncoder_BlueOnStage extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "robot." To access this class.
    RobotHardware robot = new RobotHardware(this);
    RobotHardware.BluePixelDeterminationPipeline pipeline;
    RobotHardware.BluePixelDeterminationPipeline.PixelPosition snapshotAnalysis = RobotHardware.BluePixelDeterminationPipeline.PixelPosition.LEFT; //default

    @Override
    public void runOpMode() {

        // Initialize all the hardware, using the hardware class. See how clean and simple this is?
        robot.init();

        pipeline = new RobotHardware.BluePixelDeterminationPipeline();
        robot.webcam.setPipeline(pipeline);

        // The INIT-loop: This REPLACES waitForStart!
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();
            sleep(50);
        }

        snapshotAnalysis = pipeline.getAnalysis();

        // Show that snapshot on the telemetry
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();

        switch (snapshotAnalysis) {
            case LEFT: {
                // Your autonomous code
                break;
            } case RIGHT: {
                // Your autonomous code
                break;
            } case CENTER: {
                // Step through each leg of the path,
                // Note: Reverse movement is obtained by setting a negative distance (not speed)

                // Strafe right 2.375 inches with a 5 Sec timeout
                robot.encoderDrive(RobotHardware.AUTO_DRIVE_SPEED, 2.375,-2.375,-2.375,2.375, 5.0);
                // Drive forward 27.5 Inches with 5 Sec timeout
                robot.encoderDrive(RobotHardware.AUTO_DRIVE_SPEED,  27.5,  27.5, 27.5, 27.5, 5.0);
                // Drive back 25.125 inches with a 5 Sec timeout
                robot.encoderDrive(RobotHardware.AUTO_DRIVE_SPEED, -25.125, -25.125, -25.125, -25.125,5.0);
                // Turn right 14.13717 Inches with 5 Sec timeout
                robot.encoderDrive(RobotHardware.AUTO_TURN_SPEED, 14.13717, 14.13717, -14.13717, -14.13717, 5.0);
                // Drive forward 45.5 Inches with 5 Sec timeout
                robot.encoderDrive(RobotHardware.AUTO_DRIVE_SPEED, 45.5, 45.5, 45.5, 45.5, 5.0);

                break;
            }

        }
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

}
