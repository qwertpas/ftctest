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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "AutoFODOpMode_Linear", group = "chris")
public class TeleFODOpMode_Linear extends LinearOpMode {


    @Override
    public void runOpMode() {
        //when program is selected



//INITIALIZE
    //MOTORS
        telemetry.addData("Status", "Initializing Motors"); telemetry.update();

        Motors2 moto = new Motors2(hardwareMap.get(DcMotor.class, "aDrive"),
                                   hardwareMap.get(DcMotor.class, "bDrive"),
                                   hardwareMap.get(DcMotor.class, "cDrive"),
                                   hardwareMap.get(DcMotor.class, "dDrive"));
        moto.clear(); //makes sure motors don't run before hitting start!


    //IMU
        telemetry.addData("Status", "Initializing IMU"); telemetry.update();

        IMU imu = new IMU(hardwareMap.get(BNO055IMU.class,"imu"));
        imu.initialize();
        telemetry.addData("Status", "Initializing DONE"); telemetry.update();




        waitForStart();
//START PROGRAM

        imu.resetHeading();
        double heading;

//MAIN LOOP
        while (opModeIsActive()) {

            heading = imu.getHeading();

            gamepad1.left_stick_x

            //Uses FOD
            moto.moveGlobalAngle(0, heading, 0.2, 0);



            telemetry.addData("normieHeading", Calculate.normalizeAngle(heading));
            telemetry.addData("heading", heading);

            telemetry.update();
        }
    }












}
