package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Servothing", group = "Chris")

public class AutoServoOpMode_Linear extends LinearOpMode {

    private Servo servo;


    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            servo.setPosition(0);

            Auto2OpMode_Linear.pause(1);


            servo.setPosition(1.1);

            Auto2OpMode_Linear.pause(1);


            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }



}
