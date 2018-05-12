package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;


@Autonomous(name = "AutoCircleSpinOpMode_Linear", group = "Chris")
public class AutoCircleSpinOpMode_Linear extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Motors2 motors = new Motors2(hardwareMap.get(DcMotor.class, "aDrive"), hardwareMap.get(DcMotor.class, "bDrive"), hardwareMap.get(DcMotor.class, "cDrive"), hardwareMap.get(DcMotor.class, "dDrive"));
        motors.clearmotors();

        waitForStart(); //wait until button is pressed
        ElapsedTime runtime = new ElapsedTime();
        ElapsedTime timer = new ElapsedTime();

        //Revolute and Rotate for 6 seconds (0.3 revolute, 0.08 spin)
        for (double angle = 0; angle < 360; angle = angle + 1) {
            motors.moveLocalAngle(angle, 0.3, 0.08);
            timer.reset();
            while (timer.time() < (6 / 360)) {}

            telemetry.addData("Motor Powers", Arrays.toString(motors.getPowers()));
            telemetry.addData("Angle", angle);
            telemetry.addLine();
            telemetry.addData("Runtime", runtime);
        }//end of circle

        motors.clearmotors();

    }//end of OpMode
}//end of Class
