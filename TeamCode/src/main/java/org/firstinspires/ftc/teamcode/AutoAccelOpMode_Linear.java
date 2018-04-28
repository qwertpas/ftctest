package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutooOpMode", group = "Chris")
public class AutoAccelOpMode_Linear extends LinearOpMode {

    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;
    private Motors moto = null;


    private ElapsedTime timer = new ElapsedTime();

    private void pause(double seconds) {
        while (timer.time() < seconds) {
        }
        timer.reset();
    }




    @Override
    public void runOpMode() throws InterruptedException {


        moto = new Motors(hardwareMap.get(DcMotor.class, "aDrive"), hardwareMap.get(DcMotor.class, "bDrive"), hardwareMap.get(DcMotor.class, "cDrive"), hardwareMap.get(DcMotor.class, "dDrive"));

        moto.clearmotors();

        waitForStart(); //wait until button is pressed
        timer.reset();

        for (double power = 0 ; power < 0.6 ; power = power + 0.05){
            moto.forward(power);
            timer.reset();
            while (timer.time() < 0.05) {}
        }

        timer.reset();
        while (timer.time() < 2) {}
        moto.stop();

        moto.clearmotors();

//
//
//        pause(3);   //pause for 3 seconds
//
//
//        setPowerTime(-0.7,-0.7,-0.7,-0.7,3);    //spin clockwise for 3 seconds


    }

}
