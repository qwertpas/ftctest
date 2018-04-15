package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoOpMode", group="Chris")
public class Auto1OpMode_Linear extends LinearOpMode{

    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;


    private ElapsedTime timer = new ElapsedTime();

    private void pause(double seconds){
        while(timer.time() < seconds){}
        timer.reset();
    }

    private void setPowerTime(double aPower, double bPower, double cPower, double dPower, double time){
        aDrive.setPower(aPower);
        bDrive.setPower(bPower);
        cDrive.setPower(cPower);
        dDrive.setPower(dPower);
        pause(time);
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
    }

    private void setPower(double aPower, double bPower, double cPower, double dPower){
        aDrive.setPower(aPower);
        bDrive.setPower(bPower);
        cDrive.setPower(cPower);
        dDrive.setPower(dPower);
    }


    @Override
    public void runOpMode() throws InterruptedException {


        aDrive  = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive  = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive  = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive  = hardwareMap.get(DcMotor.class, "dDrive");


        setPower(0,0,0,0);    //stop motors

        waitForStart(); //wait until button is pressed
        timer.reset();


        aDrive.setPower(0.7);
        bDrive.setPower(-0.7);
        cDrive.setPower(-0.7);
        dDrive.setPower(0.7);
        timer.reset();

        while(timer.time() < 1){}
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);
        timer.reset();

        while(timer.time() < 1){}
        aDrive.setPower(-0.7);
        bDrive.setPower(0.7);
        cDrive.setPower(0.7);
        dDrive.setPower(-0.7);
        timer.reset();

        while(timer.time() < 1){}
        aDrive.setPower(0);
        bDrive.setPower(0);
        cDrive.setPower(0);
        dDrive.setPower(0);

//
//
//        pause(3);   //pause for 3 seconds
//
//
//        setPowerTime(-0.7,-0.7,-0.7,-0.7,3);    //spin clockwise for 3 seconds


    }

}
