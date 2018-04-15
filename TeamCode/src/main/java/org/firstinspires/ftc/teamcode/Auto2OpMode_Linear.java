package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoOpMode", group="Chris")
public class Auto2OpMode_Linear extends LinearOpMode{

    private DcMotor aDrive = null;
    private DcMotor bDrive = null;
    private DcMotor cDrive = null;
    private DcMotor dDrive = null;
    private double aPowerSent = 0;
    private double bPowerSent = 0;
    private double cPowerSent = 0;
    private double dPowerSent = 0;

    private ElapsedTime timer = new ElapsedTime();

    private void pause(double seconds){
        timer.reset();
        while(timer.time() < seconds){}
    }

    private void setPowerTime(double aPower, double bPower, double cPower, double dPower, double time){
        printStuff();

        aPowerSent = aPower;
        bPowerSent = bPower;
        cPowerSent = cPower;
        dPowerSent = dPower;

        aDrive.setPower(aPowerSent);
        bDrive.setPower(bPowerSent);
        cDrive.setPower(cPowerSent);
        dDrive.setPower(dPowerSent);

        printStuff();

        timer.reset();
        while(timer.time() < time){}

        aPowerSent = 0;
        bPowerSent = 0;
        cPowerSent = 0;
        dPowerSent = 0;

        aDrive.setPower(aPowerSent);
        bDrive.setPower(bPowerSent);
        cDrive.setPower(cPowerSent);
        dDrive.setPower(dPowerSent);

        printStuff();
    }

    private void setPower(double aPower, double bPower, double cPower, double dPower){
        printStuff();

        aPowerSent = aPower;
        bPowerSent = bPower;
        cPowerSent = cPower;
        dPowerSent = dPower;

        aDrive.setPower(aPowerSent);
        bDrive.setPower(bPowerSent);
        cDrive.setPower(cPowerSent);
        dDrive.setPower(dPowerSent);

        printStuff();
    }

    private void printStuff(){
        telemetry.addData("Run Time", timer.time());
        telemetry.addData("Motors",
                          "aPowerSent (%.2f), bPowerSent (%.2f), cPowerSent (%.2f), dPowerSent (%.2f)",
                                        aPowerSent,     bPowerSent,         cPowerSent,         dPowerSent);
        telemetry.update();
    }



    @Override
    public void runOpMode() throws InterruptedException {

        aDrive  = hardwareMap.get(DcMotor.class, "aDrive");
        bDrive  = hardwareMap.get(DcMotor.class, "bDrive");
        cDrive  = hardwareMap.get(DcMotor.class, "cDrive");
        dDrive  = hardwareMap.get(DcMotor.class, "dDrive");

        waitForStart(); //wait until button is pressed

        ///////////////////////////////////////////////////////////////     AUTO CODE:

        setPowerTime(0.7,-0.7,-0.7,-0.7, 1);

        pause(1);

        setPowerTime(-0.7,0.7,0.7,-0.7,1);





        ///////////////////////////////////////////////////////////////
    }
}
