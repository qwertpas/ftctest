package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by chris on 4/1/18.
 */
public class HoloOpMode_Linear extends LinearOpMode{

    DcMotor aDrive;
    DcMotor bDrive;
    DcMotor cDrive;
    DcMotor dDrive;
    ElapsedTime timer = new ElapsedTime();

    public void pause(double seconds){
        while(timer.time() < seconds){}
        timer.reset();
    }

    public void setPowerTime(double aPower, double bPower, double cPower, double dPower, double time){
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

    public void setPower(double aPower, double bPower, double cPower, double dPower){
        aDrive.setPower(aPower);
        bDrive.setPower(bPower);
        cDrive.setPower(cPower);
        dDrive.setPower(dPower);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        aDrive = hardwareMap.dcMotor.get("aDrive");
        bDrive = hardwareMap.dcMotor.get("bDrive");
        cDrive = hardwareMap.dcMotor.get("cDrive");
        dDrive = hardwareMap.dcMotor.get("dDrive");

        setPower(0,0,0,0);    //stop motors

        waitForStart(); //wait until button is pressed
        timer.reset();

        setPowerTime(0.7,0.7,0.7,0.7,3);    //spin counter-clockwise for 3 seconds

        pause(3);   //pause for 3 seconds

        setPowerTime(-0.7,-0.7,-0.7,-0.7,3);    //spin clockwise for 3 seconds

    }

}
