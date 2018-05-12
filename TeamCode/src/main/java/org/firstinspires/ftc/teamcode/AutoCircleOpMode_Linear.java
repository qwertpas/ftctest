package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutoCircleOpMode_Linear", group = "Chris")
public class AutoCircleOpMode_Linear extends LinearOpMode {

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

    private void setPowerTime(double aPower, double bPower, double cPower, double dPower, double time) {
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

    private void setPower(double aPower, double bPower, double cPower, double dPower) {
        aDrive.setPower(aPower);
        bDrive.setPower(bPower);
        cDrive.setPower(cPower);
        dDrive.setPower(dPower);
    }


    @Override
    public void runOpMode() throws InterruptedException {


        moto = new Motors(hardwareMap.get(DcMotor.class, "aDrive"), hardwareMap.get(DcMotor.class, "bDrive"), hardwareMap.get(DcMotor.class, "cDrive"), hardwareMap.get(DcMotor.class, "dDrive"));

        moto.clearmotors();

        waitForStart(); //wait until button is pressed
        timer.reset();


        for (double angle = 0; angle < 360; angle = angle + 1) {
            moto.moveLocalAngle(angle, 0.3);
            timer.reset();
            while (timer.time() < (6 / 360)) {
            }
        }


        moto.clearmotors();
    }

}
