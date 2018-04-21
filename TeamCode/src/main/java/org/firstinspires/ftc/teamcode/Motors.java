package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by leshaseletskiy on 4/21/18.
 */

public class Motors {
    private DcMotor aDrive;
    private DcMotor bDrive;
    private DcMotor cDrive;
    private DcMotor dDrive;

    public Motors(DcMotor aDrive, DcMotor bDrive, DcMotor cDrive, DcMotor dDrive) {
        this.aDrive = aDrive;
        this.bDrive = bDrive;
        this.cDrive = cDrive;
        this.dDrive = dDrive;
    }

    public void setMotors(double forward, double right) {
        aDrive.setPower(forward);
        bDrive.setPower(-forward);
        cDrive.setPower(-forward);
        dDrive.setPower(forward);
    }
}
