package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;



public class Drive {

    public static void pause(double seconds){
        ElapsedTime pauseTimer = new ElapsedTime();
        pauseTimer.reset();
        while(pauseTimer.time() < seconds){}
    }

    public static double[] circleToSquare(double u, double v){
        double u2 = u * u;
        double v2 = v * v;
        double twosqrt2 = 2.0 * Math.sqrt(2.0);
        double subtermx = 2.0 + u2 - v2;
        double subtermy = 2.0 - u2 + v2;
        double termx1 = subtermx + u * twosqrt2;
        double termx2 = subtermx - u * twosqrt2;
        double termy1 = subtermy + v * twosqrt2;
        double termy2 = subtermy - v * twosqrt2;
        double x = (0.5 * Math.sqrt(termx1) - 0.5 * Math.sqrt(termx2));
        double y = (0.5 * Math.sqrt(termy1) - 0.5 * Math.sqrt(termy2));
        return new double[]  {x, y};
    }


    public static double[] calculateFOD(double joyX, double joyY, double gyroAngle) {
        double rotAngle = -Math.toRadians(gyroAngle);
        double relaX = 0.0001 * (Math.round((joyX * Math.cos(rotAngle) - joyY * Math.sin(rotAngle)) * 10000));
        double relaY = 0.0001 * (Math.round((joyY * Math.cos(rotAngle) + joyX * Math.sin(rotAngle)) * 10000));
        return circleToSquare(relaX, relaY);
    }


}
