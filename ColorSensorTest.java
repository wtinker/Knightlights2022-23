package org.firstinspires.ftc.teamcode.drive.code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Color Sensor Test")
public class ColorSensorTest extends LinearOpMode {

    ColorSensor color;
    DistanceSensor dist;

    int red, redmin, redmax;
    int green, greenmin, greenmax;
    int blue, bluemin, bluemax;
    int alpha, alphamin, alphamax;
    int argb, argbmin, argbmax;
    int distance, distmin, distmax;

    int updateruns = 100;

    @Override
    public void runOpMode(){

        color = hardwareMap.colorSensor.get("color");
        dist = hardwareMap.get(DistanceSensor.class, "dist");

        waitForStart();

        while(opModeIsActive()){
            if(gamepad1.a){
                SensorLoop();
            }

            telemetry.addData("AvgRed:", red);
            telemetry.addData("AvgGreen:", green);
            telemetry.addData("AvgBlue:", blue);
            telemetry.addData("AvgAplha:", alpha);
            telemetry.addData("AvgArgb:", argb);
            telemetry.addData("AvgDistance(mm):", distance);
            telemetry.addData("MinRed:", redmin);
            telemetry.addData("MinGreen:", greenmin);
            telemetry.addData("MinBlue:", bluemin);
            telemetry.addData("MinAplha:", alphamin);
            telemetry.addData("MinArgb:", argbmin);
            telemetry.addData("MinDistance(mm):", distmin);
            telemetry.addData("MaxRed:", redmax);
            telemetry.addData("MaxGreen:", greenmax);
            telemetry.addData("MaxBlue:", bluemax);
            telemetry.addData("MaxAplha:", alphamax);
            telemetry.addData("MaxArgb:", argbmax);
            telemetry.addData("MaxDistance(mm):", distmax);
            telemetry.addData("Red:", color.red());
            telemetry.addData("Green:", color.green());
            telemetry.addData("Blue:", color.blue());
            telemetry.addData("ALpha:", color.alpha());
            telemetry.addData("Argb:", color.argb());
            telemetry.addData("Distance(mm):", dist.getDistance(DistanceUnit.MM));
            telemetry.update();
        }

    }

    public void SensorLoop() {

        int[] redlist;
        int[] greenlist;
        int[] bluelist;
        int[] alphalist;
        int[] argblist;
        int[] distlist;

        redlist = new int[updateruns];
        greenlist = new int[updateruns];
        bluelist = new int[updateruns];
        alphalist = new int[updateruns];
        argblist = new int[updateruns];
        distlist = new int[updateruns];

        for(int i = 0; i < updateruns; i++) {

            redlist[i] = color.red();
            greenlist[i] = color.green();
            bluelist[i] = color.blue();
            alphalist[i] = color.alpha();
            argblist[i] = color.argb();
            distlist[i] = (int) dist.getDistance(DistanceUnit.MM);

        }

        redmin = arraymin(redlist);
        redmax = arraymax(redlist);
        red = (redmin + redmax) / 2;
        greenmin = arraymin(greenlist);
        greenmax = arraymax(greenlist);
        green = (greenmin + greenmax) / 2;
        bluemin = arraymin(bluelist);
        bluemax = arraymax(bluelist);
        blue = (bluemin + bluemax) / 2;
        alphamin = arraymin(alphalist);
        alphamax = arraymax(alphalist);
        alpha = (alphamin + alphamax) / 2;
        argbmin = arraymin(argblist);
        argbmax = arraymax(argblist);
        argb = (argbmin + argbmax) / 2;
        distmin = arraymin(distlist);
        distmax = arraymax(distlist);
        distance = (distmin + distmax) / 2;

    }



    public int arraymax(int[] arr){
        int max = arr[0];
        for(int i = 0; i < arr.length; i++){
            if(arr[i] > max){max = arr[i];}
        }
        return max;
    }

    public int arraymin(int[] arr){
        int min = arr[0];
        for(int i = 0; i < arr.length; i++){
            if(arr[i] < min){min = arr[i];}
        }
        return min;
    }

}
