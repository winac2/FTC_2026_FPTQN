package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;


public class
ColorSensor {
    NormalizedColorSensor colorSensor;

    public float normRed, normGreen, normBlue;

    public void init(HardwareMap HwMap) {
        colorSensor = HwMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(8);
    }

    public String getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); //return 4 values

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        //TODO add if statements to classify
        /*
        red, green, blue,
        P =
        G =
         */
        return "UNKNOWN";
    }
}
