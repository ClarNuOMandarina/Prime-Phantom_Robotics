package org.firstinspires.ftc.teamcode.user;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class extension {
    public Servo left_extension;
    public Servo right_extension;
    public double extension_extended=1;
    public double extension_retracted=0.69;

    public double extension_transfer=0.83;


    public extension(HardwareMap hardwareMap)
    {

        left_extension =hardwareMap.get(Servo.class,"left_extension");
        right_extension =hardwareMap.get(Servo.class,"right_extension");
    }

    public void extend(double poz){
        right_extension.setPosition(poz);
        left_extension.setPosition(poz);

    }


}
