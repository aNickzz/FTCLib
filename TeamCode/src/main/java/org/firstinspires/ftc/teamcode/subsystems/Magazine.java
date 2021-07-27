package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Constants.MagazineConstants;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Magazine extends SubsystemBase {

    private HardwareMap hardwareMap;
    private Telemetry telemetry;

    public Magazine(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

}
