package frc.robot.subsystems;

import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MarqueeSubsystem extends SubsystemBase {

    private final String subsystem = "Marquee";
    private final StringSubscriber marqueeSubscriber;
    private AddressableLED led;
    private AddressableLEDBuffer buffer;

    public MarqueeSubsystem(StringSubscriber marqueeSubscriber) {
        this.marqueeSubscriber = marqueeSubscriber;
        led = new AddressableLED(4);
        buffer = new AddressableLEDBuffer(1);
    }

    @Override
    public void periodic() {
        String status = marqueeSubscriber.get();
        Color color = new Color(0, 255, 0);
        SmartDashboard.putString(subsystem + "/Status", status);
        String rgb = "Yellow";
        double green = 0;
        double red = 0;
        double blue = 0;

        switch (status) {
            case "Running":// green
                green = 255;
                rgb = "Green";
                break;
            case "Driving": // blue
                blue = 255;
                rgb = "Blue";
                break;
            case "Shoot": // orange
                green = 165;
                red = 255;
                rgb = "Orange";
                break;
            case "Intake": // red
                red = 255;
                rgb = "Red";
                break;
            case "Outfeed": // purple
                green = 32;
                red = 160;
                blue = 240;
                rgb = "Purple";
                break;
            default: // yellow
                green = 255;
                red = 255;
                rgb = "Yellow";
                break;
        }

        color = new Color(red, green, blue);

        for (int i = 0; i < buffer.getLength(); i++) {
            buffer.setLED(i, color);
        }
        led.setData(buffer);

        SmartDashboard.putNumber(subsystem + "/Color/Red", red);
        SmartDashboard.putNumber(subsystem + "/Color/Green", green);
        SmartDashboard.putNumber(subsystem + "/Color/Blue", blue);
        SmartDashboard.putString(subsystem + "/Color/RBG", rgb);        
    }
}
