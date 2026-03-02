package frc.robot.subsystems;

import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDs;

public class MarqueeSubsystem extends SubsystemBase {

    private final String subsystem = "Marquee";
    private final StringSubscriber marqueeSubscriber;
    private PWMSparkMax led;
    private AddressableLEDBuffer buffer;

    public MarqueeSubsystem(StringSubscriber marqueeSubscriber) {
        this.marqueeSubscriber = marqueeSubscriber;
        led = new PWMSparkMax(LEDs.k_PWMId);
        buffer = new AddressableLEDBuffer(LEDs.k_totalLength);
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
double value = 0;
        switch (status) {
            case "Running":// green
                green = 255;
                rgb = "Green";
                value = 0.75;
                break;
            case "Driving": // blue
                blue = 255;
                rgb = "Blue";
                value = 0.87;
                break;
            case "Shoot": // orange
                green = 165;
                red = 255;
                rgb = "Orange";
                value = 0.65;
                break;
            case "Intake": // red
                red = 255;
                rgb = "Red";
                value = 0.61;
                break;
            case "Outfeed": // purple
                green = 32;
                red = 160;
                blue = 240;
                rgb = "Purple";
                value = 0.91;
                break;
            default: // yellow
                green = 255;
                red = 255;
                rgb = "Yellow";
                value = 0.69;
                break;
        }

        color = new Color(red, green, blue);

        // for (int i = 0; i < buffer.getLength(); i++) {
        //     buffer.setLED(i, color);
        // }
        led.set(value);

        SmartDashboard.putNumber(subsystem + "/Color/Red", red);
        SmartDashboard.putNumber(subsystem + "/Color/Green", green);
        SmartDashboard.putNumber(subsystem + "/Color/Blue", blue);
        SmartDashboard.putString(subsystem + "/Color/RBG", rgb);        
    }
}
