package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.LEDs;

public class LightsSubsystem extends SubsystemBase {

    private final String subsystem = "Marquee";
    private final StringSubscriber marqueeSubscriber;
    private PWMSparkMax led;

    public LightsSubsystem(StringSubscriber marqueeSubscriber) {
        this.marqueeSubscriber = marqueeSubscriber;
        led = new PWMSparkMax(LEDs.k_PWMId);
    }

    

    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        // If we have no alliance, we cannot be enabled, therefore no hub.
        if (alliance.isEmpty()) {
            return false;
        }
        // Hub is always enabled in autonomous.
        if (DriverStation.isAutonomousEnabled()) {
            return true;
        }
        // At this point, if we're not teleop enabled, there is no hub.
        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        // We're teleop enabled, compute.
        double matchTime = DriverStation.getMatchTime();
        String gameData = DriverStation.getGameSpecificMessage();
        // If we have no game data, we cannot compute, assume hub is active, as its
        // likely early in teleop.
        if (gameData.isEmpty()) {
            return true;
        }
        boolean redInactiveFirst = false;
        switch (gameData.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                // If we have invalid game data, assume hub is active.
                return true;
            }
        }

        // Shift was is active for blue if red won auto, or red if blue won auto.
        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            // Transition shift, hub is active.
            return true;
        } else if (matchTime > 105) {
            // Shift 1
            return shift1Active;
        } else if (matchTime > 80) {
            // Shift 2
            return !shift1Active;
        } else if (matchTime > 55) {
            // Shift 3
            return shift1Active;
        } else if (matchTime > 30) {
            // Shift 4
            return !shift1Active;
        } else {
            // End game, hub always active.
            return true;
        }
    }

    @Override
    public void periodic() {
        String status = marqueeSubscriber.get();
        SmartDashboard.putString(subsystem + "/Status", status);
        String rgb = "Yellow";
        double green = 0;
        double red = 0;
        double blue = 0;
        double value = 0;

        // for (int i = 0; i < buffer.getLength(); i++) {
        // buffer.setLED(i, color);
        // }
        led.set(value);

        SmartDashboard.putNumber(subsystem + "/Color/Red", red);
        SmartDashboard.putNumber(subsystem + "/Color/Green", green);
        SmartDashboard.putNumber(subsystem + "/Color/Blue", blue);
        SmartDashboard.putString(subsystem + "/Color/RBG", rgb);

        SmartDashboard.putBoolean("isHubActive", isHubActive());

        if (isHubActive()) {
            led.set(0.75);
        } else {
            led.set(0.59);
        }
            
    }


}
