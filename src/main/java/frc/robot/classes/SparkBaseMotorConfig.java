package frc.robot.classes;

import com.revrobotics.spark.SparkBase;

public class SparkBaseMotorConfig<T extends SparkBase> {
    public boolean isInverted;
    public SparkBaseMotorChannels channels;
    public double distancePerPulse = .1;
    public SparkBaseMotor<T> leader;
    public static final double defaultDistancePerPulse = .1;

    public SparkBaseMotorConfig(SparkBaseMotorChannels channels,
            boolean isInverted,
            double distancePerPulse) {
        this.channels = channels;
        this.isInverted = isInverted;

        double distancePerPulseValue = defaultDistancePerPulse;
        if (distancePerPulse > 0) {
            distancePerPulseValue = distancePerPulse;
        }
        this.distancePerPulse = distancePerPulseValue;
    }

    public SparkBaseMotorConfig(SparkBaseMotorChannels channels,
            boolean isInverted) {
                this(channels, isInverted, defaultDistancePerPulse);
    }

    public SparkBaseMotorConfig(SparkBaseMotorChannels channels,
            boolean isInverted,
            double distancePerPulse, SparkBaseMotor<T> leader) {
        this(channels, isInverted, distancePerPulse);
        this.leader = leader;
    }
    public SparkBaseMotorConfig(SparkBaseMotorChannels channels,
            boolean isInverted,SparkBaseMotor<T> leader) {
        this(channels, isInverted, 0);
        this.leader = leader;
    }
}
