package frc.robot.classes;

import com.revrobotics.spark.SparkBase;

public class SparkBaseMotorConfig<T extends SparkBase> {
    public boolean isInverted;
    public SparkBaseMotorChannels channels;
    public double distancePerPulse = .1;
    public SparkBaseMotor<T> leader;
    public Designation designation;
    public String subSystem;
    public static final double defaultDistancePerPulse = .1;

    public SparkBaseMotorConfig(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            double distancePerPulse, Designation designation) {
        this.subSystem = subsystem;
        this.channels = channels;
        this.isInverted = isInverted;
        this.designation = designation;

        double distancePerPulseValue = defaultDistancePerPulse;
        if (distancePerPulse > 0) {
            distancePerPulseValue = distancePerPulse;
        }
        this.distancePerPulse = distancePerPulseValue;
    }

    public SparkBaseMotorConfig(String subsystem,
            SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation) {
        this(subsystem, channels, isInverted, defaultDistancePerPulse, designation);
    }

    // public SparkBaseMotorConfig(SparkBaseMotorChannels channels,
    // boolean isInverted,
    // double distancePerPulse, Designation designation, SparkBaseMotor<T> leader) {
    // this(channels, isInverted, distancePerPulse, designation);
    // this.leader = leader;
    // }

    public SparkBaseMotorConfig(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation, SparkBaseMotor<T> leader) {
        this(subsystem, channels, isInverted, 0, designation);
        this.leader = leader;
    }
}
