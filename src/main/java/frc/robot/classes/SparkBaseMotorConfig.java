package frc.robot.classes;
import com.revrobotics.spark.SparkBase;

public class SparkBaseMotorConfig<T extends SparkBase> {
    public boolean isInverted;
    public SparkBaseMotorChannels channels;
    public double distancePerPulse = .1;
    public double maxSpeed;
    public double maxVoltage;
    public SparkBaseMotor<T> leader;
    public Designation designation;
    public String subSystem;
    public static final double defaultDistancePerPulse = .1;

    public SparkBaseMotorConfig(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted,
            double distancePerPulse, Designation designation, double maxSpeed, double maxVoltage) {
        this.subSystem = subsystem;
        this.channels = channels;
        this.isInverted = isInverted;
        this.designation = designation;
        this.maxSpeed = maxSpeed;
        this.maxVoltage = maxVoltage;

        double distancePerPulseValue = defaultDistancePerPulse;
        if (distancePerPulse > 0) {
            distancePerPulseValue = distancePerPulse;
        }
        this.distancePerPulse = distancePerPulseValue;
    }

    public SparkBaseMotorConfig(String subsystem,
            SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation, double maxSpeed, double maxVoltage) {
        this(subsystem, channels, isInverted, defaultDistancePerPulse, designation, maxSpeed, maxVoltage);
    }  

    public SparkBaseMotorConfig(String subsystem, SparkBaseMotorChannels channels,
            boolean isInverted, Designation designation,double maxSpeed, double maxVoltage, SparkBaseMotor<T> leader) {
        this(subsystem, channels, isInverted, 0, designation, maxSpeed, maxVoltage);
        this.leader = leader;
    }
}
