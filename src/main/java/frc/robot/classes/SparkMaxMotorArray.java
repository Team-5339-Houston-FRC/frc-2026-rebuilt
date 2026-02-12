package frc.robot.classes;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.abstractions.ISparkMaxMotorArray;
import frc.robot.classes.SparkMaxMotorArraySim.Designation;

public class SparkMaxMotorArray extends SparkMAXMotor implements ISparkMaxMotorArray {

    private final List<SparkMAXMotor> motors = new ArrayList<SparkMAXMotor>();
    public double distance;

    public SparkMaxMotorArray() {

    }
    
    public SparkMaxMotorArray(List<SparkBaseMotorChannels> channels, boolean isInverted) {
        super(channels.get(0), isInverted);

        int channelsSize = channels.size();
        if (channelsSize > 1) {
            for (SparkBaseMotorChannels channel : channels.subList(1, channels.size())) {
                SparkBaseMotorConfig<SparkMax> config = new SparkBaseMotorConfig<SparkMax>(channel, isInverted,
                        0, this);
                SparkMAXMotor follower = new SparkMAXMotor(config);
                motors.add(follower);
            }
        }
    }

    public void setDistance(double position) {
        distance = position;
    }

    public void simulationPeriodic(DifferentialDrivetrainSim drivetrainSim, Designation designation) {

    }
}
