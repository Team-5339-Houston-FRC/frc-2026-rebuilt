package frc.robot.classes;

import com.revrobotics.sim.SparkMaxSim;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.PWMSim;

public class SparkMAXMotorSim extends SparkMAXMotor {
    private final SparkMaxSim _sim;

    public SparkMAXMotorSim(int channelA, int channelB, boolean isInverted) {
        super(channelA, channelB, isInverted);

        _sim = new SparkMaxSim(getMotor());
    }

    public void addFollower(SparkMAXMotor follower) {
        this.addFollower(follower);
    }

    public static SparkMAXMotorSim CreateSparkMaxMotor(int channelA, int channelB, boolean isInverted) {
        return new SparkMAXMotorSim(channelA, channelB, isInverted);
    }

    @Override
    protected PWMMotorController CreateMotor(int channel, boolean isInverted) {
        PWMSparkMax l = new PWMSparkMax(channel);
        l.setInverted(isInverted);
        return l;
    }

    public void setDistance(double position) {
        _sim.setPosition(position);
    }

    public void setRate(double rate) {
        _sim.setSpeed(rate);
    }

    public void iterate() {
        
    }
}
