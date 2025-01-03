package frc.robot.utils;
 
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;

public class Cancoder extends CANcoder {
    CancoderConfig config;
    String name;
    private CANcoderConfiguration canConfig;

    public Cancoder(CancoderConfig config) {
        super(config.id, config.canbus);
        this.config = config;
		name = config.name;
		configCancoder();
        addLog();
		LogManager.log(name + " cancoder initialized");
    }
    
    private void configCancoder() {
        canConfig = new CANcoderConfiguration();
		canConfig.MagnetSensor.MagnetOffset = config.offset;
        canConfig.MagnetSensor.SensorDirection = config.inverted ? SensorDirectionValue.Clockwise_Positive: SensorDirectionValue.CounterClockwise_Positive;
        getConfigurator().apply(canConfig);
    }
    
    private void addLog() {
        LogManager.addEntry(name + "/Position", this::getPositionRadians);
        LogManager.addEntry(name + "/Absolute position", this::getAbsPositionRadians);
        LogManager.addEntry(name + "/Velocity", this::getVelocityRadiansPerSec);
    }
    
    /**when the cancoder opens its start at the absolute position
     * @return the none absolute amaunt of rotations the motor did in Radians
    */
    public double getPositionRadians() {
        return getPosition().getValueAsDouble() * 2 * Math.PI;
    }
    /**
     * @return the absolute amaunt of rotations the motor did in Radians
     */
    public double getAbsPositionRadians() {
        return getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    }
    /** 
     * @return the amount of rotations the motor do per second in Radians
     */
    public double getVelocityRadiansPerSec(){
        return getVelocity().getValueAsDouble() * 2 * Math.PI;
    }
}
