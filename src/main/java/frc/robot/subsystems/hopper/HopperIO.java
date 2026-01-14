package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

public interface HopperIO {

    @AutoLog
    public static class HopperIOInputs{
        public boolean connected = false;
        public double velocityRadPerSec = 0.0;
        public double voltageVolts = 0.0;
        public double statorCurrentAmps = 0.0;
        public double supplyCurrentAmps = 0.0;
        public double tempCelsius = 0.0;
        //temporary values
        
    }

    public default void updateInputs(HopperIOInputs inputs){}

    public default void setVelocity(double velocityRadPerSec){}

    public default void setPercentSpeed(double percent){}
}
