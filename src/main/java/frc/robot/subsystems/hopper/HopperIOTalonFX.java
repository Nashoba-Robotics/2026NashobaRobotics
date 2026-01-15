package frc.robot.subsystems.hopper;

import java.lang.constant.Constable;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants;

public class HopperIOTalonFX implements HopperIO {

    private final TalonFX hopper;
    private final TalonFXConfiguration config;

    private final DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public HopperIOTalonFX(){
        hopper = new TalonFX(Constants.Hopper.MOTOR_ID);
        config = new TalonFXConfiguration();
        
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = Constants.Hopper.STATOR_LIMIT;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.Hopper.SUPPLY_LIMIT;

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Feedback.SensorToMechanismRatio = Constants.Hopper.GEAR_RATIO;

        config.MotorOutput.Inverted = Constants.Hopper.INVERTED;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hopper.getConfigurator().apply(config);
        
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.connected = hopper.isConnected();
        inputs.tempCelsius = hopper.getDeviceTemp().getValueAsDouble();
        inputs.velocityRadsPerSec = Units.rotationsToRadians(hopper.getVelocity().getValueAsDouble());
        inputs.appliedVolts = hopper.getMotorVoltage().getValueAsDouble();
        inputs.statorCurrentAmps = hopper.getStatorCurrent().getValueAsDouble();
        inputs.supplyCurrentAmps = hopper.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void runDutyCycle(double percent) {
        hopper.setControl(dutyCycle.withOutput(percent));
    }

    @Override
    public void runVelocity(double velocityRadsPerSec) {
        hopper.set(Units.radiansToRotations(velocityRadsPerSec));
    }

    @Override
    public void stop() {
        hopper.setControl(new NeutralOut());
    }

    @Override
    public void setPID(double kP, double kD) {
        config.Slot0.kP = kP;
        config.Slot0.kD = kD;
        hopper.getConfigurator().apply(config);
    }
}
