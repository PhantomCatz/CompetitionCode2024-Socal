package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.signals.NeutralModeValue;

public interface ShooterPivotIO {
    
    @AutoLog
    class ShooterPivotIOInputs {
        public boolean isElevationMotorConnected = true;

        public double positionTicks = 0.0;
        public double appliedVolts = 0.0;
        public double tempCelcius = 0.0;
    }

    public default void updateInputs(ShooterPivotIOInputs inputs) {}
    
    public default void runPercentOutput(double percentOutput) {}

    public default void stop() {}

    public default void setPID(double p, double i, double d) {}

    public default void runSetpointTicks(double currentAngleDegrees,double setpointAngleDegrees) {}
}
