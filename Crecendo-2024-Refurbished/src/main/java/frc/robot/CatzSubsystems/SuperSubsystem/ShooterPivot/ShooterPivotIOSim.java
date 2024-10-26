package frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot;

import static frc.robot.CatzSubsystems.SuperSubsystem.ShooterPivot.ShooterPivotConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.CatzConstants;

public class ShooterPivotIOSim implements ShooterPivotIO {

    private final DCMotorSim elevationSimMotor;

    private final PIDController shooterPivotFeedback = new PIDController(2,0,0, CatzConstants.LOOP_TIME);

    
    public ShooterPivotIOSim() {
        elevationSimMotor = new DCMotorSim(DCMotor.getNeo550(1), 10, 0.025);
    }


    @Override
    public void updateInputs(ShooterPivotIOInputs inputs) {

        elevationSimMotor.update(CatzConstants.LOOP_TIME);

        inputs.positionTicks = (elevationSimMotor.getAngularPositionRotations());
        inputs.appliedVolts = elevationSimMotor.getCurrentDrawAmps();

    }

    @Override
    public void runPercentOutput(double percentOutput) {
        elevationSimMotor.setInputVoltage(percentOutput*10);
    }

    @Override
    public void runSetpointTicks(double currentPosition, double setPointTicks) {
        elevationSimMotor.setInputVoltage(
            shooterPivotFeedback.calculate(currentPosition, setPointTicks)
        );
    }
}
