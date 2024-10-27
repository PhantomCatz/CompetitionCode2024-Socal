package frc.robot.CatzSubsystems.IntakeRollers;

import static frc.robot.CatzSubsystems.IntakeRollers.IntakeRollersConstants.*;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeRollersIOReal implements IntakeRollersIO {
    // Hardware
    private final TalonFX rollerTalon;
    private final DigitalInput frontBeamBreak;

    // Status Signals
    private final StatusSignal<Double> rollerVelocity;
    private final StatusSignal<Double> rollerAppliedVolts;
    private final StatusSignal<Double> rollerSupplyCurrent;
    private final StatusSignal<Double> rollerTorqueCurrent;
    private final StatusSignal<Double> rollerTempCelsius;

    // Control
    private final Slot0Configs controllerConfig = new Slot0Configs();
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0.0);
    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0.0);
    private final NeutralOut neutralControl = new NeutralOut().withUpdateFreqHz(0.0);
    private final DutyCycleOut dudtyCycleControl = new DutyCycleOut(0.0).withUpdateFreqHz(60.0);

    public IntakeRollersIOReal() {
        frontBeamBreak = new DigitalInput(FRONT_BEAM_BREAK);
        rollerTalon = new TalonFX(INTAKE_ROLLER_ID);

        // General config
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // Controller config;
        controllerConfig.kP = gains.kP();
        controllerConfig.kI = gains.kI();
        controllerConfig.kD = gains.kD();
        controllerConfig.kS = gains.kS();
        controllerConfig.kV = gains.kV();
        controllerConfig.kA = gains.kA();

        // Set Signals
        rollerVelocity = rollerTalon.getVelocity();
        rollerAppliedVolts = rollerTalon.getMotorVoltage();
        rollerSupplyCurrent = rollerTalon.getSupplyCurrent();
        rollerTorqueCurrent = rollerTalon.getTorqueCurrent();
        rollerTempCelsius = rollerTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            rollerVelocity,
            rollerAppliedVolts,
            rollerSupplyCurrent,
            rollerTorqueCurrent,
            rollerTempCelsius
        );
    }

    @Override
    public void updateInputs(IntakeRollersIOInputs inputs) {
        inputs.isRollerMotorConnected =
            BaseStatusSignal.refreshAll(
                rollerVelocity,
                rollerAppliedVolts,
                rollerSupplyCurrent,
                rollerTorqueCurrent,
                rollerTempCelsius)
            .isOK();
        inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
        inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
        inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerTempCelsius.getValueAsDouble();

        inputs.isFrontBeambreakBroken = !frontBeamBreak.get();
    }

    /** Run motors at volts */
    @Override
    public void runVolts(double volts) {
        rollerTalon.setControl(voltageControl.withOutput(volts));
    }

    /** Run motors at duty cycle percentage */
    @Override
    public void runDutycycle(double percentOutput) {
        rollerTalon.setControl(dudtyCycleControl.withOutput(percentOutput));
    }

    /** Stops motors */
    @Override
    public void stop() {
        rollerTalon.setControl(neutralControl);
    }

    /** BEAM BREAK */

    
}
