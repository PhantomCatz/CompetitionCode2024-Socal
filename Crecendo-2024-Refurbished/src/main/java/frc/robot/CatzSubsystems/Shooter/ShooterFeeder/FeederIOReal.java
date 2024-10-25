package frc.robot.CatzSubsystems.Shooter.ShooterFeeder;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

public class FeederIOReal implements FeederIO {
    //  Hardware IO declaration
    private final CANSparkMax feederNeo;

    // Sensor declaration
    private final DigitalInput beamBreakAdjust;
    private final DigitalInput beamBreakLoad;

    public FeederIOReal() {
        // Feeder Neo hardware instantiation
        feederNeo = new CANSparkMax(23, MotorType.kBrushless);
        feederNeo.restoreFactoryDefaults();
        feederNeo.setSmartCurrentLimit(30);
        feederNeo.setIdleMode(IdleMode.kBrake);
        feederNeo.enableVoltageCompensation(12.0); 
        feederNeo.getEncoder().setPositionConversionFactor(ShooterFeederConstants.FEEDER_GEAR_RATIO); //TBD verify math
        feederNeo.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 32767);
        feederNeo.burnFlash(); //save configs so if pwr lost to be reapplied

        // Sensor hardware instantiation
        beamBreakAdjust = new DigitalInput(0);
        beamBreakLoad = new DigitalInput(1);

    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.isAdjustBeamBreakBroken = !beamBreakAdjust.get();
        inputs.isLoadBeamBreakBroken   = !beamBreakLoad.get();
        inputs.noteMovementUpInches    = (ShooterFeederConstants.FEEDER_COMPLIANT_WHEEL_DIAMETER_INCH * Math.PI) * feederNeo.getEncoder().getPosition(); //returns in revolutions of the green wheel
    }

    public void loadFoward() {
        feederNeo.set(-ShooterFeederConstants.LOAD_MOTOR_LOADING_SPEED);
    }

    public void loadBackward() {
        feederNeo.set(ShooterFeederConstants.LOAD_MOTOR_LOADING_SPEED);
    }

    public void fineAdjustFwd() {
        feederNeo.set(-ShooterFeederConstants.LOAD_MOTOR_FWD_ADJUST_SPEED);
    }

    public void fineAdjustBck() {
        feederNeo.set(ShooterFeederConstants.LOAD_MOTOR_BWD_ADJUST_SPEED);
    }

    public void feedShooter() {
        feederNeo.set(-ShooterFeederConstants.LOAD_MOTOR_SHOOTING_SPEED);
    }

    public void feedDisabled() {
        feederNeo.set(0.0);
    }

    public void resetLoadEnc() {
        feederNeo.getEncoder().setPosition(0.0);
    }


    

}
