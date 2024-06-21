package frc.robot.subsystems.DriveAndRobotOrientation.drivetrain;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.subsystems.DriveAndRobotOrientation.drivetrain.DriveConstants.ModuleConfig;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ModuleIOReal implements ModuleIO {
    //Motor instantiation
    private final CANSparkMax STEER_MOTOR;
    private final TalonFX DRIVE_MOTOR;

    //Motor Current limiting
    public static final int     KRAKEN_CURRENT_LIMIT_AMPS            = 50;
    public static final int     KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS    = 60;
    public static final double  KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS = 0.05;
    public static final boolean KRAKEN_ENABLE_CURRENT_LIMIT          = true;

    public static final int     NEO_CURRENT_LIMIT_AMPS      = 30;

    private boolean breakEnabled = false;

    //Mag enc instatiation
    private DutyCycleEncoder magEnc;
    private DigitalInput MagEncPWMInput;

    //status code initialization
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

        //create new config objects
    private TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    private Slot0Configs driveConfigs         = new Slot0Configs();

        //StatusSignal
    StatusSignal<Double> driveRotorPosition;

    public ModuleIOReal(ModuleConfig configIO) {

        //mag encoder setup
        MagEncPWMInput = new DigitalInput(configIO.absoluteEncoderChannel());
        magEnc = new DutyCycleEncoder(MagEncPWMInput);

        //steer motor setup
        STEER_MOTOR = new CANSparkMax(configIO.turnID(), MotorType.kBrushless);
        STEER_MOTOR.restoreFactoryDefaults();
        STEER_MOTOR.setSmartCurrentLimit(NEO_CURRENT_LIMIT_AMPS);
        STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        STEER_MOTOR.enableVoltageCompensation(12.0);

        //Drive Motor setup
        DRIVE_MOTOR = new TalonFX(configIO.driveID());
            //reset to factory defaults
        DRIVE_MOTOR.getConfigurator().apply(new TalonFXConfiguration());
        talonConfigs.Slot0 = driveConfigs;

        talonConfigs.Voltage.PeakForwardVoltage = 11.5;
        talonConfigs.Voltage.PeakReverseVoltage = -11.5;
            //current limit
        talonConfigs.CurrentLimits = new CurrentLimitsConfigs();
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = KRAKEN_ENABLE_CURRENT_LIMIT;
        talonConfigs.CurrentLimits.StatorCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;

        talonConfigs.CurrentLimits.SupplyCurrentThreshold   = KRAKEN_CURRENT_LIMIT_TRIGGER_AMPS;
        talonConfigs.CurrentLimits.SupplyCurrentLimit       = KRAKEN_CURRENT_LIMIT_AMPS;
        talonConfigs.CurrentLimits.SupplyTimeThreshold      = KRAKEN_CURRENT_LIMIT_TIMEOUT_SECONDS;
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();

            //neutral mode
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            //pid
        driveConfigs.kP = 0.01; //1.0 if using velocity Torque 0.01 for velocity dutycycle
        driveConfigs.kI = 0.0;
        driveConfigs.kD = 0.0;
 

        //check if drive motor is initialized correctly
        for(int i=0;i<5;i++){
            initializationStatus = DRIVE_MOTOR.getConfigurator().apply(talonConfigs);
            if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + configIO.driveID());
        }

    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        if(breakEnabled) {
            STEER_MOTOR.setIdleMode(IdleMode.kBrake);
        } else {
            STEER_MOTOR.setIdleMode(IdleMode.kCoast);
        }


        inputs.driveMtrVelocity       = DRIVE_MOTOR.getRotorVelocity().getValue();
        inputs.driveMtrSensorPosition = DRIVE_MOTOR.getRotorPosition().getValue();
        //inputs.driveAppliedVolts      = DRIVE_MOTOR.getMotorVoltage().getValueAsDouble();
        inputs.magEncoderValue        = magEnc.get();

    }

    @Override
    public void setDriveVelocityIO(double velocity, double feedForward) {
        DRIVE_MOTOR.setControl(new VelocityDutyCycle(velocity));
    }

    @Override
    public void setDrivePwrPercentIO(double drivePwrPercent) {
        DRIVE_MOTOR.setControl(new DutyCycleOut(drivePwrPercent,
                        true,
                        false,
                        false,
                        false));

    }

    @Override
    public void setSteerPwrIO(double SteerPwr) {
        STEER_MOTOR.set(SteerPwr);
    }

    @Override
    public void setSteerBrakeModeIO(boolean enabled) {
        breakEnabled = enabled;
    }

    @Override
    public void setDrvSensorPositionIO(double sensorPos) {
        DRIVE_MOTOR.setPosition(sensorPos);
    }
    @Override
    public void reverseDriveIO(boolean enable) {
        DRIVE_MOTOR.setInverted(enable);
    }
    
    @Override
    public void resetMagEncoderIO() {
        magEnc.reset();
    }

}
