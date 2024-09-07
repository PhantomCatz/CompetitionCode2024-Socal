package frc.robot.CatzSubsystems.DriveAndRobotOrientation.drivetrain;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C.Port;

public class GyroIONavX implements GyroIO 
{
    private final AHRS navX;

    public GyroIONavX() {
        navX = new AHRS(Port.kMXP, (byte) 200);
        //navX.enableLogging(true); //TODO renable this when gyro is connected
        navX.reset(); // implicitly sets the gyro to 0 heading
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
      inputs.gyroAngle = navX.getAngle(); //Acumulated Yaw
      inputs.gyroYawDegrees = (navX.getYaw());
      inputs.gyroRollDegrees = navX.getRoll();
      inputs.gyroConnected = navX.isConnected();
      inputs.gyroAngleVel = navX.getRate();
      inputs.gyroAccelX   = navX.getWorldLinearAccelX();
      inputs.gyroAccelY   = navX.getWorldLinearAccelY();
    }

}
