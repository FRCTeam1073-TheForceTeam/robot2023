package frc.robot.subsystems;

public class SwerveModuleIDConfig 
{
    public int driveMotorID = 0;
    public int steerMotorID = 0;
    public int steerEncoderID = 0;

    SwerveModuleIDConfig() {
    }

    /** Initialize a SwerveModuleIDConfig with drive, steer and encoder CAN ID. */
    SwerveModuleIDConfig(int drive, int steer, int encoder) {
        driveMotorID = drive;
        steerMotorID = steer;
        steerEncoderID = encoder;
    }
}
