package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageDriveUp extends CommandBase
{
    DriveSubsystem drivetrain;
    ChassisSpeeds chassisSpeeds;
    Pose2d robotPose;
    boolean inverted;
    private double maxLinearVelocity;
    private double linearVelocity;
    private double endPitch;

    public EngageDriveUp(DriveSubsystem ds, double maxSpeed, boolean inverted) 
    {
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = ds;
        maxLinearVelocity = maxSpeed;
        this.inverted = inverted;
        addRequirements(ds);
    }

    @Override
    public void initialize()
    {
        if (inverted)
        {
            linearVelocity = -maxLinearVelocity; // sets the direction
        }
        else
        {
            linearVelocity = maxLinearVelocity;
        }
    }

    public static void initPreferences() 
    {
        Preferences.initDouble("EngageDriveUp.maxSpeed", 0.9);
        Preferences.initDouble("EngageDriveUp.endPitch", 13.0);
    }

    @Override 
    public void execute()
    {
        robotPose = drivetrain.getOdometry();
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( // sets the speed
          linearVelocity,
          0, 
          0,
          Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
        drivetrain.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        if (Math.abs(drivetrain.getPitch()) > endPitch) // triggers when the robot starts driving onto the charging station
        {
            
            return true;
        }
        else
        {
            return false;
        }
    }
}
