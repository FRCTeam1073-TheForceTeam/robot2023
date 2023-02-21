package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private int phase;

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
            linearVelocity = -maxLinearVelocity;
        }
        else
        {
            linearVelocity = maxLinearVelocity;
        }
        phase = 0;
    }

    @Override 
    public void execute()
    {
        robotPose = drivetrain.getOdometry();
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
          linearVelocity,
          0, 
          0,
          Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading
        drivetrain.setChassisSpeeds(chassisSpeeds);
        if (Math.abs(drivetrain.getPitch()) > 13) 
        {
            phase = 1;
            //return true;
        }
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        if (phase == 1 && Math.abs(drivetrain.getPitch()) < 5)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
