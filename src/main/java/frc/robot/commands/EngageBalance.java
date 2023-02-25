package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageBalance extends CommandBase
{
    DriveSubsystem drivetrain;
    ChassisSpeeds chassisSpeeds;
    Pose2d robotPose;
    boolean inverted;
    private double maxLinearVelocity;
    private double linearVelocity;
    private double startTime;

    public EngageBalance(DriveSubsystem ds, double maxSpeed, boolean inverted) 
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
        startTime = Timer.getFPGATimestamp();
    }

    @Override 
    public void execute()
    {
        if (Math.abs(drivetrain.getPitch()) < 5)
        {
            
        }
        robotPose = drivetrain.getOdometry();
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        drivetrain.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        if (Timer.getFPGATimestamp() > startTime + 0.5 && Math.abs(drivetrain.getPitch()) < 2)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}