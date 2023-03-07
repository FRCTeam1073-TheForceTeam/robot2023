package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageRateBalance extends CommandBase
{
    DriveSubsystem drivetrain;
    ChassisSpeeds chassisSpeeds;
    Pose2d robotPose;
    boolean inverted;
    private double maxLinearVelocity;
    private double linearVelocity;
    private double startTime = 0.0;
    private int phase;

    public EngageRateBalance(DriveSubsystem ds, double maxSpeed, boolean inverted) 
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
        phase = 0; 
        if (inverted)
        {
            linearVelocity = -maxLinearVelocity; // sets the direction it will go 
        }
        else
        {
            linearVelocity = maxLinearVelocity;
        }
    }

    @Override 
    public void execute()
    {
        /*if (Math.abs(drivetrain.getPitch()) < 5 && phase == 0) // activates once the robot nears the halfway mark
        {
            phase = 1;
        }

        if (phase == 1)
        {
            if (startTime == 0.0) // sets the timer which makes sure the robot goes backwards at least a set amount of time
            {
                startTime = Timer.getFPGATimestamp();
            }
            robotPose = drivetrain.getOdometry();
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
            drivetrain.setChassisSpeeds(chassisSpeeds); // makes the robot back up at half speed
        } */
        robotPose = drivetrain.getOdometry();
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        drivetrain.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        /*if (Timer.getFPGATimestamp() > startTime + 0.25 && Math.abs(drivetrain.getPitchRate()) > 15)
        { // triggers once the robot starts nearing the middle
            return true;
        }*/
        if (Math.abs(drivetrain.getPitchRate()) > 10)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
