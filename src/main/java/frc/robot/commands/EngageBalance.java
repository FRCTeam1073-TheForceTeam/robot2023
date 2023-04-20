package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Preferences;
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
    private double startTime = 0.0;
    private int phase;

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
        startTime = Timer.getFPGATimestamp();
        if (inverted)
        {
            linearVelocity = -maxLinearVelocity; // sets the direction
        }
        else
        {
            linearVelocity = maxLinearVelocity;
        }
    }

    public static void initPreferences() {
        Preferences.initDouble("EngageBalance.maxSpeed", 0.7);
    }

    @Override 
    public void execute()
    {
        // if (Math.abs(drivetrain.getPitch()) < 5 && phase == 0)
        // {
        //     phase = 1;
        // }

        // if (phase == 1)
        // {
        //     if (startTime == 0.0)
        //     {
        //         startTime = Timer.getFPGATimestamp();
        //     }
        //     robotPose = drivetrain.getOdometry();
        //     chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        //     drivetrain.setChassisSpeeds(chassisSpeeds);
        // } 
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        drivetrain.setChassisSpeeds(chassisSpeeds); // makes the robot back up at half speed
    }

    @Override
    public void end(boolean interrupted)
    {
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
    } // stops the robot

    @Override
    public boolean isFinished()
    {
        if (Timer.getFPGATimestamp() > startTime + 0.5 && Math.abs(drivetrain.getPitch()) < 8.0)
        { // triggers once the robot drives up to the halfway point again
            return true;
        }
        else
        {
            return false;
        }
    }
}
