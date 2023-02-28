package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class EngageGyroBalance extends CommandBase {
    DriveSubsystem drivetrain;
    ChassisSpeeds chassisSpeeds;
    Pose2d robotPose;
    boolean inverted;
    private double maxLinearVelocity;
    private double linearVelocity;
    private double startTime;
    private double currentPitch;
    private double pitchRate;
    private double loopTime;

    public EngageGyroBalance(DriveSubsystem ds, double maxSpeed, boolean inverted){
        // Use addRequirements() here to declare subsystem dependencies.
        this.drivetrain = ds;
        maxLinearVelocity = maxSpeed;
        this.inverted = inverted;
        addRequirements(ds);
    }

    @Override 
    public void initialize(){
        startTime = Integer.MAX_VALUE;
        if (inverted){ 
            linearVelocity = -maxLinearVelocity; 
        }
        else{ 
            linearVelocity = maxLinearVelocity; 
        }

        System.out.println("GryoBalance Initialized");
    } 
 
    @Override  
    public void execute(){ 
        loopTime = Timer.getFPGATimestamp() - loopTime;
        pitchRate = (drivetrain.getPitch() - currentPitch) / loopTime;
        SmartDashboard.putNumber("GyroBalance/Pitch Rate", pitchRate);
        SmartDashboard.putNumber("GyroBalance/Start Time", startTime);
        SmartDashboard.putNumber("GyroBalance/Loop Time", loopTime); 
        SmartDashboard.putNumber("GyroBalance/Current Pitch", currentPitch); 
 
            robotPose = drivetrain.getOdometry(); 
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds( 
            linearVelocity, 
            0,  
            0, 
            Rotation2d.fromDegrees(drivetrain.getHeading())); // get fused heading 
            drivetrain.setChassisSpeeds(chassisSpeeds); 
 
            if (Math.abs(drivetrain.getPitch()) < 5){ 
                if (startTime == Integer.MAX_VALUE){ 
                    startTime = Timer.getFPGATimestamp();
                }
                robotPose = drivetrain.getOdometry();
                chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(linearVelocity * -0.5,0,0, Rotation2d.fromDegrees(drivetrain.getHeading()));
                drivetrain.setChassisSpeeds(chassisSpeeds); 
            }
        currentPitch = drivetrain.getPitch();
        loopTime = Timer.getFPGATimestamp();
    }

    @Override
    public void end(boolean interrupted){
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, Rotation2d.fromDegrees(drivetrain.getHeading()));
        drivetrain.setChassisSpeeds(chassisSpeeds);
    }

    @Override
    public boolean isFinished(){
        if (Timer.getFPGATimestamp() > (startTime + 0.5) && Math.abs(pitchRate) < 7){
            return false; //change back to true after check if will go backwards
        }
        else{
            return false;
        }
    }
}
