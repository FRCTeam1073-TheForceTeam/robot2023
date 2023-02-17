package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class PersistentParkingBrake extends CommandBase
{
    DriveSubsystem drivetrain;
    boolean isParkingBrakeOn;

    public PersistentParkingBrake(DriveSubsystem ds)
    {
        this.drivetrain = ds;
        addRequirements(ds);
    }

    @Override
    public void initialize()
    {
        isParkingBrakeOn = false;
    }

    @Override
    public void execute()
    {
        if (!isParkingBrakeOn)
        {
            drivetrain.parkingBrake(true);
        }
        else
        {
            
        }
        //isParkingBrakeOn = true;
    }

    @Override
    public void end(boolean interrupted)
    {

    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
