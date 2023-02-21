package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ParkingBrake extends CommandBase
{
    DriveSubsystem drivetrain;
    boolean isOn;

    public ParkingBrake(DriveSubsystem ds, boolean isOn)
    {
        this.drivetrain = ds;
        addRequirements(ds);
        this.isOn = isOn;
    }

    @Override
    public void initialize()
    {
        
    }

    @Override
    public void execute()
    {
        if (!isOn)
        {
            drivetrain.parkingBrake(true);
        }
        else
        {
            drivetrain.parkingBrake(false);
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
