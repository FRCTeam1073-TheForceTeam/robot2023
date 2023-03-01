package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ParkingBrake extends CommandBase
{
    DriveSubsystem drivetrain;


    public ParkingBrake(DriveSubsystem ds)
    {
        this.drivetrain = ds;
        addRequirements(ds);
    }

    @Override
    public void initialize()
    {
        drivetrain.parkingBrake(true);
    }

    @Override
    public void execute()
    {
        // if (!isOn)
        // {
        //     drivetrain.parkingBrake(true);
        // }
        // else
        // {
        //     drivetrain.parkingBrake(false);
        // }
        // //isParkingBrakeOn = true;
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.parkingBrake(false);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
