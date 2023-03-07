package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Bling;
import frc.robot.subsystems.DriveSubsystem;

public class ParkingBrake extends CommandBase
{
    DriveSubsystem drivetrain;
    Bling bling;


    public ParkingBrake(DriveSubsystem ds, Bling bling)
    {
        this.drivetrain = ds;
        this.bling = bling;
        addRequirements(ds);
    }

    @Override
    public void initialize()
    {
        drivetrain.parkingBrake(true); // turns on the parking brake
    }

    @Override
    public void execute()
    {
        bling.clearLEDs();
        bling.setSlot(1, 255, 0, 0); // sets the LEDs
        bling.setSlot(2, 255, 160, 0);
        bling.setSlot(3, 255, 255, 0);
        bling.setSlot(4, 0, 255, 0);
        bling.setSlot(5, 0, 0, 255);
        bling.setSlot(6, 255, 0, 255);
        bling.setSlot(7, 255, 255, 255);
    }

    @Override
    public void end(boolean interrupted)
    {
        drivetrain.parkingBrake(false); // turns off the parking brake
        bling.clearLEDs(); // turns off the LEDs
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
