package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI extends SubsystemBase
{
    public final boolean debug = true;


    public Joystick driverController;



    public OI() 
    {
        driverController = new Joystick(0);
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
         zeroDriverController();
    }

    public void onEnable() {
        zeroDriverController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    public void zeroDriverController() {
        //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
        LEFT_X_ZERO = getDriverLeftX();
        LEFT_Y_ZERO = getDriverLeftY();
        RIGHT_X_ZERO = getDriverRightX();
        RIGHT_Y_ZERO = getDriverRightY();
    }

    private final double LEFT_X_MIN = -1;
    private final double LEFT_X_MAX = 1;
    private double LEFT_X_ZERO = 0;
    public double getDriverLeftX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(0) - (LEFT_X_MAX + LEFT_X_MIN) * 0.5) / (LEFT_X_MAX - LEFT_X_MIN) - LEFT_X_ZERO, -1, 1);
    }

    private final double LEFT_Y_MIN = -1;
    private final double LEFT_Y_MAX = 1;
    private double LEFT_Y_ZERO = 0;
    public double getDriverLeftY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(1) - (LEFT_Y_MAX + LEFT_Y_MIN) * 0.5) / (LEFT_Y_MAX - LEFT_Y_MIN) - LEFT_Y_ZERO, -1, 1);
    }

    private final double RIGHT_X_MIN=-1;
    private final double RIGHT_X_MAX = 1;
    private double RIGHT_X_ZERO = 0;
    public double getDriverRightX() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(4) - (RIGHT_X_MAX + RIGHT_X_MIN) * 0.5) / (RIGHT_X_MAX - RIGHT_X_MIN) - RIGHT_X_ZERO, -1, 1);
    }
    private final double RIGHT_Y_MIN = -1;
    private final double RIGHT_Y_MAX = 1;
    private double RIGHT_Y_ZERO = 0;
    public double getDriverRightY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(5) - (RIGHT_Y_MAX + RIGHT_Y_MIN) * 0.5) / (RIGHT_Y_MAX - RIGHT_Y_MIN) - RIGHT_Y_ZERO, -1, 1);
    }

    public boolean getLeftBumper()
    {
        return driverController.getRawButton(5);
    }

    public boolean getRightBumper()
    {
        return driverController.getRawButton(6);
    }

    public boolean getFieldCentricToggle()
    {
        return driverController.getRawButtonPressed(7);
    }
    
    public boolean getMenuButton()
    {
        return driverController.getRawButton(8);
    }
    
    public void setRumble(double val){
        //operatorController.setRumble(RumbleType.kLeftRumble, val);
       // operatorController.setRumble(RumbleType.kRightRumble, val);
    }
    
   
       
}