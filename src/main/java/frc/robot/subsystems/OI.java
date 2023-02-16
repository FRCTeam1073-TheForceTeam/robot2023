package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI extends SubsystemBase
{
    public final boolean debug = true;


    public Joystick driverController;
    public Joystick operatorController;



    public OI() 
    {
        driverController = new Joystick(0);
        operatorController = new Joystick(1);
        LEFT_X_ZERO = 0;
        LEFT_Y_ZERO = 0;
        RIGHT_X_ZERO = 0;
        RIGHT_Y_ZERO = 0;
         zeroDriverController();
         zeroOperatorController();
    }

    public static void initPreferences() 
    {
  
    }

    public void onEnable() {
        zeroDriverController();
        zeroOperatorController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    //------- Driver Controller Joysticks Below

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

    //------- Operator Controller Joysticks Below
    
    public void zeroOperatorController() {
        //Sets all the offsets to zero, then uses whatever value it returns as the new offset.
        OPERATOR_LEFT_Y_ZERO = 0;
        OPERATOR_LEFT_X_ZERO = 0;
        OPERATOR_RIGHT_X_ZERO = 0;
        OPERATOR_RIGHT_Y_ZERO = 0;
        OPERATOR_LEFT_X_ZERO = getOperatorLeftX();
        OPERATOR_LEFT_Y_ZERO = getOperatorLeftY();
        OPERATOR_RIGHT_X_ZERO = getOperatorRightX();
        OPERATOR_RIGHT_Y_ZERO = getOperatorRightY();
    }

    private final double OPERATOR_LEFT_X_MIN = -1;
    private final double OPERATOR_LEFT_X_MAX = 1;
    private double OPERATOR_LEFT_X_ZERO = 0;
    public double getOperatorLeftX() {
        double value = MathUtil.clamp(2.0 * (driverController.getRawAxis(0) - (OPERATOR_LEFT_X_MAX + OPERATOR_LEFT_X_MIN) * 0.5) / (OPERATOR_LEFT_X_MAX - OPERATOR_LEFT_X_MIN) - OPERATOR_LEFT_X_ZERO, -1, 1);
        if(Math.abs(value) < .35){value = 0;} //sets deadzone equal to .35
        return value;
    }

    private final double OPERATOR_LEFT_Y_MIN = -1;
    private final double OPERATOR_LEFT_Y_MAX = 1;
    private double OPERATOR_LEFT_Y_ZERO = 0;
    public double getOperatorLeftY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(1) - (OPERATOR_LEFT_Y_MAX + OPERATOR_LEFT_Y_MIN) * 0.5) / (OPERATOR_LEFT_Y_MAX - OPERATOR_LEFT_Y_MIN) - OPERATOR_LEFT_Y_ZERO, -1, 1);
    }

    private final double OPERATOR_RIGHT_X_MIN=-1;
    private final double OPERATOR_RIGHT_X_MAX = 1;
    private double OPERATOR_RIGHT_X_ZERO = 0;
    public double getOperatorRightX() {
        double value = MathUtil.clamp(2.0 * (driverController.getRawAxis(4) - (OPERATOR_RIGHT_X_MAX + OPERATOR_RIGHT_X_MIN) * 0.5) / (OPERATOR_RIGHT_X_MAX - OPERATOR_RIGHT_X_MIN) - OPERATOR_RIGHT_X_ZERO, -1, 1);
        if(Math.abs(value) < .35){value = 0;} //sets deadzone equal to .35
        return value;
    }
    
    private final double OPERATOR_RIGHT_Y_MIN = -1;
    private final double OPERATOR_RIGHT_Y_MAX = 1;
    private double OPERATOR_RIGHT_Y_ZERO = 0;
    public double getOperatorRightY() {
        return MathUtil.clamp(2.0 * (driverController.getRawAxis(5) - (OPERATOR_RIGHT_Y_MAX + OPERATOR_RIGHT_Y_MIN) * 0.5) / (OPERATOR_RIGHT_Y_MAX - OPERATOR_RIGHT_Y_MIN) - OPERATOR_RIGHT_Y_ZERO, -1, 1);
    }

    //-------Driver Controller below

    public double getDriverRightTrigger(){
        return driverController.getRawAxis(3);
    }

    public double getDriverLeftTrigger(){
        return driverController.getRawAxis(2);
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
    
    public boolean getXButton()
    {
        return driverController.getRawButtonPressed(3);
    }

    public boolean getAButton()
    {
        return driverController.getRawButton(1);
    }

    public int getDPad(){
        return driverController.getPOV();
    }

    //-------- Operator Controller Below

    public boolean getOperatorLeftBumper()
    {
        return operatorController.getRawButton(5);
    }

    public boolean getOperatorRightBumper()
    {
        return operatorController.getRawButton(6);
    }
    
    public boolean getOperatorMenuButton()
    {
        return operatorController.getRawButton(8);
    }
    
    public boolean getOperatorXButton()
    {
        return operatorController.getRawButton(3);
    }

    public boolean getOperatorAButton()
    {
        return operatorController.getRawButton(1);
    }

    public boolean getOperatorYButton(){
        return operatorController.getRawButton(4);
    }

    public boolean getOperatorBButton(){
        return operatorController.getRawButton(2);
    }

    // public void setRumble(double val){
    //     operatorController.setRumble(RumbleType.kLeftRumble, val);
    //     operatorController.setRumble(RumbleType.kRightRumble, val);
    // }
    
}