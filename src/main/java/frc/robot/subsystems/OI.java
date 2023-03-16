package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OI extends SubsystemBase
{
    public final boolean debug = true;


    public Joystick driverController;
    public Joystick operatorController;
    Debouncer parkingBrakeDebouncer = new Debouncer(0.5);
    //private BooleanSupplier isCube;
    private boolean isCubeMode;


    /** Setting up which controllor is which
     * Drive Controller is controllor 0
     * Operator ConTroller is controllor 1
     * 
     * Sets what zero is on Driver and Operator Controllors
     */
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
         //isCube = new BooleanSupplier() {
           // return isCubeMode;
         //};
        isCubeMode = true;
    }

    public static void initPreferences() 
    {
  
    }

    /**
     * Sets the Driver and Operator's controllers to zero when we Enable
     */
    public void onEnable() {
        zeroDriverController();
        zeroOperatorController();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    //------- Driver Controller Joysticks Below

    /**Zero's out Driver Controller
     * @param value - Value to Clamp
     * @param low - The lower boundary to which to clamp value.
     * @param high - The highest boundary to which to clamp value.
    */

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

    public boolean isCubeMode() {
        return isCubeMode;
    }

    public void setCubeMode(){
        isCubeMode = true;
    }
    
    public void setConeMode(){
        isCubeMode = false;
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
    
        /**Zero's out Operator Controller
     * @param value - Value to Clamp
     * @param low - The lower boundary to which to clamp value.
     * @param high - The highest boundary to which to clamp value.
    */
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
        double value = MathUtil.clamp(2.0 * (operatorController.getRawAxis(0) - (OPERATOR_LEFT_X_MAX + OPERATOR_LEFT_X_MIN) * 0.5) / (OPERATOR_LEFT_X_MAX - OPERATOR_LEFT_X_MIN) - OPERATOR_LEFT_X_ZERO, -1, 1);
        if(Math.abs(value) < .35){value = 0;} //sets deadzone equal to .35
        return value;
    }

    private final double OPERATOR_LEFT_Y_MIN = -1;
    private final double OPERATOR_LEFT_Y_MAX = 1;
    private double OPERATOR_LEFT_Y_ZERO = 0;
    public double getOperatorLeftY() {
        return MathUtil.clamp(2.0 * (operatorController.getRawAxis(1) - (OPERATOR_LEFT_Y_MAX + OPERATOR_LEFT_Y_MIN) * 0.5) / (OPERATOR_LEFT_Y_MAX - OPERATOR_LEFT_Y_MIN) - OPERATOR_LEFT_Y_ZERO, -1, 1);
    }

    private final double OPERATOR_RIGHT_X_MIN=-1;
    private final double OPERATOR_RIGHT_X_MAX = 1;
    private double OPERATOR_RIGHT_X_ZERO = 0;
    public double getOperatorRightX() {
        double value = MathUtil.clamp(2.0 * (operatorController.getRawAxis(4) - (OPERATOR_RIGHT_X_MAX + OPERATOR_RIGHT_X_MIN) * 0.5) / (OPERATOR_RIGHT_X_MAX - OPERATOR_RIGHT_X_MIN) - OPERATOR_RIGHT_X_ZERO, -1, 1);
        if(Math.abs(value) < .35){value = 0;} //sets deadzone equal to .35
        return value;
    }
    
    private final double OPERATOR_RIGHT_Y_MIN = -1;
    private final double OPERATOR_RIGHT_Y_MAX = 1;
    private double OPERATOR_RIGHT_Y_ZERO = 0;
    public double getOperatorRightY() {
        return MathUtil.clamp(2.0 * (operatorController.getRawAxis(5) - (OPERATOR_RIGHT_Y_MAX + OPERATOR_RIGHT_Y_MIN) * 0.5) / (OPERATOR_RIGHT_Y_MAX - OPERATOR_RIGHT_Y_MIN) - OPERATOR_RIGHT_Y_ZERO, -1, 1);
    }

    //-------Driver Controller below

    /**
     * @return The Value of the driver controller's right Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getDriverRightTrigger(){
        return driverController.getRawAxis(3);
    }

    /**
     * @return The Value of the driver controller's left Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getDriverLeftTrigger(){
        return driverController.getRawAxis(2);
    }

        /**
     * @return The Value of the driver controller's left bumper
     * @param input - The current value of the input stream.
     * @return The debounced value of the input stream.     
     */
    public boolean getLeftBumper()
    {
        return parkingBrakeDebouncer.calculate(driverController.getRawButton(5));
    }
     /**
     * @return The Value of the driver controller's menu button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getRightBumper()
    {
        return driverController.getRawButton(6);
    }

     /**
     * @return The Value of the driver controller's 2 square button
     * @param button - The button index, beginning at 1.
     * @return Whether the button was pressed since the last check.
     */
    public boolean getFieldCentricToggle()
    {
        return driverController.getRawButtonPressed(7);
    }
    
     /**
     * @return The Value of the driver controller's menu button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getMenuButton()
    {
        return driverController.getRawButton(8);
    }
    
     /**
     * @return The Value of the driver controller's X button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getXButton()
    {
        return driverController.getRawButtonPressed(3);
    }

     /**
     * @return The Value of the driver controller's A button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getAButton()
    {
        return driverController.getRawButton(1);
    }

     /**
     * @return The Value of the driver controller's Y button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getYButton()
    {
        return driverController.getRawButton(4);
    }

     /**
     * @return The Value of the driver controller's B button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getBButton()
    {
        return driverController.getRawButton(2);
    }

     /**
     * @return The Value of the driver controller's DPad button
     * @return the angle of the POV in degrees, or -1 if the POV is not pressed.
     */
    public int getDPad(){
        return driverController.getPOV();
    }


    //-------- Operator Controller Below

    /**
     * @return The Value of the operator controller's right Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getOperatorRightTrigger(){
        return driverController.getRawAxis(3);
    }

    /**
     * @return The Value of the operator controller's left Trigger
     * @param axis - The axis to read, starting at 0.
     * @return The value of the axis.
     */
    public double getOperatorLeftTrigger(){
        return driverController.getRawAxis(2);
    }

    /**
     * @return The Value of the operator controller's left bumper
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorLeftBumper()
    {
        return operatorController.getRawButton(5);
    }

    /**
     * @return The Value of the operator controller's right bumper
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorRightBumper()
    {
        return operatorController.getRawButton(6);
    }

    /**
     * @return The Value of the operator controller's view button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorViewButton()
    {
        return operatorController.getRawButton(7);
    }
    
    /**
     * @return The Value of the operator controller's Menu Button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorMenuButton()
    {
        return operatorController.getRawButton(8);
    }
    
    /**
     * @return The Value of the operator controller's X button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorXButton()
    {
        return operatorController.getRawButton(3);
    }

    /**
     * @return The Value of the operator controller's A button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorAButton()
    {
        return operatorController.getRawButton(1);
    }

    /**
     * @return The Value of the operator controller's Y button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorYButton(){
        return operatorController.getRawButton(4);
    }


    /**
     * @return The Value of the operator controller's X button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorBButton(){
        return operatorController.getRawButton(2);
    }


    /**
     * @return The Value of the operator controller's top left DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadTopLeft(){
        return (operatorController.getPOV() == 315);
    }

    /**
     * @return The Value of the operator controller's left DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadLeft(){
        return (operatorController.getPOV() == 270);
    }

    /**
     * @return The Value of the operator controller's botton left DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadBottomLeft(){
        return (operatorController.getPOV() == 225);
    }

    /**
     * @return The Value of the operator controller's down DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadDown(){
        return (operatorController.getPOV() == 180);
    }

    /**
     * @return The Value of the operator controller's botton right DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadBottomRight(){
        return (operatorController.getPOV() == 135);
    }

    /**
     * @return The Value of the operator controller's right DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadRight(){
        return (operatorController.getPOV() == 90);
    }    

    /**
     * @return The Value of the operator controller's top right DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadTopRight(){
        return (operatorController.getPOV() == 45);
    }


    /**
     * @return The Value of the operator controller's up DPad button
     * @param Button - The button number to be read (starting at 1)
     * @return The state of the button.
     */
    public boolean getOperatorDPadUp(){
        return (operatorController.getPOV() == 0);
    }

    // public void setRumble(double val){
    //     operatorController.setRumble(RumbleType.kLeftRumble, val);
    //     operatorController.setRumble(RumbleType.kRightRumble, val);
    // }
    
}