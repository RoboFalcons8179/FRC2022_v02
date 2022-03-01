package frc.robot;

import java.security.KeyPair;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Sharkfin {

    public double MAX_FIN_LEN = 100000;

    // Setting up the lead controller. Make it sharp.
    private double lead_kp = 0.4;
    private double lead_ki = 0;
    private double lead_iz = 1000; //Sensor units

    private double follow_kf = 1;
    private double follow_kp = 0.000;
    private double follow_ki = 0;
    

    public WPI_TalonFX left;
    public WPI_TalonFX rght;

    private TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
    private TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
    
    Sharkfin(WPI_TalonFX rght_in, WPI_TalonFX left_in) {
        left = left_in;
        rght = rght_in;


        // // Configuring Sensors

        // // Telling the right motor to have its sensor available for remote sensing
        // _rightConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // //Telling the left motor to look at the right motor
        // _leftConfig.remoteFilter0.remoteSensorDeviceID = rght.getDeviceID();
        
        // // telling the left to look at the right's primary Remote sensor
        // _leftConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        // // Adjusting the signs so that for left side we are controlling to comand and 
        // // Right side we are driving towards the left motor. We are making both sensors positive,
        // // so we want to set it up to be the difference.

        // _leftConfig.diff1Term = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        // _leftConfig.diff0Term = TalonFXFeedbackDevice.RemoteSensor0.toFeedbackDevice();
        // _leftConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.SensorDifference.toFeedbackDevice();

        // _leftConfig.slot0.closedLoopPeriod = 20;

        
        // // Configuring Loops
        // _leftConfig.slot0.kP = follow_kp;
        // _leftConfig.slot0.kI = follow_ki;
        // _leftConfig.slot0.kF = follow_kf;


        // _rightConfig.slot0.integralZone = lead_iz;
        // _rightConfig.slot0.kP = lead_kp;
        // _rightConfig.slot0.kI = lead_ki;

        // // Apply settings
        // left.configAllSettings(_leftConfig);
        // rght.configAllSettings(_rightConfig);

        // Proper inversions for our lift
        left.setInverted(TalonFXInvertType.CounterClockwise); //false
        rght.setInverted(TalonFXInvertType.Clockwise); //true
    
        rght.setSensorPhase(false);
        left.setSensorPhase(true);

        // SETTING LOOPS

        left.config_kP(0, lead_kp);
        left.config_kI(0, lead_ki);
        left.config_IntegralZone(0, lead_iz);
        rght.config_kP(0, lead_kp);
        rght.config_kI(0, lead_ki);
        rght.config_IntegralZone(0, lead_iz);

        // Setting Motion Magic
        left.configMotionAcceleration(9999999);
        left.configMotionCruiseVelocity(6000);
        rght.configMotionAcceleration(9999999);
        rght.configMotionCruiseVelocity(6000);  

        // Brake Modes - might need to change based on game.
        rght.setNeutralMode(NeutralMode.Coast);
        left.setNeutralMode(NeutralMode.Coast);

        // Telling the controllers to follow these loops
        rght.selectProfileSlot(0, 0);
        left.selectProfileSlot(0, 0);

    }

    public void shark_initial() {

        left.set(ControlMode.PercentOutput, 0);
        rght.set(ControlMode.PercentOutput, 0);



    }

    public boolean safety = true;
    // Safety should ALWAYS be true. The only time it is false is if the 
    // drive system does not have the power.

    public void sharkPeriodic(double setpoint, boolean mode){
    //setpoint - the setpoint for the position. -1 to 1 mapped to 0 to 90k
    //  if in open loop mode, use the raw controller input [-1 to 1]
    //mode - motion magic (true) or open loop (false)

    if (mode) {


        left.set(ControlMode.MotionMagic, remap(setpoint));
        rght.set(ControlMode.MotionMagic, remap(setpoint));

    } else {
        if (safety) {
            left.setNeutralMode(NeutralMode.Coast);
            left.set(ControlMode.PercentOutput, 0);
            rght.set(ControlMode.PercentOutput, setpoint);
        } else {
            left.set(ControlMode.PercentOutput, setpoint);
            rght.set(ControlMode.PercentOutput, setpoint);
        }

    }

    }

    private double remap (double input) {

        input = input + 1;
        return input * MAX_FIN_LEN/2;

    }
}
