package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Sharkfin {

    public double MAX_FIN_LEN = 108000;

    // Setting up the lead controller. Make it sharp.
    private double lead_kp = 0.4;
    private double lead_ki = 0.0001;

    private double pull_kp = 0.1; //0.5;
    private double pull_ki =  0; //0.0001;
    private double pull_kf = 1; //5;
    private double pull_kd = 1;
    private double lead_iz = 0; //Sensor units
   
    // private double lead_kp = 0.4;
    // private double lead_ki = 0.0000;
    // private double lead_iz = 1000; //Sensor units

    private final double homePWM = 0.3;

    // private double follow_kf = 1;
    // private double follow_kp = 0.000;
    // private double follow_ki = 0;
    

    public WPI_TalonFX left;
    public WPI_TalonFX rght;

    private final int masterID;

    public double setpoint;
    private double SAFETY_BLOCK = 0.97;

    int normalAccel = 40000;
    int normalCruise = 15000;

    int pullCruise = 2000;
    int pullAccel = 4000;

    int scurve = 7;  


    // private TalonFXConfiguration _leftConfig = new TalonFXConfiguration();
    // private TalonFXConfiguration _rightConfig = new TalonFXConfiguration();
    
    Sharkfin(WPI_TalonFX rght_in, WPI_TalonFX left_in) {

        left = left_in;
        rght = rght_in;

        left.configFactoryDefault();
        rght.configFactoryDefault();

        // MASTER ID: LEFT
        masterID = left.getDeviceID();
        
        // Proper inversions for our lift
        left.setInverted(TalonFXInvertType.CounterClockwise); //false
        rght.setInverted(TalonFXInvertType.Clockwise); //true
    
        rght.setSensorPhase(false);
        left.setSensorPhase(true);

        // SETTING LOOPS

        // PID slot 0: Normal movement
        left.config_kP(0, lead_kp);
        left.config_kI(0, lead_ki);
        left.config_IntegralZone(0, lead_iz);
        rght.config_kP(0, lead_kp);
        rght.config_kI(0, lead_ki);
        rght.config_IntegralZone(0, lead_iz);

        // PID slot 1: pull very hard
        left.config_kP(1, pull_kp);
        left.config_kI(1, pull_ki);
        rght.config_kP(1, pull_kp);
        rght.config_kI(1, pull_ki);
        left.config_kF(1, pull_kf);
        rght.config_kF(1, pull_kf);
        left.config_kD(1, pull_kd);
        rght.config_kD(1, pull_kd);
 

        // Setting up limit switches
        left.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);
        left.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);

        rght.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);
        rght.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);

        // // Current Limits
        // left.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(true,25,35,1.0));
        // rght.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(true,25,35,1.0));
        // left.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(true,25,35,1.0));
        // rght.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(true,25,35,1.0));

        left.configMotionAcceleration(normalAccel);
        left.configMotionCruiseVelocity(normalCruise);
        rght.configMotionAcceleration(normalAccel);
        rght.configMotionCruiseVelocity(normalCruise); 


        // Setting Up Software Limits
        // left.configForwardSoftLimitEnable(true);
        // rght.configForwardSoftLimitEnable(true);
        // left.configForwardSoftLimitThreshold(105000);
        // rght.configForwardSoftLimitThreshold(105000);
   
        // left.configReverseSoftLimitEnable(true);
        // rght.configReverseSoftLimitEnable(true);
        // left.configReverseSoftLimitThreshold(3000);
        // rght.configReverseSoftLimitThreshold(3000);
        
        // Brake Modes - might need to change based on game.

        rght.setNeutralMode(NeutralMode.Coast);
        left.setNeutralMode(NeutralMode.Coast);

        // left.configClearPositionOnLimitF(true, 10);
        // rght.configClearPositionOnLimitF(true, 10);

        rght.setSelectedSensorPosition(0);
        left.setSelectedSensorPosition(0);

        // Set max rate for sharkfins. Prevents hopping around.

        
        

    }
    int last_status;


    public void shark_initial() {
        last_status = 0;

        left.set(ControlMode.PercentOutput, 0);
        rght.set(ControlMode.PercentOutput, 0);

        // Brake Modes - might need to change based on game.

        rght.setNeutralMode(NeutralMode.Brake);
        left.setNeutralMode(NeutralMode.Brake);

        setpoint = left.getSelectedSensorPosition();
    }

    public boolean safety = true;
    // Safety should ALWAYS be true. The only time it is false is if the 
    // drive system does not have the power.
 

    public void sharkPeriodic(double in_set, int status){
    //setpoint - the setpoint for the position. -1 to 1 mapped to 0 to 90k
    //  Statuses are below

        // If we change statuses
        if (last_status != status) {
            
            switch (last_status) {

                case 5: /// go back to our normal loops and profiling
    
                    rght.selectProfileSlot(0, 0);
                    left.selectProfileSlot(0, 0);
    
                    left.configMotionAcceleration(normalAccel);
                    left.configMotionCruiseVelocity(normalCruise);
                    rght.configMotionAcceleration(normalAccel);
                    rght.configMotionCruiseVelocity(normalCruise); 
                    break;
                
                case 6: /// go back to our normal loops and profiling
    
                    rght.selectProfileSlot(0, 0);
                    left.selectProfileSlot(0, 0);

                    left.configMotionAcceleration(normalAccel);
                    left.configMotionCruiseVelocity(normalCruise);
                    rght.configMotionAcceleration(normalAccel);
                    rght.configMotionCruiseVelocity(normalCruise); 
                    break;

            }
            
            
            switch (status) {

            case 1:
                // Shifting to hold where it is at
                setpoint = left.getSelectedSensorPosition();
                break;

            case 5:
                // Switch all our loops and variables

                // Vel Motion Profile
                rght.selectProfileSlot(1, 0);
                left.selectProfileSlot(1, 0);

                left.configMotionAcceleration(pullAccel);
                left.configMotionCruiseVelocity(pullCruise);
                rght.configMotionAcceleration(pullAccel);
                rght.configMotionCruiseVelocity(pullCruise); 

                rght.configMotionSCurveStrength(scurve);
                left.configMotionSCurveStrength(scurve);
                
                break;
            case 6:
                rght.selectProfileSlot(1, 0);
                left.selectProfileSlot(1, 0);

                left.configMotionAcceleration(pullAccel);
                left.configMotionCruiseVelocity(pullCruise);
                rght.configMotionAcceleration(pullAccel);
                rght.configMotionCruiseVelocity(pullCruise); 

                rght.configMotionSCurveStrength(scurve);
                left.configMotionSCurveStrength(scurve);
                break;
            }
            


        }

        switch (status) {

            case 1: // Holding setpoint

                left.set(ControlMode.Position, setpoint);
                rght.set(ControlMode.Position, setpoint);
                break;

            case 2: // moving to specific position, in_set
                rght.selectProfileSlot(0, 0);
                left.selectProfileSlot(0, 0);
                setpoint = remap(in_set);
                left.set(ControlMode.MotionMagic, setpoint);
                rght.set(ControlMode.MotionMagic, setpoint);
                break;

            case 3: // Adjusting by the manual control up
                rght.selectProfileSlot(0, 0);
                left.selectProfileSlot(0, 0);    
                setpoint = remap(SAFETY_BLOCK * 1);
                left.set(ControlMode.MotionMagic, setpoint);
                rght.set(ControlMode.MotionMagic, setpoint);
                break;
                

            case 4: // Adjusting by the manual control down
                rght.selectProfileSlot(0, 0);
                left.selectProfileSlot(0, 0);    
                setpoint = remap(SAFETY_BLOCK*-1);
                left.set(ControlMode.MotionMagic, setpoint);
                rght.set(ControlMode.MotionMagic, setpoint);
                break;

            case 5: // PID Loop 1: pull very hard

                setpoint = remap(-1);
                left.set(ControlMode.MotionMagic, setpoint);
                rght.set(ControlMode.MotionMagic, setpoint);
                break;
            
            case 6: // Slow move forward
                setpoint = remap(1);
                left.set(ControlMode.MotionMagic, setpoint);
                rght.set(ControlMode.MotionMagic, setpoint);
                break;

            case -1: // Home
                left.set(ControlMode.PercentOutput, homePWM);
                rght.set(ControlMode.PercentOutput, homePWM);
                break;

            default: // Off
                left.set(ControlMode.PercentOutput, 0);
                rght.set(ControlMode.PercentOutput, 0);
                break;

        }

        last_status = status;

        // Fake Zero the limit switches

        // if (left.isFwdLimitSwitchClosed() == 1) {
        //     left.setSelectedSensorPosition(MAX_FIN_LEN);
        // }

        // if (rght.isFwdLimitSwitchClosed() == 1) {
        //     rght.setSelectedSensorPosition(MAX_FIN_LEN);
        // }

    }

    private double remap (double input) {

        input = input + 1;
        return 1 * input * MAX_FIN_LEN/2;

    }

    private double unmap (double input) {
        return (2 * input / MAX_FIN_LEN) - 1;
    }
}
