package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class arm {

    private WPI_TalonFX left_arm,right_arm;

    public TalonFXInvertType right_arm_invert = TalonFXInvertType.CounterClockwise;
    public TalonFXInvertType left_arm_invert = TalonFXInvertType.Clockwise;


    // CONSTANTS

    // PWM signals that will let the arm go up.
    // Does not include the ARB Value
    private final double kUpAdjustMax = 0.2;
    private final double kDnAdjustMax = -0.2;

    // PWM for lowering to home.
    private final double homePWM = -0.1;

    // constants for the arm position
    private final double lower_deg = -37.8;
    private final double upper_deg = 26.;
    private final double min_sensor = 0;
    public final double max_sensor_balls = 57000;//60000
    private final double max_sensor = 77000;

        // Calculating A, can find empirically
        //rather than physics-ly
    // private final double maxTorque = 56;
    // private final double stallTorque = 4.6;
    // private final double gearRatio = 6.4*3*9;
    private final double A = 0.07;


    // Band for Swapping loops
    private final double loopZone = 2000;

    // constants for the loop

    private final double u_mmVel = 6000;
    private final double u_kp = 0.05;
    private final double u_ki = 0.00001;
    private final double u_kd = 0;
    private final double u_iz = 0;
    private final double u_kf = 0.09;
    private final double u_mmAcc = u_mmVel * 2;

    private final double d_mmVel = 10000;
    private final double d_kp = 0.00005;
    private final double d_ki = 0.0000;
    private final double d_kd = 0;
    private final double d_iz = 1000;
    private final double d_kf = 1023/d_mmVel;

    private final double n_kp = 0.00005;
    private final double n_ki = 0.0000;
    private final double n_kd = 0;
    private final double n_iz = 1000;

    private final double err = 1000;
    private final int s_curve = 3;    
    private final int neutralID = 0;
    private final int upslotID = 1;
    private final int downslotID = 2;


    // Calculated constants
    private final double ksensorToDeg;
    private final int masterID;


    // The position setpoint, can only be accessed by the functions.
    private static double setpoint;

    // SmartDashBoard Things
    public double Langle, Rangle;
    public double aux;
    public double Lcurr, Rcurr;
    public double set_out;

    public double ArbFeedback;

    

    arm(WPI_TalonFX left_arm_in, WPI_TalonFX right_arm_in) {

        left_arm = left_arm_in;
        right_arm = right_arm_in;

        masterID = left_arm.getDeviceID();

        // calculating the constants
        // A = gearRatio * maxTorque / stallTorque;


        ksensorToDeg = (max_sensor - min_sensor) / (upper_deg - lower_deg);

        setpoint = 0;

        left_arm.configFactoryDefault();
        right_arm.configFactoryDefault();

        right_arm.setInverted(right_arm_invert); // false
        left_arm.setInverted(left_arm_invert); // true

        right_arm.setSensorPhase(false);
        left_arm.setSensorPhase(false);

        left_arm.setNeutralMode(NeutralMode.Brake);
        right_arm.setNeutralMode(NeutralMode.Brake);
       
        left_arm.configVoltageCompSaturation(8);
        right_arm.configVoltageCompSaturation(8);
        right_arm.enableVoltageCompensation(true);
        left_arm.enableVoltageCompensation(true);

        left_arm.config_kP(upslotID, u_kp);
        left_arm.config_kI(upslotID, u_ki);
        left_arm.config_kD(upslotID, u_kd);
        left_arm.config_kF(upslotID, u_kf);
        left_arm.config_IntegralZone(upslotID, u_iz);

        left_arm.config_kP(downslotID, d_kp);
        left_arm.config_kI(downslotID, d_ki);
        left_arm.config_kD(downslotID, d_kd);
        left_arm.config_kF(downslotID, d_kf);
        left_arm.config_IntegralZone(downslotID, d_iz);

        left_arm.config_kP(neutralID, n_kp);
        left_arm.config_kI(neutralID, n_ki);
        left_arm.config_kD(neutralID, n_kd);
        left_arm.config_kF(neutralID, 0);
        left_arm.config_IntegralZone(neutralID, n_iz);
        
        left_arm.configMotionCruiseVelocity(u_mmVel);
        left_arm.configMotionAcceleration(u_mmAcc);
        left_arm.configMotionSCurveStrength(s_curve);
        

        left_arm.selectProfileSlot(upslotID, 0); // Primary loop


        // Current Limits
        // left_arm.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(true,25,35,1.0));
        // right_arm.configSupplyCurrentLimit(
        //     new SupplyCurrentLimitConfiguration(true,25,35,1.0));
        // left_arm.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(true,25,35,1.0));
        // right_arm.configStatorCurrentLimit(
        //     new StatorCurrentLimitConfiguration(true,25,35,1.0));

        // left_arm.configForwardSoftLimitThreshold(max_sensor - 3000);
        // left_arm.configReverseSoftLimitThreshold(min_sensor + 1000);
        // left_arm.configForwardSoftLimitEnable(true, 5);
        // left_arm.configReverseSoftLimitEnable(true, 5);


        // Setting up limit switches
        left_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);
        left_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);

        right_arm.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);
        right_arm.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, 
            LimitSwitchNormal.NormallyOpen);

        left_arm.configClearPositionOnLimitR(true, 25);
        right_arm.configClearPositionOnLimitR(true, 25);
        
        left_arm.setSelectedSensorPosition(0);
        right_arm.setSelectedSensorPosition(0);

        left_arm.configForwardSoftLimitThreshold(max_sensor_balls);
        right_arm.configForwardSoftLimitThreshold(max_sensor_balls);
        
        left_arm.configForwardSoftLimitEnable(true, 5);


        }

    public void arm_init() {

        left_arm.set(ControlMode.PercentOutput, 0);
        right_arm.set(ControlMode.PercentOutput, 0);

        right_arm.set(ControlMode.Follower, masterID);

        setSetpointSU(-3000);
        first = true;
        left_arm.selectProfileSlot(upslotID, 0);
    }
 
    public int status;

    private static double lockpoint;
    private int lastCommand = 0;
    private boolean lastSticky = false;
    public boolean first;


    public void arm_Periodic(double in_set, int command, boolean sticky) {
        // Command: 0 - Off
        // Command: 1 - Float
        // Command: 2 - Position Control
        // Command: 3 - Open Loop (POSSIBLY DANGEROUS - SHOULD PROBABLY SCALE DOWN)
        // Command: 4 - Adjust Up
        // Command: 5 - Adjust Down
        // Command: -1 - Home
        // Command: 10: Float in Position Control, default
        
        if (first) {
            setSetpointSU(-3000);
            sticky = true;
            first = false;
            command = 0;
        }
        if (sticky) {
            lockpoint = setpoint;
        } else
        if (command == 10) {
            setSetpointSU(lockpoint);

            if (lockpoint == 0) {
                lockpoint = -2000;
            }

            if (setpoint < 2000 && getCurrentPositionSU() < 0) {
                command = 99;
            }
        } 



        status = 0;

        findArbFB();
        double current_pos = left_arm.getSelectedSensorPosition();

        if (command == 2) {

            setSetpointSU(in_set);
            boolean tooHigh = current_pos > setpoint + loopZone;
            boolean tooLow = current_pos < setpoint - loopZone;

            if (!tooHigh && !tooLow) { // holding
                status = 1;
            } else 
            if (tooHigh) { // moving down
                status = 3;
            } else 
            if (tooLow) { // moving up
                status = 2;
            }
            
        } else 
        if (command == 10) {
            status = 1; // Hold Position reguardless of in_set 
        } else
        if (command == 0) { // turn off
            status = 99;
        } else
        if (command == 3) { // manual PWM (need in_set)
            status = 4;
        } else
        if (command == -1) { // Homing
            status = -1;
        } else
        if (command == 4) { // Adjust Up
            status = 5;
        } else
        if (command == 5) { // Adjust Down
            status = 6;
        } else {            // Float
            status = 0;
        }



        switch (status) {

            case 0: // Float

                left_arm.set(ControlMode.PercentOutput, 0, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                break;

            case 1: // holding position
                
                // Set up hold loop

                left_arm.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                 
                break;

            case 2: // moving up to specific position

                left_arm.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());               
                break;

            case 3: // moving down to specific position

                left_arm.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                break;

            case 4: // RAW OPEN LOOP - DANGEROUS
                left_arm.set(ControlMode.PercentOutput, in_set, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                if (sticky) {
                    lockpoint = getCurrentPositionSU();
                }
                break;

            case 5: // Adjust set up
                left_arm.set(ControlMode.PercentOutput, kUpAdjustMax, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                if (sticky) {
                    lockpoint = getCurrentPositionSU();
                }
                break; 
            case 6: // adjust set down
                left_arm.set(ControlMode.PercentOutput, kDnAdjustMax, DemandType.ArbitraryFeedForward, ArbFeedback);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());
                if (sticky) {
                    lockpoint = getCurrentPositionSU();
                }
                break;  
            case -1:
                // Homing the arms
                left_arm.set(ControlMode.PercentOutput, homePWM);
                right_arm.set(ControlMode.Follower, left_arm.getDeviceID());

                if (left_arm.isRevLimitSwitchClosed()==1 && right_arm.isRevLimitSwitchClosed()==1) {
                    left_arm.setSelectedSensorPosition(0);
                    right_arm.setSelectedSensorPosition(0);
                    setSetpointSU(0);
                    lockpoint = -1000;
                }
                break; 

             default:
                 left_arm.set(ControlMode.PercentOutput, 0);
                 right_arm.set(ControlMode.PercentOutput, 0);
                 break;
            }

        }
    
    public void setHighMode(boolean switchMode) {

        if (switchMode) {
            left_arm.configForwardSoftLimitThreshold(max_sensor_balls);
            right_arm.configForwardSoftLimitThreshold(max_sensor_balls);
        } else {
            left_arm.configForwardSoftLimitThreshold(max_sensor);
            right_arm.configForwardSoftLimitThreshold(max_sensor);
        }
    }

    public void findArbFB() {

        double currentSensor = left_arm.getSelectedSensorPosition();
        double currentAngle = remapSensorToDegrees(currentSensor);

        ArbFeedback = findArbFeedback(currentAngle);

    }

    public void currentLimitSwitch(boolean on) {
        left_arm.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(on,25,35,1.0));
        right_arm.configSupplyCurrentLimit(
            new SupplyCurrentLimitConfiguration(on,25,35,1.0));
        left_arm.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(on,25,35,1.0));
        right_arm.configStatorCurrentLimit(
            new StatorCurrentLimitConfiguration(on,25,35,1.0));
    }
    // conversion functions. remember line formula (x-x1)k = (y-y1)

    public void setSetpointDeg(double in) {
        setpoint = remapDegreeToSensor(in);
    }

    public void setSetpointSU(double in) {
        setpoint = in;
    }

    public double remapSensorToDegrees (double sensor) {
        double out = ((sensor - min_sensor) / ksensorToDeg) + lower_deg ;
        return out;
    }

    public double remapDegreeToSensor (double degree) {
        double out = (degree - lower_deg) * ksensorToDeg + min_sensor ;
        return out;
    }

    public double getArbOut () {
        double out = A * Math.cos(Math.toRadians(getCurrentPositionDG()));
        return out;
    }

    public double findArbFeedback(double angle) {
        double out = A * Math.cos(Math.toRadians(angle));
        return out;
    }

    public double getCurrentPositionDG() {
        return remapSensorToDegrees(left_arm.getSelectedSensorPosition());
    }
    public double getCurrentPositionSU() {
        return left_arm.getSelectedSensorPosition();
    }

    public double getAdjust(double input) {

        if (input > 0) {
            return kUpAdjustMax;
        } 
        else if (input < 0) {
            return kDnAdjustMax;
        } 
        else return 0;

    }
    

    public void updateSD() {
        // SmartDashboard Things
        Langle = remapSensorToDegrees(left_arm.getSelectedSensorPosition());
        Rangle = remapSensorToDegrees(right_arm.getSelectedSensorPosition());
        aux = ArbFeedback;
        Lcurr = left_arm.getStatorCurrent();
        Rcurr = right_arm.getStatorCurrent();
        set_out = remapSensorToDegrees(setpoint);

    }
}
