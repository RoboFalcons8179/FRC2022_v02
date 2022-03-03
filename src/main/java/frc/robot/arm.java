package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class arm {

    private WPI_TalonFX left,right;

    public TalonFXInvertType right_invert = TalonFXInvertType.CounterClockwise;
    public TalonFXInvertType left_invert = TalonFXInvertType.Clockwise;


    // CONSTANTS

    // PWM signals that will let the arm go up.
    // Does not include the ARB Value
    private final double kUpAdjustMax = 0.5;
    private final double kDnAdjustMax = -0.1;

    // PWM for lowering to home.
    private final double homePWM = -0.1;

    // constants for the arm position
    private final double lower_deg = -60.0;
    private final double upper_deg = 11.;
    private final double max_sensor = 200000;
    private final double min_sensor = 0;

        // Calculating A, can find empirically
        //rather than physics-ly
    private final double maxTorque = 30;
    private final double stallTorque = 4.6;
    private final double gearRatio = 6.4*3*9;

    // constants for the loop
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double iz = 0;
    private double mmVel = 0;
    private double mmAcc = 0;

    private int slotID = 0;


    // Calculated constants
    private final double ksensorToDeg;
    private final double A;
    private final int masterID;


    // The position setpoint, can only be accessed by the functions.
    private static double setpoint;


    arm(WPI_TalonFX left_in, WPI_TalonFX right_in) {

        left = left_in;
        right = right_in;

        masterID = left.getDeviceID();

        // calculating the constants
        ksensorToDeg = (max_sensor - min_sensor) / (upper_deg - lower_deg);
        A = gearRatio * maxTorque / stallTorque;

        setpoint = 0;

        left.configFactoryDefault();
        right.configFactoryDefault();

        right.setInverted(right_invert); // false
        left.setInverted(left_invert); // true

        right.setSensorPhase(false);
        left.setSensorPhase(false);

        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);

        left.config_kP(slotID, kp);
        left.config_kI(slotID, ki);
        left.config_kD(slotID, kd);
        left.config_IntegralZone(slotID, iz);
        left.configMotionCruiseVelocity(mmVel);
        left.configMotionCruiseVelocity(mmAcc);

        left.selectProfileSlot(slotID, 0); // Primary loop

    }

    public void arm_init() {

        left.set(ControlMode.PercentOutput, 0);
        right.set(ControlMode.PercentOutput, 0);

        right.set(ControlMode.Follower, masterID);

        setSetpointSU(left.getSelectedSensorPosition());
    }

    public void arm_Periodic(double setpoint, double move, boolean mode, boolean home) {
        // if mode is true, we are going to be using position control
        // if home is true, this is going to home.
       
        double currentSensor = left.getSelectedSensorPosition();
        double currentAngle = remapSensorToDegrees(currentSensor);

        double ArbFeedback = findArbFeedback(currentAngle);

        if (home) {
            // Homing the arms
            left.set(ControlMode.PercentOutput, homePWM);
            right.set(ControlMode.Follower, left.getDeviceID());
        } else {
            if (mode) {
                // Using position control
                left.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, ArbFeedback);
                right.set(ControlMode.Follower, left.getDeviceID());

            } else {
                // using PWM adjustments.
                double adjust = getAdjust(move);

                left.set(ControlMode.PercentOutput, adjust, DemandType.ArbitraryFeedForward, ArbFeedback);
                right.set(ControlMode.Follower, left.getDeviceID());

                setSetpointSU(left.getSelectedSensorPosition());
            }
        }
    }

    // conversion functions. remember line formula (x-x1)k = (y-y1)

    public void setSetpointDeg(double in) {
        setpoint = remapDegreeToSensor(in);
    }

    public void setSetpointSU(double in) {
        setpoint = in;
    }

    public double remapSensorToDegrees (double sensor) {
        double out = (sensor - min_sensor / ksensorToDeg) + lower_deg ;
        return out;
    }

    public double remapDegreeToSensor (double degree) {
        double out = (degree - lower_deg) * ksensorToDeg + min_sensor ;
        return out;
    }

    public double findArbFeedback(double angle) {
        double out = A * Math.cos(Math.toRadians(angle));
        return out;
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
    
}
