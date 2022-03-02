package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Velocity {

    WPI_TalonSRX _leftMaster;
    WPI_TalonSRX _rightMaster;

    VictorSPX _leftFollow;
    VictorSPX _rightFollow;


	// DifferentialDrive backup = new DifferentialDrive(_leftMaster,_rightMaster);

    	/** Config Objects for motor controllers */
	TalonSRXConfiguration _leftConfig = new TalonSRXConfiguration();
	TalonSRXConfiguration _rightConfig = new TalonSRXConfiguration();
    
    CheesyDrive Cheesy = new CheesyDrive();

    public boolean _leftInvert = false; 
	public boolean _rightInvert = true;

	public boolean leftPhase = true;
	public boolean rightPhase = false;

    // These will not change if the direction changes. This is saying
	// that the talon-victor combo goes in the same direction
    public boolean _leftFollowSame = true;
    public boolean _rightFollowSame = true;



    public double MAXSPEED = 500;
	public double forwardScale = 1;
	public double turnScale = 1;


    public Velocity( 
        WPI_TalonSRX _leftMasterIn, WPI_TalonSRX _rightMasterIn,
        WPI_VictorSPX _leftFollowIn, WPI_VictorSPX _rightFollowIn,
		double maxSpeedin) {
        // left master, right master, left follow, right follow, max Speed
        MAXSPEED = maxSpeedin;

        // Bring in Motors
        _leftMaster = _leftMasterIn;
        _rightMaster = _rightMasterIn;

        _leftFollow = _leftFollowIn;
        _rightFollow = _rightFollowIn;

		
		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Coast);
		_rightMaster.setNeutralMode(NeutralMode.Coast);
        _leftFollow.setNeutralMode(NeutralMode.Coast);
        _rightFollow.setNeutralMode(NeutralMode.Coast);

		/* Configure output Inversions */
		_leftMaster.setInverted(_leftInvert);
		_rightMaster.setInverted(_rightInvert);

		_leftMaster.setSensorPhase(leftPhase);
		_rightMaster.setSensorPhase(rightPhase);


        // Look up the Ternary operator. If the polarity of the follow motor is the 
        // same as the drive motor, it will set it to the drive motor inversion. 
        // Otherwise, it will be the opposite.
        _leftFollow.setInverted(_leftFollowSame ? _leftInvert : !_leftInvert);
        _rightFollow.setInverted(_rightFollowSame ? _rightInvert : !_rightInvert);


        /* Configure the left Talon's selected sensor as integrated sensor */
		/* 
		 * Currently, in order to use a product-specific FeedbackDevice in configAll objects,
		 * you have to call toFeedbackType. This is a workaround until a product-specific
		 * FeedbackDevice is implemented for configSensorTerm
		 */
        _leftConfig.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.QuadEncoder.toFeedbackDevice();
		_rightConfig.primaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.QuadEncoder.toFeedbackDevice();

		/* Configure the Remote (Left) Talon's selected sensor as a remote sensor for the right Talon */
		// _rightConfig.remoteFilter1.remoteSensorDeviceID = _leftMaster.getDeviceID(); //Device ID of Remote Source
		// _rightConfig.remoteFilter1.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor; //Remote Source Type

		// setRobotTurnConfigs(_rightInvert, _rightConfig);

        /* Config the neutral deadband. */
		_leftConfig.neutralDeadband = Constants.kNeutralDeadband;
		_rightConfig.neutralDeadband = Constants.kNeutralDeadband;

		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftConfig.peakOutputForward = +1.0;
		_leftConfig.peakOutputReverse = -1.0;
		_rightConfig.peakOutputForward = +1.0;
		_rightConfig.peakOutputReverse = -1.0;

        		/* FPID Gains for velocity servo */
		/* FPID for Distance */
		_rightConfig.slot2.kF = Constants.kGains_Velocit.kF;
		_rightConfig.slot2.kP = Constants.kGains_Velocit.kP;
		_rightConfig.slot2.kI = Constants.kGains_Velocit.kI;
		_rightConfig.slot2.kD = Constants.kGains_Velocit.kD;
		_rightConfig.slot2.integralZone = Constants.kGains_Velocit.kIzone;
		_rightConfig.slot2.closedLoopPeakOutput = Constants.kGains_Velocit.kPeakOutput;
		_rightConfig.slot2.maxIntegralAccumulator = 12000;
		_rightConfig.slot2.allowableClosedloopError = 0;

		/* FPID for Distance */
		_leftConfig.slot2.kF = Constants.kGains_Velocit.kF;
		_leftConfig.slot2.kP = Constants.kGains_Velocit.kP;
		_leftConfig.slot2.kI = Constants.kGains_Velocit.kI;
		_leftConfig.slot2.kD = Constants.kGains_Velocit.kD;
		_leftConfig.slot2.integralZone = Constants.kGains_Velocit.kIzone;
		_leftConfig.slot2.closedLoopPeakOutput = Constants.kGains_Velocit.kPeakOutput;
		_leftConfig.slot2.maxIntegralAccumulator = 12000;
		_leftConfig.slot2.allowableClosedloopError = 0;

		/* FPID Gains for turn servo */
		/* FPID for Distance */
		_rightConfig.slot1.kF = Constants.kGains_Turning.kF;
		_rightConfig.slot1.kP = Constants.kGains_Turning.kP;
		_rightConfig.slot1.kI = Constants.kGains_Turning.kI;
		_rightConfig.slot1.kD = Constants.kGains_Turning.kD;
		_rightConfig.slot1.integralZone = Constants.kGains_Turning.kIzone;
		_rightConfig.slot1.closedLoopPeakOutput = Constants.kGains_Turning.kPeakOutput;


        /* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		_rightConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		_leftConfig.slot3.closedLoopPeriod = closedLoopTimeMs;
		
		/* APPLY the config settings */
		_leftMaster.configAllSettings(_leftConfig);
		_rightMaster.configAllSettings(_rightConfig);

		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		

		/* Initialize */
		zeroSensors();

		// Re-initalize the loops
		_rightMaster.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
		// _rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);

		_leftMaster.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);


    }



    public void vel_initalize() {

        zeroSensors();

		_leftMaster.set(ControlMode.PercentOutput,0);
		_rightMaster.set(ControlMode.PercentOutput,0);
		_rightFollow.set(ControlMode.PercentOutput, 0);
		_leftFollow.set(ControlMode.PercentOutput, 0);



    }

    public void velPeriodic(double speed, double turn, boolean isCheesy, boolean velctl, boolean isQuickTurn) {

		Cheesy.cheesyDrive(speed, turn, isQuickTurn);

		double cheesyLeft = Cheesy.leftSpeed;
		double cheesyRight = Cheesy.rightSpeed;

		double cheesyLeftVel = cheesyLeft*MAXSPEED;
		double cheesyRightVel = cheesyRight*MAXSPEED;

		if (isCheesy && velctl) {
			_rightMaster.set(ControlMode.Velocity, cheesyRightVel); //, DemandType.AuxPID, turn
			_leftMaster.set(ControlMode.Velocity, cheesyLeftVel);
		} 
		else if (isCheesy) {
			_rightMaster.set(ControlMode.PercentOutput, cheesyRight);
			_leftMaster.set(ControlMode.PercentOutput, cheesyLeft);

		}
		else {
			//open
		}
		_rightFollow.follow(_rightMaster, FollowerType.PercentOutput);
		_leftFollow.follow(_leftMaster, FollowerType.PercentOutput);
    }

    public void zeroSensors () {
        
        _leftMaster.setSelectedSensorPosition(0);
        _rightMaster.setSelectedSensorPosition(0);

		System.out.println("[Integrated Sensors] All sensors are zeroed.\n");
    }
    


    // From CTRE Example code. Credit to CTRE.

    /** 
	 * Determines if SensorSum or SensorDiff should be used 
	 * for combining left/right sensors into Robot Distance.  
	 * 
	 * Assumes Aux Position is set as Remote Sensor 0.  
	 * 
	 * configAllSettings must still be called on the master config
	 * after this function modifies the config values. 
	 * 
	 * @param _rightInvert2 Invert of the Master Talon
	 * @param masterConfig Configuration object to fill
	 */
	void setRobotTurnConfigs(boolean _rightInvert2, TalonSRXConfiguration masterConfig){
		/**
		 * Determine if we need a Sum or Difference.
		 * 
		 * The auxiliary Talon FX will always be positive
		 * in the forward direction because it's a selected sensor
		 * over the CAN bus.
		 * 
		 * The master's native integrated sensor may not always be positive when forward because
		 * sensor phase is only applied to *Selected Sensors*, not native
		 * sensor sources.  And we need the native to be combined with the 
		 * aux (other side's) distance into a single robot heading.
		 */

		/* THIS FUNCTION should not need to be modified. 
		   This setup will work regardless of whether the master
		   is on the Right or Left side since it only deals with
		   heading magnitude.  */

		/* Check if we're inverted */
		if (_rightInvert2 == true){
			/* 
				If master is inverted, that means the integrated sensor
				will be negative in the forward direction.

				If master is inverted, the final sum/diff result will also be inverted.
				This is how Talon FX corrects the sensor phase when inverting 
				the motor direction.  This inversion applies to the *Selected Sensor*,
				not the native value.

				Will a sensor sum or difference give us a positive heading?

				Remember the Master is one side of your drivetrain distance and 
				Auxiliary is the other side's distance.

					Phase | Term 0   |   Term 1  | Result
				Sum:  -((-)Master + (+)Aux   )| OK - magnitude will cancel each other out
				Diff: -((-)Master - (+)Aux   )| NOT OK - magnitude increases with forward distance.
				Diff: -((+)Aux    - (-)Master)| NOT OK - magnitude decreases with forward distance
			*/

			masterConfig.sum0Term = TalonSRXFeedbackDevice.None.toFeedbackDevice(); //Local Integrated Sensor
			masterConfig.sum1Term = TalonSRXFeedbackDevice.RemoteSensor1.toFeedbackDevice();   //Aux Selected Sensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.SensorSum.toFeedbackDevice(); //Sum0 + Sum1

			/*
				PID Polarity

				With the sensor phasing taken care of, we now need to determine if the PID polarity is in the correct direction
				This is important because if the PID polarity is incorrect, we will run away while trying to correct

				Will inverting the polarity give us a positive counterclockwise heading?

				If we're moving counterclockwise(+), and the master is on the right side and inverted,
				it will have a negative velocity and the auxiliary will have a negative velocity
				 heading = right + left
				 heading = (-) + (-)
				 heading = (-)
				Let's assume a setpoint of 0 heading.
				This produces a positive error, in order to cancel the error, the right master needs to
				drive backwards. This means the PID polarity needs to be inverted to handle this
				
				Conversely, if we're moving counterclwise(+), and the master is on the left side and inverted,
				it will have a positive velocity and the auxiliary will have a positive velocity.
				 heading = right + left
				 heading = (+) + (+)
				 heading = (+)
				Let's assume a setpoint of 0 heading.
				This produces a negative error, in order to cancel the error, the left master needs to
				drive forwards. This means the PID polarity needs to be inverted to handle this
			*/
			masterConfig.auxPIDPolarity = true;
		} else {
			/* Master is not inverted, both sides are positive so we can diff them. */
			masterConfig.diff0Term = TalonSRXFeedbackDevice.RemoteSensor1.toFeedbackDevice();    //Aux Selected Sensor
			masterConfig.diff1Term = TalonSRXFeedbackDevice.None.toFeedbackDevice(); //Local IntegratedSensor
			masterConfig.auxiliaryPID.selectedFeedbackSensor = TalonSRXFeedbackDevice.SensorDifference.toFeedbackDevice(); //Sum0 + Sum1
			/* With current diff terms, a counterclockwise rotation results in negative heading with a right master */
			masterConfig.auxPIDPolarity = true;
		}
		/**
		 * Heading units should be scaled to ~4000 per 360 deg, due to the following limitations...
		 * - Target param for aux PID1 is 18bits with a range of [-131072,+131072] units.
		 * - Target for aux PID1 in motion profile is 14bits with a range of [-8192,+8192] units.
		 *  ... so at 3600 units per 360', that ensures 0.1 degree precision in firmware closed-loop
		 *  and motion profile trajectory points can range +-2 rotations.
		 */
		masterConfig.auxiliaryPID.selectedFeedbackCoefficient = Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation;
	}
}
