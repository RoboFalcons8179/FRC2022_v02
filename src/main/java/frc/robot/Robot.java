// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.music.Orchestra;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Timer;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding too
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
 	private static PowerDistribution PD = new PowerDistribution();

	// SAFETY
	private static final double MAX_SPEED = 550;
	private static final boolean safety = false;

	// PTHER IMPORTANT THINGS
	private static commands c = new commands();

	// DRIVE MOTORS
	private static WPI_TalonSRX rightDrive = new WPI_TalonSRX(1);
	private static WPI_TalonSRX leftDrive = new WPI_TalonSRX(3);
	
	private static WPI_VictorSPX rightFollow = new WPI_VictorSPX(2);
	private static WPI_VictorSPX leftFollow = new WPI_VictorSPX(4);

	// SHARKFIN MOTORS
	private static WPI_TalonFX right_shark = new WPI_TalonFX(6);
	private static WPI_TalonFX left_shark = new WPI_TalonFX(5);

	// ARM MOTORS
	private static WPI_TalonFX right_chop = new WPI_TalonFX(7);
	private static WPI_TalonFX left_chop = new WPI_TalonFX(8);

	// SHOOTER MOTORS
	private static WPI_TalonSRX shooterMotor = new WPI_TalonSRX(9);


	// JOYSTICKS
	private static Joystick xbox_0 = new Joystick(0); // DRIVER
	private static Joystick gamepad0 = new Joystick(1); // multi-button fun
	private static Joystick gamepad1 = new Joystick(2); // multi-button fun
	private static Buttons b = new Buttons();

	// Timer
	Timer timer = new Timer();


	// MOTION SYSTEMS
	private static Velocity vroom = new Velocity(leftDrive, rightDrive, leftFollow, rightFollow, MAX_SPEED, safety);
	private static Sharkfin fin = new Sharkfin(right_shark, left_shark);
	private static arm chop = new arm(left_chop, right_chop);
	private static shooter pew = new shooter(shooterMotor);

	// SMART DASHBOARD
	private ShuffleboardTab data = Shuffleboard.getTab("DATA");
	private NetworkTableEntry setSpeedNetwork = data.add("SET SPEED",0).getEntry();
	private NetworkTableEntry setTurnNetwork = data.add("SET TURN",0).getEntry();
	private NetworkTableEntry setFinsNetwork = data.add("SET FIN POSITION",0).getEntry();
	private NetworkTableEntry setArmCurrentSW = data.add("SET ARM CURRENT LIMS",true).getEntry();
	private NetworkTableEntry setShooterSpeed = data.add("SETSHOOTER SPEED",0).getEntry();

	// LIMELIGHT

	NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
	NetworkTableEntry tx = table.getEntry("tx");

	double limelighttx;

	
	@Override
  	public void robotInit() {

		CameraServer.startAutomaticCapture();


  }

  
  @Override
  public void robotPeriodic() {

	chop.findArbFB();
	chop.updateSD();


	updateSB_Periodic();

	// read values periodically
	limelighttx = tx.getDouble(0.0);

  }


  @Override
  public void autonomousInit() {

	timer.reset();
	timer.start();

	vroom.vel_initalize();
	fin.shark_initial();
	chop.arm_init();
	pew.init();

  }


  // Global variables
	// double speed;
	// double rot;
	// boolean vel_ctl = true;

	// // double fin_set;
	// // int fin_status;

	// double current_arm_set = 0;
	// int arm_cmd = 0;
	// double arm_lock_pos = 0;
	// boolean arm_sticky = true;

	// boolean home_arm = false;
	// boolean arm_current_limit = true;

	statecommand lastCommand = new statecommand();

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


		statecommand thisround = new statecommand();

		thisround.lime_tx = limelighttx;

		thisround.armset = 0;
		//auton 1
/*		
		if (aTS(0, 3.5)) {
			thisround.speed = -0.7;
		}
		if (aTS(3.5, 5)) {
			thisround.shootCmd = c.SHOOT_NEAR;
		}
		if (aTS(5, 7)) {
			thisround.shootCmd = c.SHOOT_NEAR;
			
			thisround.fin_status = c.FIN_SCOOP;
		}
		*/
		//auton 2

		if (aTS(0, 3.5)) {
			thisround.speed = -0.7;
		}
		if (aTS(3.5, 5)) {
			thisround.shootCmd = c.SHOOT_NEAR;
		}
		if (aTS(5, 7)) {
			thisround.shootCmd = c.SHOOT_NEAR;
			
			thisround.fin_status = c.FIN_SCOOP;
		}




		thisround.assignCmd(vroom, fin, chop, pew);
		lastCommand = thisround;

		// vroom.velPeriodic(speed, 0, true, false, false, false, 0, 0, -1);

		// fin.sharkPeriodic(fin_set, fin_status);

		// chop.arm_Periodic(armset, arm_cmd, arm_sticky);
	}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

	vroom.vel_initalize();
	fin.shark_initial();
	chop.arm_init();

  }



  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
	  
////////////// Deaults

	statecommand thisRound = new statecommand();

	boolean vel_ctl = lastCommand.velctl;

	thisRound.fin_status = c.FIN_FAST;

	boolean fin_set_point = false;

	boolean inShootMode = gamepad1.getRawButton(1);


//// Smart Dashboard
	// speed = setSpeedNetwork.getDouble(1.0);
	// rot = setTurnNetwork.getDouble(1.0);
	// fin_set = setFinsNetwork.getDouble(1.0);
	// chop.currentLimitSwitch(setArmCurrentxxSW.getBoolean(true));

//////////
	
///// VELOCITY
	thisRound.speed = deadband(xbox_0.getRawAxis(1) * -1);
	thisRound.turn = xbox_0.getRawAxis(4);
	
	// switching between cheesy open and vel ctl
	thisRound.velctl = xbox_0.getRawButtonPressed(b.xbox_a) ? !vel_ctl : vel_ctl;

	// pivot & others
	thisRound.RL = xbox_0.getRawButton(5); 
	thisRound.RR = xbox_0.getRawButton(6);
	thisRound.isQuickTurn = xbox_0.getRawButtonPressed(b.xbox_y);
	thisRound.pov = xbox_0.getPOV();

	// limelight
	thisRound.lime_enable = xbox_0.getRawButton(b.xbox_x);
	thisRound.lime_tx = limelighttx;

	// Turbo
	thisRound.maxPowerF = inShootMode ? xbox_0.getRawAxis(2) : 0;
	// double MaxPowerRev = inShootMode ? xbox_0.getRawAxis(3) * 0.6 : 0;

///// FINS
	// Buttons
	// boolean fin_adj_up = (xbox_1.getPOV() == 0);
	// boolean fin_adj_down = (xbox_1.getPOV() == 180);
	boolean fin_pull = (gamepad0.getRawButton(b.FIS));
	boolean fin_push = (gamepad0.getRawButton(b.FOS));
	boolean fin_scoop = gamepad0.getRawButton(b.scoop);
	boolean fin_extend = gamepad0.getRawButton(b.finsExtend);

	if (gamepad0.getRawButton(b.FOF)) {
		fin_set_point = true;
		thisRound.fin_set = -0.97;
	}
	if (gamepad0.getRawButton(b.FIF)) {
		fin_set_point = true;
		thisRound.fin_set = 1;
	}



///////////////////// Turning booleans into system commands

	if(!inShootMode) {
		if (fin_set_point) {
			thisRound.fin_status = c.FIN_FAST;
		} else 
		if (fin_pull){
			thisRound.fin_status = c.FIN_PULLSLOW;
		} else 
		if (fin_push){
			thisRound.fin_status = c.FIN_PUSHSLOW;
		}else
		if (fin_scoop) {
			thisRound.fin_status = c.FIN_SCOOP;
		}else
		if (fin_extend) {
			thisRound.fin_status = c.FIN_EXTEND;
		} else 
		{
			thisRound.fin_status = c.FIN_HOLD;
		}
	}
	else {

		if(gamepad0.getRawButton(b.scoop)) {
			thisRound.fin_status = c.FIN_SCOOP;
		} else {
			thisRound.fin_status = c.FIN_EXTEND;
		}

	}


	/////// ARM //////////

	if (gamepad0.getRawButton(b.MU)) { 
		// up
		
		thisRound.arm_cmd = c.ARM_ADJUP;
		thisRound.armset = 0;
		thisRound.arm_sticky = true;

	} else 
	if (gamepad0.getRawButton(b.MD)){ 
		// down

		thisRound.arm_cmd = c.ARM_ADJDOWN;
		thisRound.armset = 0;
		thisRound.arm_sticky = true;

	} else	
	if (xbox_0.getRawAxis(2) > 0.2 && ! inShootMode){
		// High mode Swing Down
		thisRound.armset = xbox_0.getRawAxis(2) * -1;
		thisRound.arm_cmd = c.ARM_OPENLOOP;
		thisRound.arm_sticky = true;


	} else
	if (xbox_0.getRawAxis(3) > 0.2 && ! inShootMode) {
		// High mode swing up
		
		thisRound.armset = xbox_0.getRawAxis(3);
		thisRound.arm_cmd = c.ARM_OPENLOOP;
		thisRound.arm_sticky = true;
	} else
	if (gamepad0.getRawButton(b.openArm)) { 
		// open arms for ball
		thisRound.armset = 18000;
		thisRound.arm_cmd = c.ARM_POSITION;
		thisRound.arm_sticky = false;
	} else

	/// Removing score arms - not used any more.
	// if (gamepad0.getRawButton(b.scoreArms)) {
	// 	// raise up to score
	// 	thisRound.armset = 60000;
	// 	thisRound.arm_cmd = c.ARM_POSITION;
	// 	thisRound.arm_sticky = false;
	// } else
	// if (gamepad0.getRawButton(b.humanScore)) {
	// 	// for human to roll balls
	// 	thisRound.armset = 46000;
	// 	thisRound.arm_cmd = c.ARM_POSITION;
	// 	thisRound.arm_sticky = false;
	// }else
	 { // Default Hold, command 1 or 10
		
		thisRound.arm_cmd = c.ARM_FLOATPOS;
		thisRound.arm_sticky = false;
	} 




	// SHOOTER

		// thisRound.shootSpeed = setShooterSpeed.getDouble(0.0);

		if (gamepad0.getRawButton(b.pew_near)) {
			thisRound.shootCmd = c.SHOOT_NEAR;
		} else 
		if (gamepad0.getRawButton(b.pew_far)) {
			thisRound.shootCmd = c.SHOOT_FAR;
		} else
		if (gamepad0.getRawButton(b.pew_ej)) {
			thisRound.shootCmd = c.SHOOT_EJECT;
		}
		else {
			thisRound.shootCmd = 0;
		}
			


	
	// Switching between the shooting mode and the hanging mode form
	// swtich on our control board	

	chop.setHighMode(inShootMode);
	fin.setHighMode(inShootMode);

	///////// ASSIGNING FUNCTONS
	
	if (lastCommand != thisRound) {

		thisRound.assignCmd(vroom, fin, chop, pew);

	}
		xbox_0.setRumble(RumbleType.kLeftRumble, rmbleDBremap(leftDrive.getSupplyCurrent() / 30));
		xbox_0.setRumble(RumbleType.kRightRumble, rmbleDBremap(rightDrive.getSupplyCurrent() / 30));


	lastCommand = thisRound;





	// vroom.velPeriodic(speed, rot, vel_ctl, handBrake, pivotL, pivotR, MaxPowerFwd, MaxPowerRev, vel_pov);
	// // Velocity Drive args in order:
	// 	// speed in range [-1,1]
	// 	// rotate in range [-1,1]
	// 	// bool is cheezy: are you using cheese drive (yes)
	// 	// bool velctl: using velocity control or open loop.
	// 	// bool cheezy sharp: in cheezy, this will start a sharp turn.

	// 	// Helper output vroom functions:
	// 	xbox_0.setRumble(RumbleType.kLeftRumble, rmbleDBremap(leftDrive.getSupplyCurrent() / 30));
	// 	xbox_0.setRumble(RumbleType.kRightRumble, rmbleDBremap(rightDrive.getSupplyCurrent() / 30));

	// fin.sharkPeriodic(fin_set, fin_status); // fin_set is range [-1,1]
	// // Fin args in order:
	// 	// Setpoint in range [-1, 1]
	// 	// Statuses:
	// 	// default: Turn off motors
	// 	// 1: Hold
	// 	// 2: Move to specific point
	// 	// 3: Move fins up
	// 	// 4: move fins down

	// chop.arm_Periodic(armset, arm_cmd, arm_sticky);
	// Arm args in order:
		// Setpoint in SENSOR Units, 
		// States
		// Bool sticky: only for position, 
		// will leave taht as teh setpoint. do not use for Open loop.



  }




/** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
	  chop.setSetpointSU(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */


  @Override
  public void testInit() {



  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

	
  }

  private double deadband(double d) {
	if (-0.05 < d && d < 0.05)
		return 0;
	else
		return d;
	}

	private double rmbleDBremap(double input) {
		if (-0.20 < input && input < 0.20)
			return 0;
		else
			return input * input;
	
	}

	private boolean aTS(double startTimeS, double endTimeS) {
		if (startTimeS < timer.get() && timer.get() < endTimeS) {
			return true;
		} else {
			return false;
		}
	}

	private void updateSB_Periodic() {
		SmartDashboard.putNumber("Forward Speed", lastCommand.speed);
		SmartDashboard.putNumber("Turn Command", lastCommand.turn);
		SmartDashboard.putNumber("Velocity Left", leftDrive.getSelectedSensorVelocity(1));
		SmartDashboard.putNumber("Velocity Right", rightDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Current Left", leftDrive.getStatorCurrent());
		SmartDashboard.putNumber("Current Right", rightDrive.getStatorCurrent());
		SmartDashboard.putBoolean("Vel Ctl Mode", lastCommand.velctl);

		SmartDashboard.putNumber("Drive Speed Error", leftDrive.getSelectedSensorVelocity(1)+rightDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putBoolean("Limit mode", gamepad1.getRawButton(1));
		SmartDashboard.putBoolean("Lime Enble", xbox_0.getRawButton(b.xbox_x));
		SmartDashboard.putNumber("Lime CMD", vroom.findLimeTurn(limelighttx));

		
		SmartDashboard.putNumber("right shark position", right_shark.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("left shark position", left_shark.getSelectedSensorPosition(0));
		SmartDashboard.putNumber("right shark current", right_shark.getStatorCurrent());
		SmartDashboard.putNumber("left shark current", left_shark.getStatorCurrent());


		SmartDashboard.putNumber("Shark CMD", lastCommand.fin_status);

		SmartDashboard.putBoolean("fin left rev limit SW", left_shark.isRevLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("fin rght rev limit SW", right_shark.isRevLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("fin left for limit SW", left_shark.isFwdLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("fin rght for limit SW", right_shark.isFwdLimitSwitchClosed() == 0);
		SmartDashboard.putNumber("fin L Out", left_shark.getMotorOutputPercent());
		SmartDashboard.putNumber("fin R Out", right_shark.getMotorOutputPercent());

		SmartDashboard.putBoolean("arm left rev limit SW", left_chop.isRevLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("arm rght rev limit SW", right_chop.isRevLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("arm left fwd limit SW", left_chop.isFwdLimitSwitchClosed() == 0);
		SmartDashboard.putBoolean("arm rght fwd limit SW", right_chop.isFwdLimitSwitchClosed() == 0);

		SmartDashboard.putNumber("Arm Set Angle", chop.set_out);
		SmartDashboard.putNumber("Arm Grav Correction", chop.aux);
		SmartDashboard.putNumber("Arm L Out", left_chop.getMotorOutputPercent());
		SmartDashboard.putNumber("Arm R Out", right_chop.getMotorOutputPercent());
		SmartDashboard.putNumber("Arm Current L Angle", chop.Langle);
		SmartDashboard.putNumber("Arm Current R Angle", chop.Rangle);
		SmartDashboard.putNumber("Arm Current Angle SU", chop.getCurrentPositionSU());

		SmartDashboard.putNumber("Arm Command", lastCommand.arm_cmd);
		SmartDashboard.putNumber("Arm Status", chop.status);		
		SmartDashboard.putNumber("Arm L Current", chop.Lcurr);
		SmartDashboard.putNumber("Arm R Current", chop.Lcurr);
		SmartDashboard.putNumber("Grav Comp", chop.getArbOut());

		SmartDashboard.putNumber("Arm R Temp", left_chop.getTemperature());
		SmartDashboard.putNumber("Arm L Temp", right_chop.getTemperature());


	}
}




// if(aTS(0, 3)) {
// 	thisround.speed = 0;

// 	thisround.fin_set = 0;
// 	thisround.fin_status = c.FIN_HOLD;
	
// 	thisround.armset = chop.remapDegreeToSensor(0);
// 	thisround.arm_cmd = c.ARM_POSITION;
// 	thisround.arm_sticky = false;


// } else
// if (aTS(3, 6.5)) {

// 	thisround.speed = 0.7;

// 	thisround.fin_set = 0;
// 	thisround.fin_status = c.FIN_HOLD;
	
// 	thisround.armset = chop.remapDegreeToSensor(0);
// 	thisround.arm_cmd = c.ARM_POSITION;
// 	thisround.arm_sticky = false;

// } else 
// if (aTS(6.5, 10)) {

// 	thisround.speed = 00;

// 	thisround.fin_set = .89;
// 	thisround.fin_status = c.FIN_FAST;
	
// 	thisround.armset = chop.remapDegreeToSensor(0);
// 	thisround.arm_cmd = c.ARM_POSITION;
// 	thisround.arm_sticky = false;

// } else
// if (aTS(10, 11.5)) {
// 	thisround.speed = -0.8;

// 	thisround.fin_set = 0;
// 	thisround.fin_status = c.FIN_FAST;
	
// 	thisround.armset = chop.remapDegreeToSensor(0);
// 	thisround.arm_cmd = c.ARM_POSITION;
// 	thisround.arm_sticky = false;


// } else
// if (aTS(11.5, 15)) {
// 	thisround.speed = -0.8;

// 	thisround.fin_set = 0;
// 	thisround.fin_status = c.FIN_FAST;
	
// 	thisround.armset = -9000;
// 	thisround.arm_cmd = c.ARM_POSITION;
// 	thisround.arm_sticky = false;

// } else
// {
// 	// keep default psoitions
// } 