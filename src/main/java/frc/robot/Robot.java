// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
 	private static PowerDistribution examplePD = new PowerDistribution();

	// SAFETY

	private static final double MAX_SPEED = 500;

	// DRIVE MOTORS
	private static WPI_TalonSRX rightDrive = new WPI_TalonSRX(1);
	private static WPI_TalonSRX leftDrive = new WPI_TalonSRX(3);
	
	private static WPI_VictorSPX rightFollow = new WPI_VictorSPX(2);
	private static WPI_VictorSPX leftFollow = new WPI_VictorSPX(4);

	// private static WPI_TalonFX right_shark = new WPI_TalonFX(6);
	// private static WPI_TalonFX left_shark = new WPI_TalonFX(5);


	// JOYSTICKS
	private static Joystick xbox_0 = new Joystick(0);

	// MOTION SYSTEMS
	private static Velocity vroom = new Velocity(leftDrive, rightDrive, leftFollow, rightFollow, MAX_SPEED);
	// private static Sharkfin fin = new Sharkfin(right_shark, left_shark);


	// SMART DASHBOARD
	private ShuffleboardTab data = Shuffleboard.getTab("DATA");
	private NetworkTableEntry setSpeedNetwork = data.add("SET SPEED",0).getEntry();
	private NetworkTableEntry setTurnNetwork = data.add("SET TURN",0).getEntry();
	private NetworkTableEntry setFinsNetwork = data.add("SET FIN POSITION",0).getEntry();

	@Override
  	public void robotInit() {
	////// Set drive motor phases and inversion //////////


	// Shuffleboard


  }


  @Override
  public void robotPeriodic() {

	updateSB_Periodic();



  }


  @Override
  public void autonomousInit() {


  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {


  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

	vroom.vel_initalize();
	// fin.shark_initial();



  }
// Global variables
double speed;
double rot;
double fin_set;


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {


	speed = deadband(xbox_0.getRawAxis(1) * -1);
	rot = deadband(xbox_0.getRawAxis(4));

	// speed = setSpeedNetwork.getDouble(1.0);
	// rot = setTurnNetwork.getDouble(1.0);

	// fin_set = setFinsNetwork.getDouble(1.0)
	
	fin_set = xbox_0.getRawAxis(1)*-1*.8;


	vroom.velPeriodic(speed, rot, true);
	// fin.sharkPeriodic(fin_set, true); // fin_set is range [-1,1]



  }




/** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  private double deadband(double d) {
	if (-0.20 < d && d < 0.20)
		return 0;
	else
		return d;
	}

	private void updateSB_Periodic() {
		SmartDashboard.putNumber("Forward Speed", speed);
		SmartDashboard.putNumber("Turn Command", rot);
		SmartDashboard.putNumber("Velocity Left", leftDrive.getSelectedSensorVelocity(1));
		SmartDashboard.putNumber("Velocity Right", rightDrive.getSelectedSensorVelocity(0));
		SmartDashboard.putNumber("Current Left", leftDrive.getStatorCurrent());
		SmartDashboard.putNumber("Current Right", rightDrive.getStatorCurrent());
	
		SmartDashboard.putNumber("Drive Speed Error", leftDrive.getSelectedSensorVelocity(1)+rightDrive.getSelectedSensorVelocity(0));
		
		
		// SmartDashboard.putNumber("right shark position", right_shark.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("left shark position", left_shark.getSelectedSensorPosition(0));
		// SmartDashboard.putNumber("right shark current", right_shark.getStatorCurrent());
		// SmartDashboard.putNumber("left shark current", left_shark.getStatorCurrent());

		// speed = SmartDashboard.getNumber("speed network command", 0);
		// rot = SmartDashboard.getNumber("turn network command", 0);
	}
}
