package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class shooter {

    private final int SLOT = 0;

    private WPI_TalonSRX shooter;
    // private WPI_VictorSPX bb;

    // SHOOTER SETPOINTS
    private double near = 500;
    private double far = 1300;
    private double eject = 300;


    shooter(WPI_TalonSRX shooter_in) {
        
        //WPI_VictorSPX bb_in
        shooter = shooter_in;
        // bb = bb_in;


        shooter.setInverted(false);
        shooter.setSensorPhase(false);

        // bb.setInverted(true);

        shooter.config_kP(SLOT, 0.04);
        shooter.config_kI(SLOT, 0);
        shooter.config_kD(SLOT, 0.5);
        shooter.config_kF(SLOT, 0.0205);
        shooter.configAllowableClosedloopError(SLOT, 0, 10);
        shooter.configClosedloopRamp(0.075);

        shooter.selectProfileSlot(SLOT,0);
        shooter.setSelectedSensorPosition(0);

    }


    public void init() {
        shooter.set(0);
        // bb.set(0);
    }
    

    public void shoot_per(double shooterSpeed, int shootCmd, int bbarCmd, double bbarForce) {



        switch (shootCmd) {
            case 0:
                shooter.set(ControlMode.PercentOutput, 0);
                break;

            case 1:

                shooter.set(ControlMode.Velocity, shooterSpeed);
                break;

            case 2:
                shooter.set(ControlMode.Velocity,  near);
                break;

            case 3: 
                shooter.set(ControlMode.Velocity,  far);
                break;

            case 9: // eject
                shooter.set(ControlMode.Velocity,  eject);
                break;

            case 10: // MANUAL OPEN LOOP OVERRIDE
                shooter.set(ControlMode.PercentOutput, shooterSpeed);
                break;

            default:
                shooter.set(ControlMode.PercentOutput, 0);


        }


    }

    public void determine_distance() {



    }
}
