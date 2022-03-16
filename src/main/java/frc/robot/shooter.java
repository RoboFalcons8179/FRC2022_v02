package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class shooter {



    private WPI_TalonSRX shooter;
    private WPI_VictorSPX bb;

    // SHOOTER SETPOINTS
    private double near = 500;


    shooter(WPI_TalonSRX shooter_in, WPI_VictorSPX bb_in) {
        shooter = shooter_in;
        bb = bb_in;


        shooter.setInverted(false);
        shooter.setSensorPhase(false);

        bb.setInverted(true);


    }


    public void init() {
        shooter.set(0);
        bb.set(0);
    }
    

    public void shoot_per(int command, double set) {

        switch (command) {

            case 1:

                shooter.set(ControlMode.Velocity, set);
                break;

            case 2:
                shooter.set(ControlMode.Velocity,  near);
                break;
            
            case 9: // eject
                shooter.set(ControlMode.Velocity,  near);
                break;

            case 10: // MANUAL OPEN LOOP OVERRIDE
                shooter.set(ControlMode.PercentOutput, set);
            
            default:
                shooter.set(ControlMode.PercentOutput, 0);


        }


    }

    public void determine_distance() {

    }
}
