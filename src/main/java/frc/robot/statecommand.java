package frc.robot;

// Class used for gathering and assigning commands to the different motion systems.

public class statecommand {

    private commands c = new commands();


    // Velocity commands
    public double speed;
    public double turn;

    public boolean velctl;
    public boolean isQuickTurn;
    public boolean RL;
    public boolean RR;
    public double  maxPowerF;
    public double  maxPowerR;
    public double pov;
     



    // fins
    public double fin_set;
    public int fin_status;
    
    // Arms
    public double armset;
    public int arm_cmd;
    public boolean arm_sticky;

    // Shooter System
    public double shootSpeed;
    public int shootCmd;
    public int bb_cmd;
    public double bb_options;

    statecommand() {

        // Default commands

        speed = 0;
        turn = 0;

        velctl = true;
        isQuickTurn = false;
        RL = false;
        RR = false;
        maxPowerF = 0;
        maxPowerR = 0;
        pov = -1;

        fin_set = 0;
        fin_status = c.FIN_HOLD;

        armset = 0;
        arm_cmd = c.ARM_FLOATPOS;
        arm_sticky = true;

        shootSpeed = 0;
        shootCmd = 0;
        bb_cmd = 0;
        bb_options = 0;
    }

    public void assignCmd (
        Velocity vroom,
        Sharkfin fins,
        arm arms,
        shooter bang
    ) {
       
        vroom.velPeriodic(speed, turn, velctl, isQuickTurn, RL, RR, maxPowerF, maxPowerR, pov);
        fins.sharkPeriodic(fin_set, fin_status);
        arms.arm_Periodic(armset, arm_cmd, arm_sticky);
        bang.shoot_per(shootSpeed, shootCmd, bb_cmd, bb_options);

    }


    


    
}
