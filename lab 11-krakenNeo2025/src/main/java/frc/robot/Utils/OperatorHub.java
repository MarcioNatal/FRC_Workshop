package frc.robot.Utils;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.OIConstants.MesinhaJoy1Constants;
import frc.robot.Constants.OIConstants.MesinhaJoy2Constants;


public class OperatorHub {
    private final Joystick mesinha1;
    private final Joystick mesinha2;

/**************************************************************
 *                         Mesinha1                           *
 **************************************************************/
    //Botões mesinha1

    public int kOne = MesinhaJoy1Constants.kNine;
    public int kTwo = MesinhaJoy1Constants.kTen;
    public int kThree = MesinhaJoy1Constants.kTwelve;
    public int kFour = MesinhaJoy1Constants.kEleven;
    public int kEight = MesinhaJoy1Constants.kOne;

    public int kSelectOne = MesinhaJoy1Constants.kSix;
    public int kSelectTwo = MesinhaJoy1Constants.kEight;
    public int kSelectThree = MesinhaJoy1Constants.kSeven;

    public int kBlackAxisUp = MesinhaJoy1Constants.kBlackAxisUp;

    public int kBlackAxisSides = MesinhaJoy1Constants.kBlackAxisSides;


/**************************************************************
 *                         Mesinha2                           *
 **************************************************************/
    //Botões mesinha2

    //Positions

    public int kFive = MesinhaJoy2Constants.kThree;
    public int kSix = MesinhaJoy2Constants.kTwo;
    public int kSeven = MesinhaJoy2Constants.kOne;
    public int kNine = MesinhaJoy2Constants.kSix;

    public int kTen = MesinhaJoy2Constants.kSeven;
    public int kEleven = MesinhaJoy2Constants.kEight;


    public int kRedAxisUp = MesinhaJoy2Constants.kRedAxisUp;
    public int kRedAxisSides = MesinhaJoy2Constants.kRedAxisSides;

    public OperatorHub(Joystick mesinha1, Joystick mesinha2){
        this.mesinha1 = mesinha1;
        this.mesinha2 = mesinha2;
    }


    ///////////////////////////////////////////////////////////////////////////////////////
    
    //Mesinha1
    public boolean getBlackAxisUp(double percent)
    {
        return mesinha1.getRawAxis(kBlackAxisUp) > percent;
    }

    public boolean getBlackAxisRight(double percent)
    {
        return mesinha1.getRawAxis(1) > percent;
    }

    public boolean getBlackAxisLeft(double percent)
    {
        return mesinha1.getRawAxis(1) < percent;
    }

    public boolean getBlackAxisDown(double percent)
    {
        return mesinha1.getRawAxis(kBlackAxisUp) < percent;
    }

    //Mesinha2


    public boolean getRedAxisUp(double percent)
    {
        if ( mesinha2.getRawAxis(kBlackAxisUp) >percent)
        {
            return true;
        }
        
       return false;
    }

    public boolean getRedAxisDown(double percent)
    {
        if ( mesinha2.getRawAxis(kBlackAxisUp) < percent)
        {
            return true;
        }
        
       return false;
    }
    public boolean getRedAxisUpDownHome( )
    {
        if ( mesinha2.getRawAxis(kBlackAxisUp) < 0.3 && mesinha2.getRawAxis(kBlackAxisUp)>-0.3)
        {
            return true;
        }
        
       return false;
    }
    public int getRedAxisUpInt(double percent)
    {
        if ( mesinha2.getRawAxis(kBlackAxisUp) >percent)
        {
            return 1;
        }
        
       return 0;
    }

    public int getRedAxisDownInt(double percent)
    {
        if ( mesinha2.getRawAxis(kBlackAxisUp) < percent)
        {
            return -1;
        }
        
       return 0;
    }
    
    public int getRedAxisUpDownCenterInt(double percent)
    {
        if (getRedAxisDownInt(-0.75)==0&&getRedAxisUpInt(0.75)==0)
        {
            return 1;
        }
        
       return 0;
    }

    public boolean getBlackAxisUpDownHome( )
    {
        if ( mesinha1.getRawAxis(kBlackAxisUp) < 0.3 && mesinha1.getRawAxis(kBlackAxisUp)>-0.3)
        {
            return true;
        }
        
       return false;
    }

    public boolean getBlackAxisLeftRightHome( )
    {
        if ( mesinha1.getRawAxis(1) < 0.3 && mesinha1.getRawAxis(1)>-0.3)
        {
            return true;
        }
        
       return false;
    }

    

       public int getBlackAxisUpInt(double percent)
    {
        if ( mesinha1.getRawAxis(kBlackAxisUp) >percent)
        {
            return 1;
        }
        
       return 0;
    }

    public int getBlackAxisDownInt(double percent)
    {
        if ( mesinha1.getRawAxis(kBlackAxisUp) < percent)
        {
            return -1;
        }
        
       return 0;
    }
    
    public int getBlackAxisUpDownCenterInt(double percent)
    {
        if (getBlackAxisDownInt(-0.75)==0&&getBlackAxisDownInt(0.75)==0)
        {
            return 1;
        }
       return 0;
    }


   
    //Mesinha2
    public boolean getRedAxisSides(double percent){
        return mesinha2.getRawAxis(kBlackAxisUp) < percent;
    }

    //Mesinha1
    public boolean getOne(){
        return mesinha1.getRawButton(kOne);
    }

    //Mesinha1
    public boolean getTwo(){
        return mesinha1.getRawButton(kTwo);
    }

    //Mesinha1
    public boolean getThree(){
        return mesinha1.getRawButton(kThree);
    }

    //Mesinha1
    public boolean getFour(){
        return mesinha1.getRawButton(kFour);
    }
    //Mesinha2
    public boolean getFive(){
        return mesinha2.getRawButton(kFive);
    }
    //Mesinha2
    public boolean getSix(){
        return mesinha2.getRawButton(kSix);
    }
    //Mesinha2
    public boolean getSeven(){
        return mesinha2.getRawButton(kSeven);
    }

    public boolean getEight(){
        return mesinha1.getRawButton(kEight);
    }

    public boolean getNine(){
        return mesinha2.getRawButton(kNine);
    }
    
    //Mesinha1
    public boolean getSelectOne(){
        return mesinha1.getRawButton(kSelectOne);
    }

    //Mesinha1
    public boolean getSelectTwo(){
        return mesinha1.getRawButton(kSelectTwo);
    }
        
    //Mesinha1
    public boolean getSelectTrhee(){
        return mesinha1.getRawButton(kSelectThree);
    }

}
