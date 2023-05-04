/////////////////////////////////////////////////////////////////////
//  File:  pneumatics.java
/////////////////////////////////////////////////////////////////////
//
//  Purpose:  Encapsulates pneumatics operation in a class that
//            implements wpilibj features regarding pneumatic
//            operations.  Assumes use of the CTR pneumatic
//            module.
//
//  Compiler: Java via Microsoft VS
//
//  Inception Date:  1/21/2022
//
//  Revisions:  2/12/2022:  Cleaned up, added comments.
//  Note that the test board does not have an analog
//  pressure sensor - we can't read the pressure or set
//  analog limits.  Our present CTREPCM does not have the
//  analog inputs.  Functions for doing these operations
//  are included but are not used at this time.  We would
//  need a pneumatics hub.  See https://www.revrobotics.com/rev-11-1852/
//  for details.  I think the same argument applies to the
//  reading of the compressor current.
//
//
/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////


package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
//import edu.wpi.first.wpilibj.AnalogInput;
//import edu.wpi.first.wpilibj.AnalogPotentiometer;

public class pneumatics {
    Compressor comp;
    Solenoid s_solenoid;  //  single solenoid
    DoubleSolenoid d_solenoid;  //  double solenoid


    //  default constructor
    pneumatics()  {

        comp=new Compressor(0,PneumaticsModuleType.CTREPCM);

        d_solenoid=new DoubleSolenoid(PneumaticsModuleType.CTREPCM,0,1);        
    
    }

   
    /////////////////////////////////////////////////////////////////
    //  Function:      int compressorOff()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Turns the compressor off
    //
    //  Arguments: void
    //
    //  Returns: Zero if successful, '1' if compressor is still
    //           enabled.
    //
    //  Remarks:  Tested 2/12/2022 - works
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int compressorOff()
    {
        boolean status;

        comp.disable();
        status=comp.enabled();
        if(status==false)  {
            return(0);
        }  
        return(1);
  
    }

    /////////////////////////////////////////////////////////////////
    //  Function:    int compressorOn()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Should turn the compressor on.
    //
    //  Arguments:void
    //
    //  Returns:  Zero if successful,'1' if compressor is not
    //            enabled.
    //
    //  Remarks:  2/12/2022:  Untested  
    //            2/13/2022:  Tested, works as advertised
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int compressorOn()
    {
        boolean status;

        comp.enableDigital();
        status=comp.enabled();
        if(status==false)  {
            return(1);
        }  
        return(0);
  
    }

    /////////////////////////////////////////////////////////////////
    //  Function:      int compressorSetLimits(double low_pressure,double high_pressure)
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Sets the compressor upper and lower limits
    //
    //  Arguments:doubles representing the low and high pressure
    //            limits.
    //
    //  Returns:  Zero
    //
    //  Remarks:  2/12/2022:  Untested.  Not of much use unless
    //  we have a pneumatics HUB and analog sensor.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    int compressorSetLimits(double low_pressure,double high_pressure)
    {
        comp.enableAnalog(low_pressure, high_pressure);
        return(0);

    }

    /////////////////////////////////////////////////////////////////
    //  Function:     double compressorReadPressure()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Reads the compressor (tank?) pressure
    //
    //  Arguments:void
    //
    //  Returns: Returns pressure in psig
    //
    //  Remarks: 2/12/2022:  Untested, test board would require
    //  an analog pressure sensor.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compressorReadPressure() {
        double pressure;

        pressure=comp.getPressure();

        return(pressure);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:   double compressorReadCurrent()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose: Reads compressor current.
    //
    //  Arguments:void
    //
    //  Returns: Returns the compressor current in amps.
    //
    //  Remarks: 2/12/2022:  Untested, not sure if test board has
    //  a current sensor.
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    double compressorReadCurrent() {
        double current;

        current=comp.getCurrent();
        
        return(current);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:    void enableSingleSolenoid()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Opens the single solenoid valve.
    //
    //  Arguments:void
    //
    //  Returns:  void
    //
    //  Remarks:  2/12/2022:  Untested
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    void enableSingleSolenoid()  {
        s_solenoid.set(true);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:  void disableSingleSolenoid()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Closes the single solenoid valve.
    //
    //  Arguments: void
    //
    //  Returns:  void
    //
    //  Remarks:  2/2/2022:  Untested
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    void disableSingleSolenoid()  {
        s_solenoid.set(false);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:    void disableDoubleSolenoid()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Closes both forward and reverse solenoid valves
    //
    //  Arguments:void
    //
    //  Returns:  void
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    void disableDoubleSolenoid()
    {
        d_solenoid.set(DoubleSolenoid.Value.kOff);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:   void enableDoubleSolenoidForward()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Opens forward solenoid channel
    //
    //  Arguments:void
    //
    //  Returns:  void
    //
    //  Remarks:  2/12/2022:  Tested - works
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    void enableDoubleSolenoidForward()
    {
        d_solenoid.set(DoubleSolenoid.Value.kForward);
    }

    /////////////////////////////////////////////////////////////////
    //  Function:   void enableDoubleSolenoidForward()
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:  Opens reverse solenoid channel
    //
    //  Arguments: void
    //
    //  Returns:  void
    //
    //  Remarks:  2/12/2022:  Tested - works
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
    void enableDoubleSolenoidReverse()
    {
        d_solenoid.set(DoubleSolenoid.Value.kReverse);
    }

    
} //  end of class definition

    /////////////////////////////////////////////////////////////////
    //  Function:
    /////////////////////////////////////////////////////////////////
    //
    //  Purpose:
    //
    //  Arguments:
    //
    //  Returns:
    //
    //  Remarks:
    //
    /////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////
