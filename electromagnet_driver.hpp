#ifndef ELECTROMAGNET_DRIVER_
#define ELECTROMAGNET_DRIVER_


#define SOLENOIDPIN 3

bool solenoid_state;

// class solenoid_Driver {

// public:
//     solenoid_Driver();
//     ~solenoid_Driver();
    
    void initializeSolenoid()
    {
      pinMode(SOLENOIDPIN, OUTPUT);
      solenoid_state = false;
    }
    
//     void solenoidState()
//     {
//       digitalWrite(SOLENOIDPIN, HIGH);
//     }

// private:


// };


#endif