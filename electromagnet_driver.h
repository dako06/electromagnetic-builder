#ifndef ELECTROMAGNET_DRIVER_
#define ELECTROMAGNET_DRIVER_

// openCR mapping 3 50 BDPIN_GPIO_1
#define SOLENOIDPIN 3


class solenoid_Driver {

public:
    solenoid_Driver();
    ~solenoid_Driver();
    void initializeSolenoid();
    void solenoidState();

private:


};


#endif