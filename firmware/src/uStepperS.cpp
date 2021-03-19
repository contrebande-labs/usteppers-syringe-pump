/********************************************************************************************
* 	 	File: 		uStepperS.cpp															*
*		Version:    2.2.0                                           						*
*      	Date: 		September 22nd, 2020  	                                    			*
*      	Authors: 	Thomas Hørring Olsen                                   					*
*					Emil Jacobsen															*
*                                                   										*	
*********************************************************************************************
*	(C) 2020																				*
*																							*
*	uStepper ApS																			*
*	www.ustepper.com 																		*
*	administration@ustepper.com 															*
*																							*
*	The code contained in this file is released under the following open source license:	*
*																							*
*			Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International			*
* 																							*
* 	The code in this file is provided without warranty of any kind - use at own risk!		*
* 	neither uStepper ApS nor the author, can be held responsible for any damage				*
* 	caused by the use of the code contained in this file ! 									*
*                                                                                           *
********************************************************************************************/
/**
* @file uStepperS.cpp
*
* @brief      Function and class implementation for the uStepper S library
*
*             This file contains class and function implementations for the library.
*
* @author     Thomas Hørring Olsen (thomas@ustepper.com)
*/
#include <uStepperS.h>

uStepperS* uStepperS::singleton = NULL; 

uStepperS::uStepperS()
{
	singleton = this;

	this->microSteps = 256;
	this->init();	

	this->setMaxAcceleration(2000.0);
	this->setMaxDeceleration(2000.0);
	this->setMaxVelocity(100.0);
}

uStepperS* uStepperS::getInstance() {

	if (!singleton) singleton = new uStepperS;

	return singleton;

}

void uStepperS::init( void ){

	
	this->pidDisabled = 1;
	/* Set CS, MOSI, SCK and DRV_ENN as Output */
	DDRC = (1<<SCK1)|(1<<MOSI_ENC);
	DDRD = (1<<DRV_ENN)|(1<<SD_MODE)|(1<<CS_ENCODER);
	DDRE = (1<<MOSI1)|(1<<CS_DRIVER);

	PORTD |= (1 << DRV_ENN);  // Set DRV_ENN HIGH, while configuring
	PORTD &= ~(1 << SD_MODE);  // Set SD_MODE LOW  

	/* 
	*  ---- Global SPI1 configuration ----
	*  SPE   = 1: SPI enabled
	*  MSTR  = 1: Master
	*  SPR0  = 0 & SPR1 = 0: fOSC/4 = 4Mhz
	*/
	SPCR1 = (1<<SPE1)|(1<<MSTR1);	

	driver.init( this );
	encoder.init( this );
	PORTD &= ~(1 << DRV_ENN);  // Set DRV_ENN LOW
}

bool uStepperS::getMotorState(uint8_t statusType)
{
	this->driver.readMotorStatus();
	if(this->driver.status & statusType)
	{
		return 0;
	}
	return 1;
}

float uStepperS::getDriverRPM( void )
{
	int32_t velocity = this->driver.getVelocity();

	return (float)velocity * this->velToRpm;
}

void uStepperS::checkOrientation(const uint16_t angleDeg) {

	int32_t angleMicroDeg = ((uint32_t) angleDeg) * 1000 * 1000;
	
	uint8_t inverted = 0;

	uint8_t noninverted = 0;

	this->disablePid();

	this->shaftDir = 0;

	this->driver.setShaftDirection(this->shaftDir);
	
	int32_t startAngleMicroDeg = this->encoder.getMicroDegAngleMoved();


	this->moveAngle(angleMicroDeg / (1000 * 1000));


/*
	while(this->getMotorState());

	startAngleMicroDeg -= angleMicroDeg / 2;

	int32_t angleMovedMilliDeg = this->encoder.getMicroDegAngleMoved();

	if(angleMovedMilliDeg < startAngleMicroDeg) {

		inverted++;

	} else {

		noninverted++;

	}

	startAngleMicroDeg = this->encoder.getMicroDegAngleMoved();

	this->moveAngle(-angleMicroDeg / (1000 * 1000));

	while(this->getMotorState());

	startAngleMicroDeg += angleMicroDeg / 2;

	angleMovedMilliDeg = this->encoder.getMicroDegAngleMoved();

	if(angleMovedMilliDeg > startAngleMicroDeg) {

		inverted++;

	} else {

		noninverted++;

	}

	startAngleMicroDeg = this->encoder.getMicroDegAngleMoved();

	this->moveAngle(angleMicroDeg / (1000 * 1000));

	while(this->getMotorState());

	startAngleMicroDeg -= angleMicroDeg / 2.0;

	angleMovedMilliDeg = this->encoder.getMicroDegAngleMoved();

	if(angleMovedMilliDeg < startAngleMicroDeg) {

		inverted++;

	} else {

		noninverted++;

	}

	this->moveAngle(-angleMicroDeg / (1000 * 1000));
	
	while(this->getMotorState());

	if(inverted > noninverted) {

		this->shaftDir = 1;

		this->driver.setShaftDirection(this->shaftDir);

	}

	this->enablePid();
*/
}

void uStepperS::setup(	uint16_t stepsPerRevolution,
						bool setHome,
						uint8_t runCurrent,
						uint8_t holdCurrent)
{
	this->pidDisabled = 1;

	// Should setup mode etc. later
	this->fullSteps = stepsPerRevolution;
	this->angleToStep = (float)this->fullSteps * (float)this->microSteps / 360.0;
	this->rpmToVelocity = (float)(279620.267 * fullSteps * microSteps)/(CLOCKFREQ);
	this->stepsPerSecondToRPM = 60.0/(this->microSteps*this->fullSteps);
	this->RPMToStepsPerSecond = (this->microSteps*this->fullSteps)/60.0;

	this->stepTime = 16777216.0/CLOCKFREQ; // 2^24/CLOCKFREQ
	this->rpmToVel = (this->fullSteps*this->microSteps)/(60.0/this->stepTime);
	this->velToRpm = 1.0/this->rpmToVel;

	this->init();

	this->driver.setDeceleration( (uint32_t)( this->maxDeceleration ) );
	this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );

	this->setCurrent(0.0);
	this->setHoldCurrent(0.0);

	this->setCurrent(40.0);
	this->setHoldCurrent(0.0);	

	this->encoder.Beta = 4;

	if(setHome == true) encoder.setHome();

	this->pidDisabled = 0;

	DDRB |= (1 << 4);
}

void uStepperS::moveSteps( int32_t steps )
{
	this->driver.setDeceleration( (uint16_t)( this->maxDeceleration ) );
	this->driver.setAcceleration( (uint16_t)(this->maxAcceleration ) );
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );
	
	// Get current position
	int32_t current = this->driver.getPosition();

	// Set new position
	this->driver.setPosition( current + steps);
}



void uStepperS::moveAngle( float angle )
{
	int32_t steps;

	if(angle < 0.0)
	{
		steps = (int32_t)((angle * angleToStep) - 0.5);
		this->moveSteps( steps ); 
	}
	else
	{
		steps = (int32_t)((angle * angleToStep) + 0.5);
		this->moveSteps( steps );
	}
}


void uStepperS::moveToAngle( float angle )
{

	float diff = angle - this->angleMoved();
	int32_t steps = (int32_t)( (abs(diff) * angleToStep) + 0.5);

	if(diff < 0.0)
	{
		this->moveSteps( -steps );
	}
	else
	{
		this->moveSteps( steps );
	}
}

void uStepperS::enableStallguard( int8_t threshold, bool stopOnStall, float rpm )
{
	this->clearStall();
	this->stallThreshold = threshold;
	this->stallStop = stopOnStall;

	this->driver.enableStallguard( threshold, stopOnStall, rpm);

	this->stallEnabled = true;
}

void uStepperS::disableStallguard( void )
{
	this->driver.disableStallguard();

	this->stallEnabled = false;
}

void uStepperS::clearStall( void ) 
{
	this->driver.clearStall();
}

bool uStepperS::isStalled( void )
{
	return this->isStalled( this->stallThreshold );
}

bool uStepperS::isStalled( int8_t threshold )
{	
	// If the threshold is different from what is configured..
	if( threshold != this->stallThreshold || this->stallEnabled == false ){
		// Reconfigure stallguard
		this->enableStallguard( threshold, this->stallStop, 10 );
	}

	int32_t stats = this->driver.readRegister(RAMP_STAT);

	// Only interested in 'status_sg', with bit position 13 (last bit in RAMP_STAT).
	return ( stats >> 13 );
}

void uStepperS::setBrakeMode( uint8_t mode, float brakeCurrent )
{
	int32_t registerContent = this->driver.readRegister(PWMCONF);
	registerContent &= ~(3UL << 20);
	if(mode == FREEWHEELBRAKE)
	{
		this->setHoldCurrent(0.0);
		this->driver.writeRegister( PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(1) ); 
	}
	else if(mode == COOLBRAKE)
	{
		this->setHoldCurrent(0.0);
		this->driver.writeRegister( PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(2) );  
	}
	else
	{
		this->setHoldCurrent(brakeCurrent);
		this->driver.writeRegister( PWMCONF, PWM_AUTOSCALE(1) | PWM_GRAD(1) | PWM_AMPL(128) | PWM_FREQ(0) | FREEWHEEL(0) ); 
	}
}

void uStepperS::setRPM( float rpm)
{
	int32_t velocityDir = rpmToVelocity * rpm;

	if(velocityDir > 0){
		driver.setDirection(1);
	}else{
		driver.setDirection(0);
	}

	// The velocity cannot be signed
	uint32_t velocity = abs(velocityDir);

	driver.setVelocity( (uint32_t)velocity );
}


void uStepperS::setSPIMode( uint8_t mode ){

	switch(mode){
		case 2:
			SPCR1 |= (1<<CPOL1);  // Set CPOL HIGH = 1
			SPCR1 &= ~(1<<CPHA1);  // Set CPHA LOW = 0
		break;

		case 3:
			SPCR1 |= (1<<CPOL1);  // Set CPOL HIGH = 1
			SPCR1 |= (1<<CPHA1);  // Set CPHA HIGH = 1
		break;
	}
}

uint8_t uStepperS::SPI(uint8_t data){

	SPDR1 = data;

	// Wait for transmission complete
	while(!( SPSR1 & (1 << SPIF1) ));    

	return SPDR1;

}

void uStepperS::setMaxVelocity( float velocity )
{
	velocity *= (float)this->microSteps;
	velocity = abs(velocity)*VELOCITYCONVERSION;

	this->maxVelocity = velocity;

	// Steps per second, has to be converted to microsteps
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );
}

void uStepperS::setMaxAcceleration( float acceleration )
{
	acceleration *= (float)this->microSteps;
	acceleration = abs(acceleration) * ACCELERATIONCONVERSION;

	this->maxAcceleration = acceleration;

	
	// Steps per second, has to be converted to microsteps
	this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
}

void uStepperS::setMaxDeceleration( float deceleration )
{
	deceleration *= (float)this->microSteps;
	deceleration = abs(deceleration) * ACCELERATIONCONVERSION;
	
	this->maxDeceleration = deceleration;
	
	// Steps per second, has to be converted to microsteps
	this->driver.setDeceleration( (uint32_t)(this->maxDeceleration ) );
}

void uStepperS::setCurrent( double current )
{
	if( current <= 100.0 && current >= 0.0){
		// The current needs to be in the range of 0-31
		this->driver.current = ceil(0.31 * current); 
	}else{
		// If value is out of range, set default
		this->driver.current = 16; 
	}

	driver.updateCurrent();
}

void uStepperS::setHoldCurrent( double current )
{
	// The current needs to be in the range of 0-31
	if( current <= 100.0 && current >= 0.0){
		// The current needs to be in the range of 0-31
		this->driver.holdCurrent = ceil(0.31 * current);
	}else{
		// If value is out of range, set default
		this->driver.holdCurrent = 16; 
	}

	driver.updateCurrent();
}

void uStepperS::runContinous( bool direction )
{
	this->driver.setDeceleration( (uint32_t)( this->maxDeceleration ) );
	this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
	this->driver.setVelocity( (uint32_t)( this->maxVelocity  ) );

	// Make sure we use velocity mode
	this->driver.setRampMode( VELOCITY_MODE_POS );

	// Set the direction
	this->driver.setDirection( direction );
}

float uStepperS::angleMoved ( void )
{
	return this->encoder.getAngleMoved();
}

void uStepperS::stop( bool mode){

	if(mode == HARD)
	{
		this->driver.setDeceleration( 0xFFFE );
		this->driver.setAcceleration( 0xFFFE );
		this->setRPM(0);
		while(this->driver.readRegister(VACTUAL) != 0);
		this->driver.setDeceleration( (uint32_t)( this->maxDeceleration ) );
		this->driver.setAcceleration( (uint32_t)(this->maxAcceleration ) );
	}
	else
	{
		this->setRPM(0);
	}
}

void uStepperS::filterSpeedPos(posFilter_t *filter, int32_t steps)
{
	filter->posEst += filter->velEst * ENCODERINTPERIOD;	
	filter->posError = (float)steps - filter->posEst;
	filter->velIntegrator += filter->posError * PULSEFILTERKI * 0.5f;
	filter->velEst = (filter->posError * PULSEFILTERKP) + filter->velIntegrator;
}

void TIMER1_COMPA_vect(void) {
/*
	uStepperS * pointer = uStepperS::getInstance();

	pointer->encoder.captureAngle();
	
	if(!pointer->pidDisabled) {

		int32_t stepsMoved = pointer->driver.getPosition();

		pointer->currentPidError = stepsMoved - pointer->encoder.angleMoved * ENCODERDATATOSTEP;

		if(abs(pointer->currentPidError) >= pointer->controlThreshold) {

			pointer->driver.writeRegister(XACTUAL, pointer->encoder.angleMoved * ENCODERDATATOSTEP);

			pointer->driver.writeRegister(XTARGET, pointer->driver.xTarget);

		}
		
		pointer->currentPidSpeed = pointer->encoder.encoderFilter.velIntegrator * ENCODERDATATOSTEP;

	}
*/
}

void uStepperS::setControlThreshold(float threshold)
{
	this->controlThreshold = threshold;
}
void uStepperS::enablePid(void)
{
	cli();
	this->pidDisabled = 0;
	sei();
}

void uStepperS::disablePid(void)
{
	cli();
	this->pidDisabled = 1;
	sei();
}

float uStepperS::moveToEnd(bool dir, float rpm, int8_t threshold)
{
	// Lowest reliable speed for stallguard
	if (rpm < 10.0)
		rpm = 10.0;
	
	if(dir == CW)
		this->setRPM(abs(rpm));
	else
		this->setRPM(-abs(rpm));
	
	delay(100);

	this->isStalled();
	// Enable stallguard to detect hardware stop (use driver directly, as to not override user stall settings)
	this->driver.enableStallguard( threshold, true, rpm );

	float length = this->encoder.getAngleMoved();
	
	while( !this->isStalled() ){}
	this->stop();
	this->driver.clearStall();

	// Return to normal operation
	this->driver.disableStallguard();

	length -= this->encoder.getAngleMoved();
	delay(1000);
	return abs(length);
}

float uStepperS::getPidError(void)
{
	return this->currentPidError;
}