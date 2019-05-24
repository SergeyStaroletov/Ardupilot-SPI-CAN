// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

 #if CONFIG_SONAR == ENABLED
static void init_sonar(void)
{
    sonar.init();
}
#endif

static void init_barometer(bool full_calibration)
{
    gcs_send_text_P(SEVERITY_LOW, PSTR("Calibrating barometer"));
    if (full_calibration) {
        barometer.calibrate();
    }else{
        barometer.update_calibration();
    }
    gcs_send_text_P(SEVERITY_LOW, PSTR("barometer calibration complete"));
}

// return barometric altitude in centimeters
static void read_barometer(void)
{
    barometer.read();
    if (should_log(MASK_LOG_IMU)) {
        Log_Write_Baro();
    }
    baro_alt = barometer.get_altitude() * 100.0f;
    baro_climbrate = barometer.get_climb_rate() * 100.0f;

    // run glitch protection and update AP_Notify if home has been initialised
    baro_glitch.check_alt();
    bool report_baro_glitch = (baro_glitch.glitching() && !ap.usb_connected && hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED);
    if (AP_Notify::flags.baro_glitching != report_baro_glitch) {
        if (baro_glitch.glitching()) {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_BARO_GLITCH);
        } else {
            Log_Write_Error(ERROR_SUBSYSTEM_BARO, ERROR_CODE_ERROR_RESOLVED);
        }
        AP_Notify::flags.baro_glitching = report_baro_glitch;
    }
}

// return sonar altitude in centimeters
static int16_t read_sonar(void)
{
#if CONFIG_SONAR == ENABLED
    sonar.update();

    // exit immediately if sonar is disabled
    if (!sonar_enabled || !sonar.healthy()) {
        sonar_alt_health = 0;
        return 0;
    }

    int16_t temp_alt = sonar.distance_cm();

    if (temp_alt >= sonar.min_distance_cm() && 
        temp_alt <= sonar.max_distance_cm() * SONAR_RELIABLE_DISTANCE_PCT) {
        if ( sonar_alt_health < SONAR_ALT_HEALTH_MAX ) {
            sonar_alt_health++;
        }
    }else{
        sonar_alt_health = 0;
    }

 #if SONAR_TILT_CORRECTION == 1
    // correct alt for angle of the sonar
    float temp = ahrs.cos_pitch() * ahrs.cos_roll();
    temp = max(temp, 0.707f);
    temp_alt = (float)temp_alt * temp;
 #endif

    return temp_alt;
#else
    return 0;
#endif
}

static void init_compass()
{
    if (!compass.init() || !compass.read()) {
        // make sure we don't pass a broken compass to DCM
        cliSerial->println_P(PSTR("COMPASS INIT ERROR"));
        Log_Write_Error(ERROR_SUBSYSTEM_COMPASS,ERROR_CODE_FAILED_TO_INITIALISE);
        return;
    }
    ahrs.set_compass(&compass);
}

static void init_optflow()
{
#if OPTFLOW == ENABLED
    optflow.init();
    if (!optflow.healthy()) {
        g.optflow_enabled = false;
        cliSerial->print_P(PSTR("Failed to Init OptFlow\n"));
        Log_Write_Error(ERROR_SUBSYSTEM_OPTFLOW,ERROR_CODE_FAILED_TO_INITIALISE);
    }
#endif      // OPTFLOW == ENABLED
}

// read_battery - check battery voltage and current and invoke failsafe if necessary
// called at 10hz
static void read_battery(void)
{
    battery.read();

    // update compass with current value
    if (battery.monitoring() == AP_BATT_MONITOR_VOLTAGE_AND_CURRENT) {
        compass.set_current(battery.current_amps());
    }

    // check for low voltage or current if the low voltage check hasn't already been triggered
    // we only check when we're not powered by USB to avoid false alarms during bench tests
    if (!ap.usb_connected && !failsafe.battery && battery.exhausted(g.fs_batt_voltage, g.fs_batt_mah)) {
        failsafe_battery_event();
    }
}

// read the receiver RSSI as an 8 bit number for MAVLink
// RC_CHANNELS_SCALED message
void read_receiver_rssi(void)
{
    // avoid divide by zero
    if (g.rssi_range <= 0) {
        receiver_rssi = 0;
    }else{
        rssi_analog_source->set_pin(g.rssi_pin);
        float ret = rssi_analog_source->voltage_average() * 255 / g.rssi_range;
        receiver_rssi = constrain_int16(ret, 0, 255);
    }
}



//addition
#define OTHER_SELECT_PIN 61
#define SELECT_OTHER digitalWrite(OTHER_SELECT_PIN, LOW);
#define DESELECT_OTHER digitalWrite(OTHER_SELECT_PIN, HIGH);
#define SPI_FIRST_TRANSACTION_DELAY 30
#define SPI_TRANSACTION_DELAY 1

const uint8_t READ = 0b11111100;     // SPI's read command
const uint8_t WRITE = 0b00000010;   // SPI''s write command
const uint8_t ACK = 0xaa;
const uint8_t NAK = 0x55;

// cmd packet
typedef struct {
    uint8_t len;
    uint8_t cmd;
}spi_packet_hdr_t;

typedef enum{
    SPI_CMD_RPM_VERSION = 0,
    SPI_CMD_RPM_DATA,
    SPI_CMD_ETC,
    SPI_CMD_MAX
}SPI_CMD_T;


typedef struct {
    spi_packet_hdr_t hdr;
}spi_packet_rpmData_t;

typedef struct {
    spi_packet_hdr_t hdr;
}spi_packet_etc_t;

typedef union {
    spi_packet_hdr_t hdr;
    spi_packet_rpmData_t rpmData;
    spi_packet_etc_t etc;
}spi_packet_t;

// response packet
typedef struct {
    uint8_t len;
    char* data;
}spi_resp_packet_rpmData_t;

typedef struct {
    uint8_t len;
    uint8_t data;
}spi_resp_packet_etc_t;

typedef union {
    spi_resp_packet_rpmData_t rpmData;
    spi_resp_packet_etc_t etc;
}spi_resp_packet_t;

spi_packet_t spi_packet_buf;

uint8_t spi_rpm_version[16]; // we may use just 8byte for saving the version string.

AP_HAL::SPIDeviceDriver *_spi_pok;

#define MAX_VERSION_NAME 8
const char version[MAX_VERSION_NAME] = "kang-01";

static void ReadPOK_Update(void)
{
    /*
            this function will be call every 1msec by timer2 isr.
      */
    AP_HAL::Semaphore* _spi_sem_rpm;
    uint8_t resp, read_len;
    static uint32_t                 _timer = 0;
    uint32_t tnow = hal.scheduler->micros();

    // Throttle read rate to 100hz maximum.
    if (tnow - _timer < 10000) {
        return;
    }

    _timer = tnow;

    /////////////////////////////////////////////////////////////////////////////////////////////
    //   Sart - Read the RPM
    /////////////////////////////////////////////////////////////////////////////////////////////

    // get spi bus semaphore
    _spi_sem_rpm = _spi_pok->get_semaphore();

    if (_spi_sem_rpm == NULL || !_spi_sem_rpm->take_nonblocking())
    {
        //cliSerial->println_P(PSTR("ReadRPM() failed - sem error "));
        goto spi_error_retrun;
    }

    _spi_pok->cs_assert();

    //First, send the length to be sent data :
    _spi_pok->transfer(0x2); // Length
    hal.scheduler->delay_microseconds(SPI_FIRST_TRANSACTION_DELAY);

    // Second, send the cmd ID
    resp = _spi_pok->transfer(SPI_CMD_RPM_DATA);

    //Check whether resp is ACK or not.
    if(resp != ACK)
    {
        goto spi_error_sem_and_cs_retrun;
    }

    hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);

    //Third, Read  the lengh which we should read.
    read_len = _spi_pok->transfer(ACK);

    for(uint8_t i = 0; i < read_len; i++)
    {
      hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);
      //((char*)&rpm_data)[i] = _
      _spi_pok->transfer(ACK);
    }

    // release slave select
    _spi_pok->cs_release();


    // release the spi bus
    _spi_sem_rpm->give();

    /////////////////////////////////////////////////////////////////////////////////////////////
    //   End - Read the RPM
    /////////////////////////////////////////////////////////////////////////////////////////////

    return;

spi_error_retrun:
    return;

spi_error_sem_and_cs_retrun:
    // release slave select
    _spi_pok->cs_release();

    // release the spi bus
    _spi_sem_rpm->give();

    //cliSerial->println_P(PSTR("ReadRPM() failed - spi Nak"));
}

static void ReadPOK_Init(void)
{
    AP_HAL::Semaphore* _spi_sem_rpm;
    uint8_t resp, read_len;
    int i;
    char hello[] = "hello, world\n";

    _spi_pok = hal.spi->device(AP_HAL::SPIDevice_POK);
    if (_spi_pok == NULL)
    {
        cliSerial->println_P(PSTR("ReadPOKInit() failed - device error "));
        return;
    }

    // now that we have initialised, we set the SPI bus speed to high
    // (2MHz on APM2)
    _spi_pok->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

    for(i = 0; i < 100; i++)
    {
        /////////////////////////////////////////////////////////////////////////////////////////////
        //   Sart - Check the SPI device version
        /////////////////////////////////////////////////////////////////////////////////////////////

        hal.scheduler->suspend_timer_procs();

        // get spi bus semaphore
        _spi_sem_rpm = _spi_pok->get_semaphore();

        if (_spi_sem_rpm == NULL)
        {
            cliSerial->println_P(PSTR("ReadPOK_Init() failed - sem error "));
            goto error_continue;
        }

        if (!_spi_sem_rpm->take(100))
            hal.scheduler->panic(PSTR("ReadRPM_Init: Unable to get semaphore"));

        _spi_pok->cs_assert();

        //Send hello injection!!

        for (int h = 0; h < sizeof(hello); h++) {
        	_spi_pok->transfer(hello[h]);
        }


        //First, send the length to be sent data :
        _spi_pok->transfer(0x2); // length
        hal.scheduler->delay_microseconds(SPI_FIRST_TRANSACTION_DELAY);

        // second, send the cmd ID
        resp = _spi_pok->transfer(SPI_CMD_RPM_VERSION);

        //Check whether resp is ACK or not.
        if(resp != ACK)
        {
            // release slave select
            _spi_pok->cs_release();

            // release the spi bus
            _spi_sem_rpm->give();

            cliSerial->println_P(PSTR("ReadPOK_Init() failed - spi Nak"));
            goto error_continue;
        }

        hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);

        //Third, Read  the lengh which we should read.
        read_len = _spi_pok->transfer(ACK);

        for(uint8_t i = 0; i < read_len; i++)
        {
          hal.scheduler->delay_microseconds(SPI_TRANSACTION_DELAY);
          spi_rpm_version[i] = _spi_pok->transfer(ACK);
        }

        // release slave select
        _spi_pok->cs_release();


        // release the spi bus
        _spi_sem_rpm->give();

        hal.scheduler->resume_timer_procs();

        if(strcmp((const char *)spi_rpm_version, version) == 0)
        {
            cliSerial->printf_P(PSTR("\nReadPOK_Init() - version= %s \n"), (uint8_t *)spi_rpm_version);
            break;
        }
        else
        {
            cliSerial->printf_P(PSTR("\nReadRPM_Init() - version= read failed \n"));
        }
        /////////////////////////////////////////////////////////////////////////////////////////////
        //   End - Check the SPI device version
        /////////////////////////////////////////////////////////////////////////////////////////////

        error_continue:
            hal.scheduler->resume_timer_procs();

            hal.scheduler->delay(200);
    }

    if(i != 3)
        hal.scheduler->register_timer_process((AP_HAL::MemberProc)&ReadPOK_Update);

    return;
}
