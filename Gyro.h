#include <twi_nonblock.h>
#define GYRO_IDLE 0
#define GYRO_READING 1
#define GYRO_WRITING 2
#define L3GD20_ADDRESS (0x6B)

class Gyro
{
  public:
    typedef enum
      {                                               // DEFAULT    TYPE
        L3GD20_REGISTER_WHO_AM_I            = 0x0F,   // 11010100   r
        L3GD20_REGISTER_CTRL_REG1           = 0x20,   // 00000111   rw
        L3GD20_REGISTER_CTRL_REG2           = 0x21,   // 00000000   rw
        L3GD20_REGISTER_CTRL_REG3           = 0x22,   // 00000000   rw
        L3GD20_REGISTER_CTRL_REG4           = 0x23,   // 00000000   rw
        L3GD20_REGISTER_CTRL_REG5           = 0x24,   // 00000000   rw
        L3GD20_REGISTER_REFERENCE           = 0x25,   // 00000000   rw
        L3GD20_REGISTER_OUT_TEMP            = 0x26,   //            r
        L3GD20_REGISTER_STATUS_REG          = 0x27,   //            r
        L3GD20_REGISTER_OUT_X_L             = 0x28,   //            r
        L3GD20_REGISTER_OUT_X_H             = 0x29,   //            r
        L3GD20_REGISTER_OUT_Y_L             = 0x2A,   //            r
        L3GD20_REGISTER_OUT_Y_H             = 0x2B,   //            r
        L3GD20_REGISTER_OUT_Z_L             = 0x2C,   //            r
        L3GD20_REGISTER_OUT_Z_H             = 0x2D,   //            r
        L3GD20_REGISTER_FIFO_CTRL_REG       = 0x2E,   // 00000000   rw
        L3GD20_REGISTER_FIFO_SRC_REG        = 0x2F,   //            r
        L3GD20_REGISTER_INT1_CFG            = 0x30,   // 00000000   rw
        L3GD20_REGISTER_INT1_SRC            = 0x31,   //            r
        L3GD20_REGISTER_TSH_XH              = 0x32,   // 00000000   rw
        L3GD20_REGISTER_TSH_XL              = 0x33,   // 00000000   rw
        L3GD20_REGISTER_TSH_YH              = 0x34,   // 00000000   rw
        L3GD20_REGISTER_TSH_YL              = 0x35,   // 00000000   rw
        L3GD20_REGISTER_TSH_ZH              = 0x36,   // 00000000   rw
        L3GD20_REGISTER_TSH_ZL              = 0x37,   // 00000000   rw
        L3GD20_REGISTER_INT1_DURATION       = 0x38    // 00000000   rw
      } l3gd20Registers_t;
    uint8_t gyrobytedata[6];
    float x;
    float y;
    float z;
    float xAngle;
    float yAngle;
    float zAngle;
    void setup_gyro();
    void update_gyro();
  private:
    uint8_t gyro_status;
    void initiate_request_gyro();
    void initiate_read_gyro();
    void finalise_request_gyro();
    void gyro_writeTo(byte address, byte val);
};



void Gyro::setup_gyro()
{
  initialize_twi_nonblock();
  gyro_writeTo(L3GD20_REGISTER_CTRL_REG1, 0x0F);
  gyro_writeTo(L3GD20_REGISTER_CTRL_REG4, 0x00);
  gyro_status = GYRO_IDLE;
}


/// ---------- non-blocking version ----------
void Gyro::initiate_read_gyro()
{
  // Reads num bytes starting from address register on device in to _buff array
  // indicate that we are transmitting
  //   transmitting = 1;
  // set address of targeted slave
  txAddress = L3GD20_ADDRESS;
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;

  // put byte in tx buffer
  txBuffer[txBufferIndex] = L3GD20_REGISTER_OUT_X_L | 0x80;
  ++txBufferIndex;
  // update amount in buffer   
  txBufferLength = txBufferIndex;

  twi_initiateWriteTo(txAddress, txBuffer, txBufferLength);
  gyro_status = GYRO_WRITING;
}

void Gyro::initiate_request_gyro()
{
  // reset tx buffer iterator vars
  txBufferIndex = 0;
  txBufferLength = 0;
  // indicate that we are done transmitting
  transmitting = 0;

  uint8_t read = twi_initiateReadFrom(L3GD20_ADDRESS, 6);
  gyro_status = GYRO_READING;
}

void Gyro::finalise_request_gyro()
{
  uint8_t read = twi_readMasterBuffer( rxBuffer, 6 );
  // set rx buffer iterator vars
  rxBufferIndex = 0;
  rxBufferLength = read;

  uint8_t i = 0;
  while( rxBufferLength - rxBufferIndex > 0)         // device may send less than requested (abnormal)
  {
    gyrobytedata[i] = rxBuffer[rxBufferIndex];
    ++rxBufferIndex;
    i++;
  }

  gyro_status = GYRO_IDLE;
}
/// ----end------ non-blocking version ----------

void Gyro::update_gyro()
{
    switch( gyro_status )
  {
  case GYRO_IDLE:
    x = (gyrobytedata[0] | (gyrobytedata[1] << 8)) * 0.0001;
    y = (gyrobytedata[2] | (gyrobytedata[3] << 8)) * 0.00875;
    z = (gyrobytedata[4] | (gyrobytedata[5] << 8)) * 0.00875;
    xAngle += x;
    yAngle += y;
    zAngle += z;
    initiate_read_gyro();
    break;
  case GYRO_WRITING:
    if ( TWI_MTX != twi_state ){
      initiate_request_gyro();
    }
    break;
  case GYRO_READING:
    if ( TWI_MRX != twi_state ){
      finalise_request_gyro();
    }
    break;
  }
}

// Writes val to address register on device
void Gyro::gyro_writeTo(byte address, byte val)
{
  //   Wire.beginTransmission(L3GD20_ADDRESS); // start transmission to device   
  twowire_beginTransmission(L3GD20_ADDRESS); // start transmission to device   
  //   Wire.send(address);             // send register address
  twowire_send( address );
  //   Wire.send(val);                 // send value to write
  twowire_send( val );
  //   Wire.endTransmission();         // end transmission
  twowire_endTransmission();
}
