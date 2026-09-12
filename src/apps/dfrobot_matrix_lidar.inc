// Vendored (not fetched via lib_deps) I2C driver for the DFRobot Gravity
// 8x8 Matrix ToF sensor (VL53L7CX + onboard RP2040, SKU SEN0628).
// Trimmed from github.com/DFRobot/DFRobot_MatrixLidar (MIT license,
// (c) 2024 DFRobot Co.Ltd) down to the I2C path only - the upstream repo
// also ships a UART variant we don't use. Protocol logic (packet
// layout, command/status bytes) kept as-is from upstream; only the
// UART class and the intermediate virtual-base abstraction (which only
// mattered for sharing code between the I2C/UART variants) were
// dropped, since this project only ever talks I2C.
#include <Arduino.h>
#include <Wire.h>

typedef enum{
  eMatrix_4x4 = 4,
  eMatrix_8X8 = 8,
}eMatrix_t;

#define MATRIXLIDAR_CMD_SETMODE 1
#define MATRIXLIDAR_CMD_ALLDATA 2

#define MATRIXLIDAR_STATUS_SUCCESS 0x53  ///< Status of successful response
#define MATRIXLIDAR_STATUS_FAILED  0x63  ///< Status of failed response

#define MATRIXLIDAR_I2C_MAX_TRANSFER 32  ///< Max bytes per I2C transaction
#define MATRIXLIDAR_TIMEOUT_MS 8000

typedef struct{
  uint8_t head;
  uint8_t argsNumH;
  uint8_t argsNumL;
  uint8_t cmd;
  uint8_t args[0];
}__attribute__ ((packed)) sMatrixLidarSendPkt_t, *pMatrixLidarSendPkt_t;

typedef struct{
  uint8_t status;
  uint8_t cmd;
  uint8_t lenL;
  uint8_t lenH;
  uint8_t buf[0];
}__attribute__ ((packed)) sMatrixLidarRecvPkt_t, *pMatrixLidarRecvPkt_t;

class DFRobot_MatrixLidar_I2C {
public:
  DFRobot_MatrixLidar_I2C(uint8_t addr = 0x33, TwoWire *pWire = &Wire)
    : _pWire(pWire), _addr(addr) {}

  uint8_t begin(void) {
    _pWire->begin();
    _pWire->beginTransmission(_addr);
    return _pWire->endTransmission() ? 1 : 0;
  }

  uint8_t setRangingMode(eMatrix_t matrix) {
    uint8_t length = 4;
    pMatrixLidarSendPkt_t sendpkt = (pMatrixLidarSendPkt_t)malloc(sizeof(sMatrixLidarSendPkt_t) + length);
    if (sendpkt == NULL) return 1;
    sendpkt->head = 0x55;
    sendpkt->argsNumH = ((length + 1) >> 8) & 0xFF;
    sendpkt->argsNumL = (length + 1) & 0xFF;
    sendpkt->cmd = MATRIXLIDAR_CMD_SETMODE;
    sendpkt->args[0] = 0;
    sendpkt->args[1] = 0;
    sendpkt->args[2] = 0;
    sendpkt->args[3] = matrix;

    sendPacket(sendpkt, length + sizeof(sMatrixLidarSendPkt_t));
    free(sendpkt);

    uint8_t errorCode;
    pMatrixLidarRecvPkt_t rcvpkt = (pMatrixLidarRecvPkt_t)recvPacket(MATRIXLIDAR_CMD_SETMODE, &errorCode);
    bool ok = rcvpkt != NULL && rcvpkt->status == MATRIXLIDAR_STATUS_SUCCESS;
    if (rcvpkt != NULL) free(rcvpkt);
    if (ok) delay(5000);  // sensor needs time to switch ranging mode
    return ok ? 0 : 1;
  }

  // buf must point at an array of (matrix*matrix) uint16_t values (mm).
  uint8_t getAllData(void *buf) {
    pMatrixLidarSendPkt_t sendpkt = (pMatrixLidarSendPkt_t)malloc(sizeof(sMatrixLidarSendPkt_t));
    if (sendpkt == NULL) return 1;
    sendpkt->head = 0x55;
    sendpkt->argsNumH = 0;
    sendpkt->argsNumL = 1;
    sendpkt->cmd = MATRIXLIDAR_CMD_ALLDATA;

    sendPacket(sendpkt, sizeof(sMatrixLidarSendPkt_t));
    free(sendpkt);

    uint8_t errorCode;
    pMatrixLidarRecvPkt_t rcvpkt = (pMatrixLidarRecvPkt_t)recvPacket(MATRIXLIDAR_CMD_ALLDATA, &errorCode);
    if (rcvpkt == NULL) return 1;
    bool ok = rcvpkt->status == MATRIXLIDAR_STATUS_SUCCESS;
    if (ok) {
      uint16_t length = (rcvpkt->lenH << 8) | rcvpkt->lenL;
      memcpy(buf, rcvpkt->buf, length);
    }
    free(rcvpkt);
    return ok ? 0 : 1;
  }

private:
  TwoWire *_pWire;
  uint8_t _addr;

  void sendPacket(void *pkt, int length) {
    uint8_t *pBuf = (uint8_t *)pkt;
    int remain = length;
    if (pkt == NULL || length == 0) return;
    _pWire->beginTransmission(_addr);
    while (remain) {
      int chunk = (remain > MATRIXLIDAR_I2C_MAX_TRANSFER) ? MATRIXLIDAR_I2C_MAX_TRANSFER : remain;
      _pWire->write(pBuf, chunk);
      remain -= chunk;
      pBuf += chunk;
      if (remain) _pWire->endTransmission(true);
    }
    _pWire->endTransmission();
  }

  int recvData(void *data, int len) {
    uint8_t *pBuf = (uint8_t *)data;
    int remain = len;
    int total = 0;
    if (pBuf == NULL) return 0;
    while (remain) {
      int chunk = remain > MATRIXLIDAR_I2C_MAX_TRANSFER ? MATRIXLIDAR_I2C_MAX_TRANSFER : remain;
      remain -= chunk;
      _pWire->requestFrom(_addr, chunk, true);
      for (int i = 0; i < chunk; i++) {
        pBuf[i] = _pWire->read();
        yield();
      }
      pBuf += chunk;
      total += chunk;
    }
    return total;
  }

  void *recvPacket(uint8_t cmd, uint8_t *errorCode) {
    sMatrixLidarRecvPkt_t recvPkt;
    pMatrixLidarRecvPkt_t recvPktPtr = NULL;
    uint32_t t = millis();
    while (millis() - t < MATRIXLIDAR_TIMEOUT_MS) {
      recvData(&recvPkt.status, 1);
      if (recvPkt.status == MATRIXLIDAR_STATUS_SUCCESS || recvPkt.status == MATRIXLIDAR_STATUS_FAILED) {
        recvData(&recvPkt.cmd, 1);
        if (recvPkt.cmd != cmd) {
          if (errorCode) *errorCode = 1;
          return NULL;
        }
        recvData(&recvPkt.lenL, 2);
        uint16_t length = (recvPkt.lenH << 8) | recvPkt.lenL;
        if (length >= 1000) return NULL;
        recvPktPtr = (pMatrixLidarRecvPkt_t)malloc(sizeof(sMatrixLidarRecvPkt_t) + length);
        if (recvPktPtr == NULL) {
          if (errorCode) *errorCode = 1;
          return NULL;
        }
        memcpy(recvPktPtr, &recvPkt, sizeof(sMatrixLidarRecvPkt_t));
        if (length) recvData(recvPktPtr->buf, length);
        if (errorCode) *errorCode = 0;
        return recvPktPtr;
      }
      delay(17);
    }
    if (errorCode) *errorCode = 1;  // timed out
    return NULL;
  }
};
