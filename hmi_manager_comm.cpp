#include <bitset>                                           //비트 단위 연산을 위한 클래스
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time_source.hpp"

#include "hmi_manager/hmi_manager_comm.hpp"

using namespace std::chrono_literals;                       //시간, 분, 초 등을 나타내는 12개의 사용자 정의 literal

namespace hmi_manager
{
constexpr int BUFFER_SIZE = 1024;                       //constexpr는 주로 상수를 지정하는데 사용되며, 읽기 전용 메모리에 저장해서 성능을 높인다.
                                                        // \const는 컴파일, 런타임 상수 가능하지만 constexpr은 컴파일 시간상수만 가능하다(명확하게 숫자로 변수 지정)
constexpr long TIME_LIMIT = 500;                        //int로 충분할텐데 왜 long으로 했는지? long은 표현범위가 int보다 엄청 큰 정수 자료형

volatile sig_atomic_t running = 1;                      //volatile로 선언된 변수는 레지스터에 로드된 값을 사용하지 않고 매번 메모리를 참조한다.
                                                        //Memory-mapped I/O ( ex)임베디드 장비), Interrupt Service Routine, Multithreading 환경에서 사용한다
                                                        //https://blog.naver.com/ycpiglet/222811478036 정리 잘되어있음
serial::Serial pSerial_;                                //serial 네임스페이스 안에 있는 함수 serial 호출..? 확인필요 클래스명::

HmiComm::HmiComm(const std::string & node_name)
: Node(node_name)
{


}

HmiComm::~HmiComm()                                     // 
{
  if (pCommThread_.joinable()) {
    pCommThread_.join();

    RCLCPP_INFO(rclcpp::get_logger("HmiComm"), "serial thead joined");
  }
}

void HmiComm::on_configure(
  const std::string port = "/dev/ttyS0", uint32_t baudrate = 115200,
  uint32_t timeout = 2000)
{
  try {
    serial::Timeout to = serial::Timeout::simpleTimeout(timeout);
    pSerial_.setPort(port);
    pSerial_.setBaudrate(baudrate);
    pSerial_.setTimeout(to);
    pSerial_.open();
  } catch (serial::IOException & e) {
    RCLCPP_INFO(rclcpp::get_logger("HmiComm"), "unable to open port");
  }

  if (pSerial_.isOpen()) {
    RCLCPP_INFO(rclcpp::get_logger("HmiComm"), "serial port initailzed");
  }

  runThread_ = true;
  pCommThread_ = std::thread(&HmiComm::serial_thread_func, this);
  language = AlarmHelper::E_LANG::ENGLISH;
}

uint16_t HmiComm::CRC_Cal(uint8_t * data_set, uint16_t Length)
{
  uint8_t Temp_Num;
  uint16_t CRC_Process = 0xFFFF;

  while (Length--) {
    Temp_Num = *data_set++ ^ CRC_Process;
    CRC_Process >>= 8;
    CRC_Process ^= CRC_Table[Temp_Num];
  }
  return CRC_Process;
}

void HmiComm::serial_thread_func()
{
  try {
    while (runThread_) {
      std::vector<KEY_VALUE> vAlarm = getAlarmlist();
      if (pSerial_.isOpen()) {
        BYTE buff_num = 0;
        BYTE recv_buff[250] = {0};

        buff_num = pSerial_.available();
        if (pSerial_.read(recv_buff, buff_num) > 0) {
          if (recv_buff[0] == SLAVE_ID) {
            switch (recv_buff[1]) {
              case FUNC_03:
                if (((int)recv_buff[2] << 8 | recv_buff[3]) == ADDR_Alarm) {
                  send_alarm_list(vAlarm);
                  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                }
                break;

              case FUNC_05:
                if (((int)recv_buff[2] << 8 | (int)recv_buff[3]) == ADDR_LanguageSet) {
                  recv_language_set();
                  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
                }
                break;
            }
          }
        }
      } else {
        //연결이 끊어질 시 1초에 한 번씩 serial open 시도
        RCLCPP_INFO(rclcpp::get_logger("HmiComm"), "serial disconnected!");
        pSerial_.open();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
    }
  } catch (const std::exception & e) {
    std::cerr << e.what() << '\n';
  }
}

std::vector<KEY_VALUE> HmiComm::getAlarmlist()
{
  auto request = std::make_shared<wia_msgs::srv::AlarmListReq::Request>();
  auto result = alarm_list_req_sync_serv_.sendRequest(request, 5s);
  std::vector<KEY_VALUE> alarmList;

  if (result.future_state == rclcpp::FutureReturnCode::SUCCESS) {
    for (auto alarm : result.response.result) {
      KEY_VALUE value;
      value.key = std::to_string(alarm.alarm_number);
      value.level = alarm.alarm_level;

      auto alarm_by_lang = alarm_helper_.get_alarm_info(alarm.alarm_number, language);
      if (flag_language > 0) {
        language = AlarmHelper::E_LANG::KOREAN;
        alarm_by_lang = alarm_helper_.get_alarm_info(alarm.alarm_number, language);
      }
      value.value = alarm_by_lang->alarm_message;

      alarmList.push_back(value);
    }
  }

  return alarmList;
}

void HmiComm::send_alarm_list(std::vector<KEY_VALUE> alarmList)
{
  bool runcomm = true;
  BYTE buff_num = 0;
  BYTE recv_buff[250] = {0};

  do {
    buff_num = pSerial_.available();
    if (pSerial_.read(recv_buff, buff_num) > 0) {
      int dataByteCnt = (int)recv_buff[5] * 2;

      if (alarmList.size() > 0) {
        for (int i = 0; i < alarmList.size(); i++) {
          response_to_fc3(dataByteCnt, alarmList[i].value);
        }
      } else {
        std::string alarmReset = " ";
        response_to_fc3(dataByteCnt, alarmReset);
      }

      runcomm = false;
    }
  } while (runcomm);
}

void HmiComm::recv_language_set()
{
  bool runcomm = true;
  BYTE buff_num = 0;
  BYTE recv_buff[250] = {0};

  do {
    buff_num = pSerial_.available();
    if (pSerial_.read(recv_buff, buff_num) > 0) {
      flag_language = (int)recv_buff[4] << 8 | (int)recv_buff[5];
      // std::cout << "flag_language: " << flag_language << std::endl;

      response_to_fc5(recv_buff);

      runcomm = false;
    }
  } while (runcomm);
}

bool HmiComm::response_to_fc3(int dataByteCnt, std::string alarmMsg)
{
  // UTF-8 -> UTF-16 변환
  std::wstring_convert<std::codecvt_utf8_utf16<char16_t>, char16_t> converter;
  std::u16string alarmMsg_u16 = converter.from_bytes(alarmMsg);

  BYTE alarmBuf[2 * alarmMsg_u16.length()];
  WORD alarmBuf_u16[alarmMsg_u16.length()];
  std::memcpy(alarmBuf_u16, alarmMsg_u16.data(), alarmMsg_u16.length() * 2);

  for (int k = 0; k < sizeof(alarmBuf); k++) {
    if (k % 2 == 0) {
      alarmBuf[k] = alarmBuf_u16[k / 2] >> 8;            // byte_high
      alarmBuf[k + 1] = alarmBuf_u16[k / 2] & 0xff;      // byte_low
    }
  }

  int sndbuff_num = 5 + dataByteCnt;
  BYTE bySndBuf[sndbuff_num] = {0};

  bySndBuf[0] = SLAVE_ID;
  bySndBuf[1] = FUNC_03;
  bySndBuf[2] = dataByteCnt;

  for (int i = 3; i < 3 + sizeof(alarmBuf); i++) {
    bySndBuf[i] = alarmBuf[i - 3];
  }

  //crc16 계산
  uint16_t crc_2byte;
  crc_2byte = CRC_Cal(bySndBuf, sndbuff_num - 2);

  bySndBuf[sndbuff_num - 2] = crc_2byte & 0xff;              //CRC low
  bySndBuf[sndbuff_num - 1] = crc_2byte >> 8;                //CRC high

  if (pSerial_.write(bySndBuf, sizeof(bySndBuf)) > 0) {return true;} else {return false;}
}

bool HmiComm::response_to_fc5(BYTE * Buf)
{
  if (pSerial_.write(Buf, sizeof(Buf)) > 0) {return true;} else {return false;}
}
}
