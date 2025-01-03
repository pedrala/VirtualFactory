#define PIN_ENA   8  // ENA 핀 정의
#define PIN_DIR   9  // DIR 핀 정의
#define PIN_PUL   10 // PUL 핀 정의

typedef enum _CONVEYOR_STATE {Conveyor_Ready, Conveyor_Run} CONVEYOR_STATE; // 컨베이어 상태를 나타내는 열거형 정의

unsigned long recv_cnt = 0; // 수신된 숫자를 저장하는 변수
unsigned long time_p = 0;   // 이전 시간 기록을 위한 변수
unsigned long step_count  = 0; // 스텝 수를 저장하는 변수
CONVEYOR_STATE state = Conveyor_Ready; // 초기 상태를 Conveyor_Ready로 설정

void setup() {
  Serial.begin(115200);  // 시리얼 통신 속도 설정
  Serial.write('s');     // 's' 문자 전송 (시작 신호)

  pinMode(PIN_ENA, OUTPUT); // ENA 핀을 출력으로 설정
  pinMode(PIN_DIR, OUTPUT); // DIR 핀을 출력으로 설정
  pinMode(PIN_PUL, OUTPUT); // PUL 핀을 출력으로 설정

  digitalWrite(PIN_ENA, LOW); // ENA 핀 초기 상태를 LOW로 설정
  digitalWrite(PIN_DIR, LOW); // DIR 핀 초기 상태를 LOW로 설정
}

void loop() {
  unsigned long time_c = millis(); // 현재 시간(밀리초) 읽기

  // 시리얼 데이터가 수신되었는지 확인
  if (Serial.available() > 0) {
    int incommingByte = Serial.read(); // 수신된 데이터 읽기

    // 수신된 문자가 숫자(0~9)인지 확인
    if (incommingByte >= '0' && incommingByte <= '9') {
      recv_cnt = recv_cnt * 10 + (incommingByte - '0'); // 기존 값에 새 숫자 추가

      // 최대값 초과 시 초기화
      if (recv_cnt > RECV_MAX_COUNT) {
        recv_cnt = 0;
      }
    } else { 
      // 숫자가 아닌 문자가 수신되었을 때 처리
      if (recv_cnt > 0) {
        state = Conveyor_Run;  // 상태를 Conveyor_Run으로 변경
        step_count = recv_cnt; // 스텝 수를 업데이트
        recv_cnt = 0;          // 수신 카운트 초기화
      }
    }
  }

  step_run(time_c); // step_run 함수 호출
}
