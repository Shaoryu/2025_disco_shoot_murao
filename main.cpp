/*確認事項
　UnbufferedSerialはコメントアウトされているか？シリアル読み込みはどうか？(rbms.cppも)
　can_motorはPB_5,6か？
　max_degは-か+か
　data以下のnumber_accは正しいか？
　ソフトリミットの範囲は正しいか？
  吸引ポンプのは正しいか？
*/
#include "mbed.h"
#include "rbms.h"
#include <cstdio>
UnbufferedSerial pc(USBTX, USBRX, 921600);


CAN can_motor(PB_5, PB_6);  // CAN RX, TX
CAN can_signal(PA_11,PA_12,1000000);

CANMessage msg_controller,msg_limit,msg_can1;

static constexpr int motor_num = 4;
static constexpr int max_speed = 300;
static constexpr short receivecan_id = 0x05;//コントローラの信号ID正直命名ミスった枠その１
static constexpr short limitcan_id = 0x301;//pc用
bool limit_status[6]={1};//リミット
bool is_pump_start=false;
char pre_pump=0;

int motor_output[motor_num] = {0};
int setpoint[motor_num] = {0};              // 指令値（角度 or 速度）
short offset_deg;


MotorType mtype[motor_num] = {
    M2006,
    M2006,
    M2006,
    M3508
};
ControlMode control_mode[motor_num] = {SPEED};  // 制御モード配列（初期値: すべて ANGLE）

EventQueue queue_motor(32 * EVENTS_EVENT_SIZE);
EventQueue queue_control(32 * EVENTS_EVENT_SIZE);
rbms motor_controller(can_motor, mtype, motor_num);
Thread can_thread_motor,can_thread_control;
Thread control_thread;
Thread send_thread;


//DigitalOut ledmain(LED1);
DigitalOut leds[4]{
    DigitalOut(PC_0),
    DigitalOut(PC_1),
    DigitalOut(LED1),
    DigitalOut(PC_3)
};

auto& now_ang = motor_controller.getSumDeg();

void can_receive_loop_motor() {
    motor_controller.can_read();
    // can_signal.read(msg_controller);
}

void can_receive_loop_control() {
        //motor_controller.can_read();
        if(can_signal.read(msg_can1))
            leds[1] =! leds[1];
        if(msg_can1.id==receivecan_id){
            leds[2] =! leds[2];
            msg_controller = msg_can1;
        }
        else if(msg_can1.id==0x301 ||msg_can1.id==0x302){
            msg_limit = msg_can1;
        }
}

void control_loop() {
    motor_controller.unified_control(control_mode, setpoint, motor_output,max_speed);
}

void send_loop() {
    while (true) {
        motor_controller.rbms_send(motor_output);
        ThisThread::sleep_for(1ms);
    }
}


void onReceive_motor() {
    queue_motor.call(can_receive_loop_motor);
}

void onReceive_control() {
    queue_control.call(can_receive_loop_control);
}


//void limitswitch();
PwmOut pump(PB_9);
void switch_pump(){
    is_pump_start=false;
    switch (msg_controller.data[6]) {
        case 0:pump.pulsewidth_us(1000);break;
        case 1: pump.pulsewidth_us(1500);break;
        case 2: pump.pulsewidth_us(1600);break;
        case 3: pump.pulsewidth_us(1700);break;
        case 0x11: pump.pulsewidth_us(1100);break;
        case 0x12: pump.pulsewidth_us(1200);break;
        case 0x13: pump.pulsewidth_us(1300);break;
        case 0x14: pump.pulsewidth_us(1400);break;
        case 0x15: pump.pulsewidth_us(1500);break;
        case 0x16: pump.pulsewidth_us(1600);break;
        case 0x17: pump.pulsewidth_us(1700);break;
        case 0x18: pump.pulsewidth_us(1800);break;
        case 0x19: pump.pulsewidth_us(1900);break;
        case 0x1a: pump.pulsewidth_us(2000);break;
        default:pump.pulsewidth_us(1000);
    }
}


int main() {
    //can_motor.reset();
    leds[0] = true;
    leds[2] = true;
    pump.pulsewidth_us(1000);
    can_motor.frequency(1000000);
    bool pre_flag_limit=false;
    bool flag_limit1=false;
    bool flag_limit2=false;
    //printf("Unified control start...\r\n");
    can_thread_motor.start(callback(&queue_motor, &EventQueue::dispatch_forever));
    can_motor.attach(onReceive_motor, CAN::RxIrq);
    can_thread_control.start(callback(&queue_control, &EventQueue::dispatch_forever));
    can_signal.attach(onReceive_control, CAN::RxIrq);
    control_thread.start(control_loop);
    send_thread.start(send_loop);
    ThisThread::sleep_for(2ms);
    offset_deg = now_ang[0] /36;
    control_mode[0]=SPEED;
    setpoint[0]=0;
    msg_controller.id=limitcan_id;
    msg_controller.len=8;
    for(int i=0;i<8;i++)msg_controller.data[i]=0;
    while (true) {
        char key=0;
        if(pc.readable()){
            pc.read(&key,1);
            if(key>='0'&&key<='9'){
                msg_controller.data[6]=(key-'0')+0x10;
                switch_pump();
            }
            switch(key){
                case ' ':
                    msg_controller.data[6]=0x13;
                    switch_pump();
                    break;
                case 'q':
                    pump.pulsewidth_us(1000);
                    break;
                case 'e':
                    pump.pulsewidth_us(1300);
                    break;
                case 'w':
                    for(auto &i:setpoint)i=50;
                    break;
                case 's':
                    for(auto &i:setpoint)i=-50;
                    break;
                default:
                    for(auto &i:setpoint)i=0;
                    break;
            }
        }
        printf("%d,%d\r\n",pump.read_pulsewidth_us(),setpoint[0]);
        leds[3] = !leds[3];
    }
}//