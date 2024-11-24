#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "dynamixel.h"
#include <vector>
#include <ESP32Servo.h>

Servo servoSG90;
std::vector<int> target_floor_queue; //Antrian tidak terbatas

void initPins();
void calibrateElevator();
void checkControlPanelButtons();
void checkIRSensor();
void updateLCD();
void controlElevator();
void handleOverloadWarning();
void openDoor();
void closeDoor();
void setElevatorCabinHeight(int target_floor);
void moveElevator(int direction);
int getUltrasonicDistance();
void addToQueue(int floor, bool in_direction);

// Pin Definitions
#define CONTROL_FLOOR1_BTN 27
#define CONTROL_FLOOR2_BTN 34
#define CONTROL_FLOOR3_BTN 35
#define LCD_SCL_PIN 21
#define IR_SENSOR_PIN 12
#define ECHO_PIN 19
#define BUZZER_PIN 32
#define LCD_SDA_PIN 22
#define TRIG_PIN 23
#define SERVO_DOOR_ID 13 // Pin untuk servo SG90 (Pintu)

#define FLOOR1_HEIGHT 530
#define FLOOR2_HEIGHT 215
#define FLOOR3_HEIGHT 0
#define HEIGHT_TOLERANCE 5 // ±5 mm tolerance

// Constants
#define DOOR_OPEN_POS 120
#define MAX_OCCUPANCY 5         // Maksimum kapasitas penumpang
#define BUZZER_FREQUENCY 2000   // Frekuensi buzzer 2KHz
#define BUZZER_DURATION 500     // Durasi buzzer 500ms

#define SPEED_UP 300     // Kecepatan untuk naik (CCW)
#define SPEED_DOWN 1324  // Kecepatan untuk turun
#define SPEED_STOP 0     // Kecepatan untuk berhenti
// Elevator control variables
int current_floor = 0; 
bool is_calibrated = false;
int queue_front = 0, queue_rear = 0;
bool door_open = false;
unsigned long door_open_time = 0;
unsigned long ir_detection_time = 0;
unsigned long door_close_delay = 4000; // 4 detik
int object_count = 0;
bool control_panel_active = false;
int elevator_direction = 0; // 0 = stopped, 1 = up, -1 = down

// LCD Instance (Address 0x27, 16 chars, 2 lines)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
    Serial.begin(115200);
    initPins();
    initDynamixel();
    calibrateElevator();
    
    // Hubungkan servo ke pin yang didefinisikan
    servoSG90.attach(SERVO_DOOR_ID);
    servoSG90.write(0);
    wheelMode(XL320_ID[0], 300);
    delay(200);
    wheelMode(XL320_ID[1], 300);
    setLED(XL320_ID[0],2);
    setLED(XL320_ID[1],2);

    // Inisialisasi LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();

    // Menampilkan pesan awal pada LCD
    lcd.setCursor(0, 0); // Baris pertama, kolom pertama
    lcd.print("Selamat Datang!");
    lcd.setCursor(0, 1); // Baris kedua
    lcd.print("Sistem Lift Aktif");
}

void loop() {
    checkControlPanelButtons();        // Periksa tombol panel kontrol
    checkIRSensor();                   // Periksa sensor IR
    updateLCD();                       // Perbarui tampilan LCD
    controlElevator();                 // Kontrol elevator
    handleOverloadWarning();           // Tangani peringatan kelebihan beban
    // Periksa dan kontrol pintu
    if (door_open && millis() - ir_detection_time > door_close_delay) {
        closeDoor();
    }
}

void initPins() {
    pinMode(CONTROL_FLOOR1_BTN, INPUT_PULLUP);
    pinMode(CONTROL_FLOOR2_BTN, INPUT_PULLUP);
    pinMode(CONTROL_FLOOR3_BTN, INPUT_PULLUP);
    pinMode(IR_SENSOR_PIN, INPUT);
    pinMode(SERVO_DOOR_ID, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(XL320_DIR_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
}

void calibrateElevator() {
    // Cek tombol kontrol lantai yang terakhir ditekan
    if (digitalRead(CONTROL_FLOOR1_BTN) == LOW) {
        current_floor = 1;
    } else if (digitalRead(CONTROL_FLOOR2_BTN) == LOW) {
        current_floor = 2;
    } else if (digitalRead(CONTROL_FLOOR3_BTN) == LOW) {
        current_floor = 3;
    }
    // Set ketinggian kabin elevator berdasarkan lantai terakhir
    setElevatorCabinHeight(current_floor);
    is_calibrated = true;
}

void setElevatorCabinHeight(int target_floor) {
    int target_height = 0;
    switch (target_floor) {
        case 1:
            target_height = FLOOR1_HEIGHT;
            break;
        case 2:
            target_height = FLOOR2_HEIGHT;
            break;
        case 3:
            target_height = FLOOR3_HEIGHT;
            break;
        default:
            target_height = FLOOR1_HEIGHT;
            break;
    }

    int current_height = getUltrasonicDistance();
    int height_diff = target_height - current_height;

  // Tentukan arah gerakan berdasarkan perbedaan ketinggian
    if (abs(height_diff) > HEIGHT_TOLERANCE) {
        if (height_diff > 0) {
            // Perlu naik
            moveElevator(1);
            // Tunggu sampai mencapai ketinggian yang diinginkan
            while (abs(getUltrasonicDistance() - target_height) > HEIGHT_TOLERANCE) {
                delay(10);  // Berikan sedikit delay untuk mengurangi beban CPU
                // Periksa sensor ultrasonic secara berkala
                if (abs(getUltrasonicDistance() - target_height) <= HEIGHT_TOLERANCE) {
                    break;
                }
            }
        } else {
            // Perlu turun
            moveElevator(-1);
            // Tunggu sampai mencapai ketinggian yang diinginkan
            while (abs(getUltrasonicDistance() - target_height) > HEIGHT_TOLERANCE) {
                delay(10);
                if (abs(getUltrasonicDistance() - target_height) <= HEIGHT_TOLERANCE) {
                    break;
                }
            }
        }
        // Hentikan elevator ketika sudah mencapai target
        moveElevator(0);
    }
}

// Fungsi untuk mengontrol gerakan elevator
void moveElevator(int direction) {
    switch(direction) {
        case 1: // Naik
            wheelMode(XL320_ID[0], SPEED_UP);    // Motor kiri bergerak CCW
            wheelMode(XL320_ID[1], SPEED_DOWN);   // Motor kanan bergerak CW
            break;
            
        case -1: // Turun
            wheelMode(XL320_ID[0], SPEED_DOWN);  // Motor kiri bergerak CW
            wheelMode(XL320_ID[1], SPEED_UP); // Motor kanan bergerak CCW
            break;
            
        case 0: // Berhenti
            wheelMode(XL320_ID[0], SPEED_STOP);
            wheelMode(XL320_ID[1], SPEED_STOP);
            break;
    }
}

int getUltrasonicDistance() {
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    return duration * 0.34 / 2;
}

void checkControlPanelButtons() {
    if (digitalRead(CONTROL_FLOOR1_BTN) == LOW) {
        control_panel_active = true;
        addToQueue(1, elevator_direction >= 0);
    } else if (digitalRead(CONTROL_FLOOR2_BTN) == LOW) {
        control_panel_active = true;
        addToQueue(2, elevator_direction >= 0);
    } else if (digitalRead(CONTROL_FLOOR3_BTN) == LOW) {
        control_panel_active = true;
        addToQueue(3, elevator_direction <= 0);
    } else {
        control_panel_active = false;
    }
}

void addToQueue(int floor, bool in_direction) {
   if (in_direction) {
        // Menambahkan permintaan ke depan antrian
        target_floor_queue.insert(target_floor_queue.begin(), floor);
    } else {
        // Menambahkan permintaan ke belakang antrian
        target_floor_queue.push_back(floor);
    }
}

int getNextFloor() {
     if (!target_floor_queue.empty()) {
        int next_floor = target_floor_queue.front();
        target_floor_queue.erase(target_floor_queue.begin());
        return next_floor;
    }
    return -1; // Antrian kosong
}

void checkIRSensor() {
    if (digitalRead(IR_SENSOR_PIN) == LOW) {
        ir_detection_time = millis();
        object_count++;
        
        // Aktifkan buzzer jika jumlah objek melebihi kapasitas maksimum
        if (object_count > MAX_OCCUPANCY) {
            tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
        }        
        openDoor();
    } else if (object_count > 0 && millis() - ir_detection_time > door_close_delay) {
        object_count--;
        if (object_count == 0) {
            closeDoor();
        }
    }
}

void openDoor() {
 if (!door_open) {
      // Buka pintu dengan servo SG90
        servoSG90.write(DOOR_OPEN_POS); // Buka pintu ke posisi tertentu
        door_open = true;
        door_open_time = millis();
    }
}

void closeDoor() {
   if (door_open && millis() - ir_detection_time > door_close_delay) {
        // Tutup pintu dengan servo SG90 setelah 4 detik tanpa aktivitas
        servoSG90.write(0); // Kembalikan servo ke posisi tertutup
        door_open = false;
    }
}

// Fungsi untuk memperbarui tampilan LCD
void updateLCD() {
    Serial.println("Updating LCD...");
    lcd.setCursor(0, 0);
    lcd.print("Lantai Saat Ini: ");
    lcd.print(current_floor);    
    lcd.setCursor(0, 1);
    lcd.print("Antrian: ");
    
    // Tampilkan sesuai batas lebar LCD
    int display_limit = min(target_floor_queue.size(), (size_t)8); // Sesuaikan dengan lebar LCD
    for (int i = 0; i < display_limit; i++) {
        lcd.print(target_floor_queue[i]);
        lcd.print(" ");
    }
    
    if (target_floor_queue.size() > display_limit) {
        lcd.print("...");
    }
    
    // Tampilkan jumlah objek di dalam lift
    lcd.setCursor(0, 2); // Baris tambahan di LCD
    lcd.print("Jumlah objek: ");
    lcd.print(object_count);

    // Tampilkan peringatan jika melebihi kapasitas
    if (object_count > MAX_OCCUPANCY) {
        lcd.setCursor(0, 0);
        lcd.print("PERINGATAN:KELEBIHAN BEBAN!");
    }
}

void controlElevator() {
    if (is_calibrated) {
        int next_floor = getNextFloor();
        if (next_floor != -1 && next_floor != current_floor) {
            // Pastikan pintu tertutup sebelum bergerak
            if (door_open) {
                closeDoor();
                delay(1000); // Tunggu pintu benar-benar tertutup
            }

            // Tentukan arah gerakan berdasarkan tujuan berikutnya
            if (next_floor > current_floor) {
                elevator_direction = 1; // Bergerak naik
            } else {
                elevator_direction = -1; // Bergerak turun
            }

            // Aktifkan LED pada servo sebagai indikator gerakan
            setLED(XL320_ID[0], elevator_direction > 0 ? 2 : 4);  // Hijau untuk naik, Kuning untuk turun
            setLED(XL320_ID[1], elevator_direction > 0 ? 2 : 4);  // Hijau untuk naik, Kuning untuk turun

            // Gerakkan elevator ke lantai tujuan
            setElevatorCabinHeight(next_floor);
            
            // Matikan LED setelah sampai
            setLED(XL320_ID[0], 1);  // Putih saat berhenti
            setLED(XL320_ID[1], 1);
            
            current_floor = next_floor;
            
            // Buka pintu setelah sampai di lantai tujuan
            openDoor();
        }
    }
}

// Fungsi untuk menangani peringatan kelebihan beban
void handleOverloadWarning() {
    if (object_count > MAX_OCCUPANCY) {
        // Pola peringatan berkelanjutan
        static unsigned long lastBuzzerTime = 0;
        const unsigned long BUZZER_INTERVAL = 2000; // Interval 2 detik antar peringatan
        
        if (millis() - lastBuzzerTime >= BUZZER_INTERVAL) {
            tone(BUZZER_PIN, BUZZER_FREQUENCY, BUZZER_DURATION);
            lastBuzzerTime = millis();
        }
    }
}