#include <HX711_ADC.h>

const int HX711_dout_1 = 27;
const int HX711_sck_1 = 26;

const int HX711_dout_2 = 25; // Ganti dengan pin dout sensor berat kedua
const int HX711_sck_2 = 33;  // Ganti dengan pin sck sensor berat kedua

HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1);
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2);

void setup()
{
    Serial.begin(9600);

    LoadCell_1.begin();
    LoadCell_1.start(2000);
    LoadCell_1.setCalFactor(106.26);

    LoadCell_2.begin();
    LoadCell_2.start(2000);
    LoadCell_2.setCalFactor(106.26);
}

void loop()
{
    LoadCell_1.update();
    LoadCell_2.update();

    float weight_1 = LoadCell_1.getData();
    float weight_2 = LoadCell_2.getData();

    if (weight_1 < 0)
    {
        weight_1 = 0;
    }

    if (weight_2 < 0)
    {
        weight_2 = 0;
    }

    Serial.print("Weight Sensor 1 [g]: ");
    Serial.println(weight_1);

    Serial.print("Weight Sensor 2 [g]: ");
    Serial.println(weight_2);

    delay(1000); // Ganti dengan interval yang sesuai dengan kebutuhan Anda
}
