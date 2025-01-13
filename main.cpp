#include <wiringPi.h>

// "LED-Pin: wiringPi-Pin 0" entspricht "BCM_GPIO 17".
// Bei einer Initialisierung mit wiringPiSetupSys muss die BCM-Nummerierung verwendet werden.
// Wenn Sie eine andere Pin-Nummer wählen, verwenden Sie die BCM-Nummerierung, und
// aktualisieren Sie außerdem unter "Eigenschaftenseiten" > "Buildereignisse" den Remote-Postbuildereignisbefehl,
// der den GPIO-Export zum Einrichten von wiringPiSetupSys verwendet.
#define	LED	17

int main(void)
{
	wiringPiSetupSys();

	pinMode(LED, OUTPUT);

	while (true)
	{
		digitalWrite(LED, HIGH);  // Ein
		delay(500); // ms
		digitalWrite(LED, LOW);	  // Aus
		delay(500);
	}
	return 0;
}