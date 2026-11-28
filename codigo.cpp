#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// --- IDENTIFICACAO DO NO E PINOS (ESP32) ---
const char* ID_JURADO = "JURADO_C"; // Teste inicial como entrada C (3.3V -> LV level shifter -> HV 5V)
#define PIN_I2C_SDA 21              // SDA padrao da ESP32
#define PIN_I2C_SCL 22              // SCL padrao da ESP32
#define PIN_SAIDA_VOTO 25           // Saida digital 3.3V -> LV do conversor -> HV (5V) entrada C
#define PIN_LED_STATUS 2            // LED onboard comum na ESP32 (ajuste se necessario)

// --- CONFIGURACAO DO SENSOR E VOO ---
Adafruit_BMP280 bmp;

#define PRESSAO_MAR_PADRAO 1013.25 // hPa (usado apenas na formula barometrica)
#define LIMIAR_SUBIDA_INICIAL 3.0  // Ganho minimo (m) em relacao a base para sair de SOLO
#define ALTITUDE_MIN_APOGEU 10.0   // Altitude minima (m) para validar apogeu
#define MARGEM_QUEDA_APOGEU 0.5    // Queda minima (m) entre media antiga e recente
#define AMOSTRAS_HISTORICO 6       // Par: 3 amostras antigas vs 3 amostras recentes

// --- MODO DE TESTE NO SERIAL MONITOR ---
const bool MODO_DEBUG_SERIAL = true;

enum EstadosVoo {
  ESTADO_SOLO = 0,
  ESTADO_SUBIDA,
  ESTADO_APOGEU_DETECTADO
};

EstadosVoo estadoAtual = ESTADO_SOLO;
bool votoConfirmado = false;
bool estadoLed = false;

float altitudeBase = 0.0;
float historicoAltitudes[AMOSTRAS_HISTORICO];
int indiceEscrita = 0;

unsigned long ultimoTempoLeitura = 0;
const unsigned long intervaloLeitura = 50; // 50 ms = 20 Hz
unsigned long ultimoTempoPisca = 0;

float obterAltitudeCronologica(int posCronologica) {
  int indiceReal = (indiceEscrita + posCronologica) % AMOSTRAS_HISTORICO;
  return historicoAltitudes[indiceReal];
}

const char* nomeEstado(EstadosVoo estado) {
  switch (estado) {
    case ESTADO_SOLO: return "SOLO";
    case ESTADO_SUBIDA: return "SUBIDA";
    case ESTADO_APOGEU_DETECTADO: return "APOGEU_DETECTADO";
    default: return "DESCONHECIDO";
  }
}

void definirVoto(bool votar) {
  votoConfirmado = votar;
  digitalWrite(PIN_SAIDA_VOTO, votar ? HIGH : LOW);
}

void setLedStatus(bool ligado) {
  estadoLed = ligado;
  digitalWrite(PIN_LED_STATUS, estadoLed ? HIGH : LOW);
}

void imprimirCabecalhoSerial() {
  Serial.println(F("=========================================================================="));
  Serial.print(F("TESTE "));
  Serial.print(ID_JURADO);
  Serial.println(F(" (BMP280 + ESP32 [SDA 21 / SCL 22] -> LEVEL SHIFTER -> VOTACAO 5V [ENTRADA C])"));
  Serial.println(F("Tempo(ms) | Jurado   | Estado           | AltRel(m) | MedAnt | MedRec | VotoC"));
  Serial.println(F("=========================================================================="));
}

void setup() {
  pinMode(PIN_SAIDA_VOTO, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);

  // Garante saida em LOW antes de qualquer inicializacao
  definirVoto(false);
  setLedStatus(false);

  Serial.begin(115200);
  delay(500);
  Serial.print(F("\nInicializando "));
  Serial.print(ID_JURADO);
  Serial.println(F(" na ESP32..."));

  // Inicializa o barramento I2C nos pinos 21 (SDA) e 22 (SCL)
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  bool sensorOk = bmp.begin(0x76);
  uint8_t enderecoUsado = 0x76;

  if (!sensorOk) {
    Serial.println(F("Aviso: BMP280 nao encontrado em 0x76. Tentando 0x77..."));
    sensorOk = bmp.begin(0x77);
    enderecoUsado = 0x77;
  }

  if (!sensorOk) {
    Serial.println(F("ERRO CRITICO: BMP280 nao encontrado em 0x76 nem 0x77!"));
    definirVoto(false);
    while (1) {
      setLedStatus(!estadoLed);
      delay(200);
    }
  }

  Serial.print(F("BMP280 conectado no endereco I2C 0x"));
  Serial.println(enderecoUsado, HEX);

  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_55);

  Serial.println(F("Calibrando altitude base no solo (20 amostras)..."));
  double somaPressaoPa = 0.0;
  for (int i = 0; i < 20; i++) {
    somaPressaoPa += bmp.readPressure();
    delay(25);
  }

  float pressaoSoloHpa = (somaPressaoPa / 20.0) / 100.0;
  altitudeBase = 44330.0 * (1.0 - pow(pressaoSoloHpa / PRESSAO_MAR_PADRAO, 0.1903));

  for (int i = 0; i < AMOSTRAS_HISTORICO; i++) {
    historicoAltitudes[i] = 0.0;
  }
  indiceEscrita = 0;

  Serial.print(F("Pressao base: "));
  Serial.print(pressaoSoloHpa, 2);
  Serial.print(F(" hPa | Altitude base ref: "));
  Serial.print(altitudeBase, 2);
  Serial.println(F(" m"));
  Serial.println(F("VotoC inicial: LOW (0)"));
  imprimirCabecalhoSerial();
}

void loop() {
  unsigned long tempoAtual = millis();

  if (tempoAtual - ultimoTempoLeitura >= intervaloLeitura) {
    ultimoTempoLeitura = tempoAtual;

    float pressaoHpa = bmp.readPressure() / 100.0;
    float altitudeAbsoluta = 44330.0 * (1.0 - pow(pressaoHpa / PRESSAO_MAR_PADRAO, 0.1903));
    float altitudeRelativa = altitudeAbsoluta - altitudeBase;

    historicoAltitudes[indiceEscrita] = altitudeRelativa;
    indiceEscrita = (indiceEscrita + 1) % AMOSTRAS_HISTORICO;

    int metade = AMOSTRAS_HISTORICO / 2;
    float somaAntiga = 0.0;
    float somaRecente = 0.0;

    for (int i = 0; i < metade; i++) {
      somaAntiga += obterAltitudeCronologica(i);
    }
    for (int i = metade; i < AMOSTRAS_HISTORICO; i++) {
      somaRecente += obterAltitudeCronologica(i);
    }

    float mediaAntiga = somaAntiga / metade;
    float mediaRecente = somaRecente / (AMOSTRAS_HISTORICO - metade);

    if (estadoAtual == ESTADO_SOLO) {
      definirVoto(false);
      setLedStatus(false);

      if (altitudeRelativa > LIMIAR_SUBIDA_INICIAL) {
        estadoAtual = ESTADO_SUBIDA;
        setLedStatus(true);
        Serial.print(F(">>> ["));
        Serial.print(ID_JURADO);
        Serial.println(F("] TRANSICAO: SOLO -> SUBIDA <<<"));
      }
    }
    else if (estadoAtual == ESTADO_SUBIDA) {
      bool quedaConfirmada = (mediaAntiga - mediaRecente) >= MARGEM_QUEDA_APOGEU;
      bool acimaDoMinimo = (altitudeRelativa >= ALTITUDE_MIN_APOGEU);

      if (quedaConfirmada && acimaDoMinimo) {
        estadoAtual = ESTADO_APOGEU_DETECTADO;
        definirVoto(true);
        Serial.print(F(">>> ["));
        Serial.print(ID_JURADO);
        Serial.println(F("] TRANSICAO: SUBIDA -> APOGEU_DETECTADO (VotoC = HIGH) <<<"));
      }
    }
    else if (estadoAtual == ESTADO_APOGEU_DETECTADO) {
      definirVoto(true);

      if (tempoAtual - ultimoTempoPisca >= 150) {
        ultimoTempoPisca = tempoAtual;
        setLedStatus(!estadoLed);
      }
    }

    if (MODO_DEBUG_SERIAL) {
      Serial.print(tempoAtual);
      Serial.print(F(" | "));
      Serial.print(ID_JURADO);
      Serial.print(F(" | "));
      Serial.print(nomeEstado(estadoAtual));
      Serial.print(F(" | "));
      Serial.print(altitudeRelativa, 2);
      Serial.print(F(" | "));
      Serial.print(mediaAntiga, 2);
      Serial.print(F(" | "));
      Serial.print(mediaRecente, 2);
      Serial.print(F(" | "));
      Serial.println(votoConfirmado ? F("HIGH") : F("LOW"));
    }
  }
}
