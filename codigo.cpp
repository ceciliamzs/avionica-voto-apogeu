/*
  JURADO C - Deteccao de apogeu com BMP280 + ESP32

  Ligacoes:
    BMP280 VCC -> 3.3V | GND -> GND | SDA -> GPIO 21 | SCL -> GPIO 22
    GPIO 25 -> LV do level shifter -> HV (5V) -> entrada C da votacao
    IMPORTANTE (hardware): coloque um resistor de pull-down de 10k entre a linha do
    voto e o GND. Durante o reset/boot da ESP32 o GPIO fica flutuando e a entrada
    de 5V pode ler um voto falso.

  Logica de voo:
    SOLO     -> base de altitude acompanha a deriva lenta da pressao. Sai para SUBIDA
                quando (apos o armamento) a altitude relativa passa de
                LIMIAR_SUBIDA_INICIAL E a velocidade vertical passa de VEL_DECOLAGEM.
    SUBIDA   -> detecta apogeu por queda da altitude (media antiga - media recente),
                confirmada por varias leituras seguidas. Se a altitude voltar para
                perto de 0 por alguns segundos (falsa subida), retorna para SOLO.
    APOGEU   -> voto fica travado em HIGH.

  LED: pisca lento = desarmado (primeiros segundos) | apagado = armado em SOLO |
       aceso = SUBIDA | pisca rapido = APOGEU detectado.
*/

#include <Arduino.h>
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

#define PRESSAO_MAR_PADRAO 1013.25   // hPa (usado apenas na formula barometrica)
#define LIMIAR_SUBIDA_INICIAL 3.0    // Ganho minimo (m) em relacao a base para sair de SOLO
#define VEL_DECOLAGEM 5.0            // Velocidade vertical minima (m/s) para sair de SOLO
#define ALTITUDE_MIN_APOGEU 10.0     // Altitude minima (m) para validar apogeu
#define MARGEM_QUEDA_APOGEU 0.5      // Queda minima (m) entre media antiga e recente
#define AMOSTRAS_HISTORICO 6         // Par: 3 amostras antigas vs 3 amostras recentes
#define CONFIRMACOES_APOGEU 3        // Leituras seguidas com queda confirmada para votar

#define TEMPO_ARMAMENTO_MS 15000     // Ignora decolagem nos primeiros 15 s apos ligar
#define ALFA_BASE_ARMANDO 0.1        // Velocidade de ajuste da base enquanto desarmado
#define ALFA_BASE_ARMADO 0.005       // Velocidade de ajuste da base depois de armado (~10 s)

#define ALTITUDE_RETORNO_SOLO 1.0    // Abaixo disso (m) em SUBIDA conta como "voltou ao solo"
#define TEMPO_RETORNO_SOLO_MS 3000   // ... por este tempo, retorna para SOLO

#define PRESSAO_MIN_HPA 300.0        // Faixa plausivel do BMP280; fora dela a leitura e descartada
#define PRESSAO_MAX_HPA 1100.0

// --- MODO DE TESTE NO SERIAL MONITOR ---
// Em voo real, coloque false.
const bool MODO_DEBUG_SERIAL = true;

enum EstadosVoo {
  ESTADO_SOLO = 0,
  ESTADO_SUBIDA,
  ESTADO_APOGEU_DETECTADO
};

EstadosVoo estadoAtual = ESTADO_SOLO;
bool votoConfirmado = false;
bool estadoLed = false;

float altitudeBase = 0.0;          // altitude absoluta de referencia (acompanha a deriva em SOLO)
float historicoAltitudes[AMOSTRAS_HISTORICO];
int indiceEscrita = 0;

int contadorQueda = 0;             // confirmacoes consecutivas de queda
bool abaixoDoRetorno = false;      // altitude baixa em SUBIDA (possivel falsa subida)
unsigned long tempoAbaixoDoRetorno = 0;
unsigned long leiturasInvalidas = 0;

unsigned long ultimoTempoLeitura = 0;
const unsigned long intervaloLeitura = 50; // 50 ms = 20 Hz
unsigned long ultimoTempoPisca = 0;

float obterAltitudeCronologica(int posCronologica) {
  // indiceEscrita aponta para a amostra mais antiga; pos 0 = mais antiga
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

float calcularAltitude(float pressaoHpa) {
  return 44330.0 * (1.0 - pow(pressaoHpa / PRESSAO_MAR_PADRAO, 0.1903));
}

bool pressaoValida(float pressaoHpa) {
  return !isnan(pressaoHpa) && pressaoHpa >= PRESSAO_MIN_HPA && pressaoHpa <= PRESSAO_MAX_HPA;
}

// Erro grave: voto em LOW e LED piscando para sempre
void erroCritico(const __FlashStringHelper* mensagem) {
  Serial.println(mensagem);
  definirVoto(false);
  while (true) {
    setLedStatus(!estadoLed);
    delay(200);
  }
}

// LED indica o estado, sem depender de leitura valida do sensor
void atualizarLed(unsigned long agora) {
  switch (estadoAtual) {
    case ESTADO_SOLO:
      if (agora < TEMPO_ARMAMENTO_MS) {          // piscando devagar = ainda desarmado
        if (agora - ultimoTempoPisca >= 500) {
          ultimoTempoPisca = agora;
          setLedStatus(!estadoLed);
        }
      } else {
        setLedStatus(false);
      }
      break;
    case ESTADO_SUBIDA:
      setLedStatus(true);
      break;
    case ESTADO_APOGEU_DETECTADO:
      if (agora - ultimoTempoPisca >= 150) {
        ultimoTempoPisca = agora;
        setLedStatus(!estadoLed);
      }
      break;
  }
}

void imprimirCabecalhoSerial() {
  Serial.println(F("=================================================================================="));
  Serial.print(F("TESTE "));
  Serial.print(ID_JURADO);
  Serial.println(F(" (BMP280 + ESP32 [SDA 21 / SCL 22] -> LEVEL SHIFTER -> VOTACAO 5V [ENTRADA C])"));
  Serial.println(F("Tempo(ms) | Jurado   | Estado           | AltRel(m) | MedAnt | MedRec | Vel(m/s) | VotoC"));
  Serial.println(F("=================================================================================="));
}

void setup() {
  // Garante saida em LOW o mais cedo possivel
  digitalWrite(PIN_SAIDA_VOTO, LOW);
  pinMode(PIN_SAIDA_VOTO, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
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
    erroCritico(F("ERRO CRITICO: BMP280 nao encontrado em 0x76 nem 0x77!"));
  }

  Serial.print(F("BMP280 conectado no endereco I2C 0x"));
  Serial.println(enderecoUsado, HEX);

  // Standby de 1 ms: o sensor gera uma medida nova a cada ~40 ms (~25 Hz),
  // entao cada leitura do loop (a cada 50 ms) e uma medida nova, sem repeticao.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X4,
                  Adafruit_BMP280::STANDBY_MS_1);

  // Espera o sensor e o filtro interno estabilizarem antes de calibrar
  Serial.println(F("Aguardando estabilizacao do sensor (1,5 s)..."));
  delay(1500);

  Serial.println(F("Calibrando altitude base no solo (30 amostras)..."));
  double somaPressaoPa = 0.0;
  int amostrasValidas = 0;
  for (int i = 0; i < 30; i++) {
    float p = bmp.readPressure();
    if (pressaoValida(p / 100.0)) {
      somaPressaoPa += p;
      amostrasValidas++;
    }
    delay(40);
  }

  if (amostrasValidas < 20) {
    erroCritico(F("ERRO CRITICO: leituras de pressao invalidas na calibracao!"));
  }

  float pressaoSoloHpa = (somaPressaoPa / amostrasValidas) / 100.0;
  altitudeBase = calcularAltitude(pressaoSoloHpa);

  for (int i = 0; i < AMOSTRAS_HISTORICO; i++) {
    historicoAltitudes[i] = 0.0;
  }
  indiceEscrita = 0;

  Serial.print(F("Pressao base: "));
  Serial.print(pressaoSoloHpa, 2);
  Serial.print(F(" hPa | Altitude base ref: "));
  Serial.print(altitudeBase, 2);
  Serial.println(F(" m"));
  Serial.print(F("Armamento em "));
  Serial.print(TEMPO_ARMAMENTO_MS / 1000);
  Serial.println(F(" s apos ligar."));
  Serial.println(F("VotoC inicial: LOW (0)"));
  imprimirCabecalhoSerial();
}

void loop() {
  unsigned long tempoAtual = millis();

  atualizarLed(tempoAtual);

  if (tempoAtual - ultimoTempoLeitura < intervaloLeitura) return;
  ultimoTempoLeitura = tempoAtual;

  // --- Leitura com checagem de validade ---
  float pressaoHpa = bmp.readPressure() / 100.0;
  if (!pressaoValida(pressaoHpa)) {
    leiturasInvalidas++;
    if (MODO_DEBUG_SERIAL && (leiturasInvalidas % 20 == 1)) {
      Serial.println(F("AVISO: leitura de pressao invalida, ignorada."));
    }
    return;   // nao atualiza historico nem estado com dado ruim
  }
  leiturasInvalidas = 0;

  float altitudeAbsoluta = calcularAltitude(pressaoHpa);
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

  // Velocidade vertical estimada: diferenca entre as medias / tempo entre os centros das janelas
  float velEstimada = (mediaRecente - mediaAntiga) / (metade * intervaloLeitura / 1000.0);

  if (estadoAtual == ESTADO_SOLO) {
    definirVoto(false);

    bool armado = (tempoAtual >= TEMPO_ARMAMENTO_MS);

    if (armado &&
        altitudeRelativa > LIMIAR_SUBIDA_INICIAL &&
        velEstimada > VEL_DECOLAGEM) {
      estadoAtual = ESTADO_SUBIDA;
      contadorQueda = 0;
      abaixoDoRetorno = false;
      Serial.print(F(">>> ["));
      Serial.print(ID_JURADO);
      Serial.println(F("] TRANSICAO: SOLO -> SUBIDA <<<"));
    } else {
      // Base acompanha a deriva lenta da pressao (rapido ao ligar, devagar depois)
      float alfa = armado ? ALFA_BASE_ARMADO : ALFA_BASE_ARMANDO;
      altitudeBase += alfa * (altitudeAbsoluta - altitudeBase);
    }
  }
  else if (estadoAtual == ESTADO_SUBIDA) {
    bool quedaConfirmada = (mediaAntiga - mediaRecente) >= MARGEM_QUEDA_APOGEU;
    bool acimaDoMinimo = (altitudeRelativa >= ALTITUDE_MIN_APOGEU);

    // Debounce: exige varias leituras seguidas com queda confirmada
    if (quedaConfirmada && acimaDoMinimo) {
      contadorQueda++;
    } else {
      contadorQueda = 0;
    }

    if (contadorQueda >= CONFIRMACOES_APOGEU) {
      estadoAtual = ESTADO_APOGEU_DETECTADO;
      definirVoto(true);
      Serial.print(F(">>> ["));
      Serial.print(ID_JURADO);
      Serial.println(F("] TRANSICAO: SUBIDA -> APOGEU_DETECTADO (VotoC = HIGH) <<<"));
    }
    else if (altitudeRelativa < ALTITUDE_RETORNO_SOLO) {
      // Falsa subida: altitude voltou para perto de zero e ficou la
      if (!abaixoDoRetorno) {
        abaixoDoRetorno = true;
        tempoAbaixoDoRetorno = tempoAtual;
      } else if (tempoAtual - tempoAbaixoDoRetorno >= TEMPO_RETORNO_SOLO_MS) {
        estadoAtual = ESTADO_SOLO;
        contadorQueda = 0;
        abaixoDoRetorno = false;
        Serial.print(F(">>> ["));
        Serial.print(ID_JURADO);
        Serial.println(F("] TRANSICAO: SUBIDA -> SOLO (falsa subida) <<<"));
      }
    } else {
      abaixoDoRetorno = false;
    }
  }
  else if (estadoAtual == ESTADO_APOGEU_DETECTADO) {
    definirVoto(true);   // voto travado em HIGH
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
    Serial.print(velEstimada, 2);
    Serial.print(F(" | "));
    Serial.println(votoConfirmado ? F("HIGH") : F("LOW"));
  }
}
