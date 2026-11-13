#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// definição dos pinos
#define PIN_SAIDA_VOTO 8        // Pino digital que vai para o circuito de votação/MOSFET
#define PIN_LED_STATUS 13       // LED onboard para indicação visual

// instância do sensor barométrico
Adafruit_BMP280 bmp;

// constantes de calibração e voo
#define PRESSAO_MAR_PADRAO 1013.25 // hPa (será calibrado no solo)
#define LIMIAR_SUBIDA_INICIAL 3.0  // Altitude mínima (metros) para considerar que o foguete decolou
#define AMOSTRAS_HISTORICO 5       // Número de pontos para suavização e tendência

// variáveis de controle de estado
enum EstadosVoo {
  ESTADO_SOLO = 0,
  ESTADO_SUBIDA,
  ESTADO_APOGEU_DETECTADO
};
EstadosVoo estadoAtual = ESTADO_SOLO;

// variáveis de altitude e tempo
float altitudeBase = 0.0;
float historicoAltitudes[AMOSTRAS_HISTORICO];
int indiceHistorico = 0;
unsigned long ultimoTempoLeitura = 0;
const unsigned long intervaloLeitura = 50; // leitura a cada 50ms (20 Hz)

void setup() {
  pinMode(PIN_SAIDA_VOTO, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_SAIDA_VOTO, LOW); // garante que inicia desarmado/sem votar
  digitalWrite(PIN_LED_STATUS, LOW);

  Serial.begin(9600);

  // inicializa o barômetro (endereço I2C padrão 0x76 ou 0x77)
  if (!bmp.begin(0x76)) {
    Serial.println(F("Erro ao encontrar o sensor BMP280! Verifique as conexões."));
    while (1) {
      digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
      delay(200); // pisca rápido se o sensor falhar
    }
  }

  // configuração recomendada para modo de voo (alta precisão e filtro)
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     // modo contínuo
                  Adafruit_BMP280::SAMPLING_X2,     // oversampling de temperatura
                  Adafruit_BMP280::SAMPLING_X16,    // oversampling de pressão
                  Adafruit_BMP280::FILTER_X4,       // filtro IIR interno contra ruídos
                  Adafruit_BMP280::STANDBY_MS_55);

  // calibração inicial no solo (média de leituras para zerar a altitude base)
  float somaPressao = 0;
  for (int i = 0; i < 20; i++) {
    somaPressao += bmp.readPressure();
    delay(25);
  }
  float pressaoSoloMedia = somaPressao / 20.0;
  altitudeBase = bmp.readAltitude(pressaoSoloMedia / 100.0);

  // preenche o histórico inicial com a altitude de solo
  for (int i = 0; i < AMOSTRAS_HISTORICO; i++) {
    historicoAltitudes[i] = altitudeBase;
  }
}

void loop() {
  unsigned long tempoAtual = millis();

  // executa a leitura e a lógica na taxa definida (ex: 20Hz)
  if (tempoAtual - ultimoTempoLeitura >= intervaloLeitura) {
    ultimoTempoLeitura = tempoAtual;

    // leitura da altitude atual em relação ao nível do mar convertida para a base
    float pressaoAtual = bmp.readPressure() / 100.0; // converte para hPa
    float altitudeAtual = bmp.readAltitude(PRESSAO_MAR_PADRAO) - altitudeBase;

    // atualiza o buffer circular do histórico
    historicoAltitudes[indiceHistorico] = altitudeAtual;
    indiceHistorico = (indiceHistorico + 1) % AMOSTRAS_HISTORICO;

    // executa a Máquina de Estados do Voo
    switch (estadoAtual) {
      
      case ESTADO_SOLO:
        // se a altitude subir mais que o limiar mínimo, consideramos que o foguete decolou
        if (altitudeAtual > LIMIAR_SUBIDA_INICIAL) {
          estadoAtual = ESTADO_SUBIDA;
          digitalWrite(PIN_LED_STATUS, HIGH); // LED aceso indica voo detectado
        }
        break;

      case ESTADO_SUBIDA: {
        
        float mediaRecente = 0;
        float mediaAntiga = 0;
        
        int metade = AMOSTRAS_HISTORICO / 2;
        for (int i = 0; i < AMOSTRAS_HISTORICO; i++) {
          int idx = (indiceHistorico + i) % AMOSTRAS_HISTORICO;
          if (i < metade) {
            mediaAntiga += historicoAltitudes[idx];
          } else {
            mediaRecente += historicoAltitudes[idx];
          }
        }
        mediaAntiga = mediaAntiga / metade;
        mediaRecente = mediaRecente / (AMOSTRAS_HISTORICO - metade);

        if ((mediaRecente < mediaAntiga) && (altitudeAtual > 10.0)) {
          estadoAtual = ESTADO_APOGEU_DETECTADO;
          
          // joga o pino em HIGH para sinalizar apogeu
          digitalWrite(PIN_SAIDA_VOTO, HIGH); 
        }
        break;
      }

      case ESTADO_APOGEU_DETECTADO:
        // mantém o voto ativo. pode travar aqui até o reset físico.
        digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS)); // pisca indicando evento disparado
        break;
    }
  }
}
