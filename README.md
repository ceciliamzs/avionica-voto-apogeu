# avionica-voto-apogeu

Firmware desenvolvido em C++ (Arduino) para atuar como uma unidade de decisão independente ("jurado") em um sistema de recuperação de foguetes com arquitetura de votação (*Triple Modular Redundancy* — TMR).

---

## Visão Geral da Arquitetura

Em um sistema TMR, três unidades de processamento independentes (`Jurado A`, `Jurado B` e `Jurado C`) monitoram o voo em paralelo e enviam seus respectivos votos para uma placa lógica de votação em hardware (baseada em portas lógicas `74LS08` / `74HC32`), responsável por consolidar a decisão por maioria (2 de 3) e acionar os estágios de recuperação (`SV1` e `SV2`).

* **Escolha do JURADO_C:** O identificador C foi escolhido apenas como exemplo de demonstração para simular um dos canais na prática; como os três nós são independentes, o mesmo código serve para qualquer entrada (A, B ou C).

---

## Hardware e Interface Elétrica

* **Microcontrolador:** Arduino Pro Mini (3,3V)
* **Sensor Barométrico:** BMP280 via barramento I2C (endereços `0x76` ou `0x77`)
* **Adequação de Nível Lógico:** Como o microcontrolador opera em **3,3V** e a placa de votação opera em **5V**, a saída de voto do jurado passa obrigatoriamente por um **conversor de nível lógico (`3,3V -> 5V`)** antes de alcançar a entrada correspondente (`A`, `B` ou `C`) da placa votadora, mantendo referência de terra (`GND`) comum entre todos os módulos.

### Pinagem Principal (Firmware de Voo/Bancada)

| Pino (Pro Mini) | Função | Descrição |
| :--- | :--- | :--- |
| `A4` (`SDA`) | Comunicação I2C | Dados do sensor barométrico BMP280 |
| `A5` (`SCL`) | Comunicação I2C | Clock do sensor barométrico BMP280 |
| `D8` (`PIN_SAIDA_VOTO`) | Saída de Voto | Envia `LOW` em solo/subida e trava em `HIGH` no apogeu (via *level shifter* 3,3V $\rightarrow$ 5V) |
| `D13` (`PIN_LED_STATUS`) | LED de Status | Apagado em solo, aceso na subida, piscando rápido após detecção de apogeu ou erro de sensor |

---

## Lógica de Detecção de Apogeu

1. **Inicialização Segura:** Garante `PIN_SAIDA_VOTO` em `LOW` antes mesmo da comunicação com o sensor, evitando votos falsos na energização ou em caso de falha de hardware.
2. **Calibração no Solo:** Coleta 20 amostras iniciais de pressão para estabelecer a altitude de referência (`altitudeBase = 0 m`).
3. **Máquina de Estados (`20 Hz`):**
   * `SOLO`: Aguarda ganho de altitude acima de `LIMIAR_SUBIDA_INICIAL` (`3,0 m`).
   * `SUBIDA`: Mantém um buffer circular de 6 amostras comparando a média das 3 leituras mais antigas com a média das 3 mais recentes. Confirma o apogeu quando `altitudeRelativa >= 10,0 m` e há queda mínima de `0,5 m` entre as médias.
   * `APOGEU_DETECTADO`: Trava a saída de voto em `HIGH` de forma persistente e sinaliza no LED de status e no *Serial Monitor*.

---

## Simulação Virtual (Wokwi)

A lógica do nó (`JURADO_C`) pode ser validada previamente em ambiente virtual sem depender do hardware físico:

* **Link da simulação:** `https://wokwi.com/projects/481623083136087041`
* **O que é validado:** Inicialização, calibração no solo, transições de estado (`SOLO` $\rightarrow$ `SUBIDA` $\rightarrow$ `APOGEU_DETECTADO`), telemetria no *Serial Monitor* (`115200 bps`) e acionamento da saída de voto (`D8`) com resistor em série e LED até o `GND`.
* **Adaptações em relação ao hardware real:**
  * Utiliza o sensor virtual **BMP180** (`Adafruit_BMP085.h`) no lugar do **BMP280** físico.
  * Não simula o conversor de nível lógico (`3,3V -> 5V`) nem a placa lógica de votação (`A, B, C` e saídas `SV1`/`SV2`).

---

## Dependências de Bibliotecas

* **Versão Hardware Real (BMP280):** `Adafruit BMP280 Library` e `Adafruit Unified Sensor`
* **Versão Simulação Wokwi (BMP180):** `Adafruit BMP085 Library`
