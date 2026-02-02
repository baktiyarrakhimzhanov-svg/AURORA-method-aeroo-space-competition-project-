Каждый день спутники Земли делают миллионы снимков.
Но катастрофы начинаются не с катастроф- а с едва заметных отклонений.
Незаметное изменение цвета воды.
Небольшое смещение температуры.
Странная тень там, где ее раньше не было.
Эти сигналы теряются в океане данных.
Не потому что сенсоры плохие.
А потому что мы не умеем вовремя понять, что именно важно.

AURORA-это метод, который учит системы видеть то, что еще не стало проблемой.

Проблема:

Слишком большое количество ненужных данных.
Изза чего приходится тратить много времени на обработку, и отсеивании ненужной информации.
Данных слишком много, а фильтра нет. 
Проблема не в отсутвии данных, а в отсутвиии метода их приоритизации. 

AURORA

Это метод анализа данных, который выявляет неочевидные, но потенциально критические отклонения и приоритизирует их по риску.
AURORA использует многоуровневую архитектуру анализа данных,
направленную не на поиск заранее известных событий, а на выявление статистически редких,
структурно необычных и потенциально опасных отклонений.

Принцип работы:

Этап 1- Adaptive Baseline Modeling.
(формирование нормы)

Система строит динамическую модель "нормального состояния" среды на основе:

Исторических данных.
Сезонных циклов.
Пространственных закономерностей.
Долгосрочных трендов.

Модель нормы адаптивна, что позволяет  учитывать естественные изменения,
не теряя чуствительности к аномалиям.

Этап 2- Multidimensional Anomaly Detection.

Каждый новый набор данных 
анализируется в нескольких измерениях:

Статистический- Насколько значение отклоняется от нормы.
Темпоральный- Насколько быстро происходиь изменение.
Пространственный- Насколько аномалия локальна или распростроняется.
Структурный- Необычные формы, паттерны, текстуры.

Система использует методы unsupervised learnong, что позволяет обнаруживать неизвестные ранее типы отклонений.

Этап 3-Sensor Confidence Filtering.

AURORA  оценивает надежность сигнала, чтобы отличать:
Реальную аномалию.
Шум.
Ошибку сенсора.
Это снижает ложные тревоги.

Этап 4- Risk-Oriented Prioritization.

Все обнаруженные аномалии получают Risk score - оценку потенциальной угрозы.

Как это работает?

Формула RISK SCORE:
Risk score calculation.

Каждое отклонение оценивается по нескольким параметрам:

R=(wr x dr)+(wv x vr)+(ws x sr) + (wu x ur) - (wc x c)

где:
dr- степень статического отклонения от нормы.
vr-скорость изменения во времени.
sr-пространственный масштаб распростронения.
ur-уровень необычности(насколько событие похоже на ранее наблюдавшиеся).
c-надежность сенсора.
wr,wv,ws,wu,wc-весовые коэфиценты(настраивается под задачу).

Простыми словами:

AURORA считает событие важным, если оно:

отличается от нормы.
развивается.
распростроняется.
выглядит необычно.
подтверждается надежными сенсорами.

Научная новизна:

Обычно как работает космический мониторинг:
Люди заранее решают, что искать.
Прописывают правила.
Спутник ищет только это.

1-AURORA- это использование адаптативного аномалийного ИИ для автономного научного открытия, 
а не просто класификации заранее известных событий.

Сейчас спутники = камеры+ передатчики.
Они не думают, они просто заливают Землю гиганскими потоками данных.

2-Перенос когнитивной обработки данных в космос, что снижает зависимость от пропускной способности связи и ускоряет реакцию на критические события.

Большинство систем:
отдельная для пожаров.
отдельная для наводнений.
отдельная для лесов.
отдельная для городов.

3-Создание обобщенной системы обнаружения аномалий для мультисенсорных космических данных, способной выявлять ранее неизвестные типы событий.
Что позволяет использовать AURORA в разных областях, что делает ее универсальной.

Применение ИИ:

ИИ — это мозг, который решает, какие данные важные, а какие нет.
Спутник просто собирает кучу сырых данных:
свет, расстояние, температура, изображения, сигналы и т.д.
Сам по себе он ничего не понимает.

Что делает ИИ внутри AURORA?

ИИ выполняет 3 ключевые задачи:

1️-Понимает, что “нормально”.
Он учится на потоке данных:

какой уровень света обычный.
какие расстояния типичные.
какие изменения происходят постоянно.

То есть он строит модель нормы.

2-Находит отклонения.

Если что-то резко отличается от нормы -
ИИ говорит:

"Стоп. Это необычно."

Но не каждое отклонение = катастрофа. Поэтому дальше…

3-Оценивает риск.
ИИ решает, что делать с этим отклонением:
Что делает AURORA:
Похоже на пожар	СРОЧНО отправляет данные.
Похоже на наводнение СРОЧНО отправляет.
Что-то странное, но не критично	Помечает как «интересное для науки».
Просто шум	Игнорирует.

Без ИИ это была бы просто камера и датчики.
Без AURORA спутник:
либо отправляет ВСЁ (слишком много мусора).
либо работает по жёстким правилам (видит только то, что уже известно).

С ИИ:

он сам учится.
сам замечает новое.
сам понимает насколько это важно.

вкратце:
ИИ в AURORA нужен, чтобы спутник не просто видел, а понимал, что он видит, и решал, что из этого действительно важно для людей.

Бизнес модель:

Aurora применим для государственных служб, крупных компаний (энергетика, транспорт, добыча) и международных организаций. 
Метод использует существующие данные: спутники, сенсоры, метео- и сейсмическую информацию. 
Мы продаём только технологию и алгоритм, не разрабатываем новые спутники и не работаем с аппаратной частью, поэтому проект полностью реализуем.

Бизнес-модель Aurora основана на подписке, лицензировании метода, оплате за критические оповещения и аналитических отчётах. 
Метод легко масштабируется на новые регионы, типы данных и категории угроз, а также интегрируется с современными сенсорными системами и IoT.

Aurora строится на модели B2B и B2G и включает несколько каналов монетизации:
Подписка (SaaS): ежемесячная или годовая оплата за использование платформы, в зависимости от объёма данных, числа регионов и уровня критичности информации.
Лицензирование метода: предоставление технологии для интеграции в закрытые системы клиента, включая государственные или корпоративные сети.
Оплата за инциденты (Pay-per-Alert): дополнительная оплата за высокоточные оповещения о критических событиях.
Аналитические отчёты: долгосрочные прогнозы, кастомные модели и рекомендации для клиентов.

Метод легко масштабируется:
Горизонтально — расширение на новые страны, регионы и типы данных.
Вертикально — добавление новых категорий угроз, углублённая аналитика, интеграция с IoT и спутниковыми источниками.

Конкурентные преимущества:
Универсальность — один метод для любых аномалий и угроз.
Выявление неизвестного — Aurora ищет не только известные опасности, но и новое, необычное поведение систем.
Приоритет критических случаев — ничего важного не остаётся без внимания.
Реализуемость — технология работает с существующими данными и легко интегрируется, не требуя строительства новых спутников или аппаратной инфраструктуры.

Экономическая ценность

Для клиентов Aurora приносит:
снижение рисков и предотвращение убытков миллиардного масштаба,
уменьшение человеческих жертв,
оптимизацию работы служб и инфраструктуры,
повышение доверия к системам мониторинга.

Масшабируемость.

Этап-1. ПЕРВЫЕ ПИЛОТЫ.(школы,маленькие склады)

Цель:
Не заработать, а
Поставить систему.
получить кейсы.
собрать данные.

Цена:
300$-700$ за объект.

Почему не дешевле:
Потому что даже прототип:
оборудование.
время.
настройка.

Этап 2- ПЕРВЫЕ КОММЕРЧЕСКИЕ КЛИЕНТЫ.(частные компании)

Когда уже есть:
2-5 объектов.
отзывы.
зафиксированные "опасные случаи"

цена:
2000$-5000$ за объект.

Почему цена выросла:
теперь продается не идея, а система которая уже предотвратила инциденты.

Добавляется:
больше датчиков.
улучшеный алгоритм.
настройка под объект.
поддержка.

Этап 3- СЕТИ И КРУПНЫЕ ОБЪЕКТЫ.(сети складов,университеты,заводы)

Цена:
15000$-40000$ за объект.

или

2000$-5000$ в месяц подписка.

теперь это уже:
инфраструктурная система.
аналитика.
интеграция безопасности.
круглосуточный мониторинг.

Этап 4-ГОСУДАРСТВО И НАЦ. СИСТЕМЫ.(аэропорты,города,стратегические объекты)

Цена:
250000$ за объект.

почему так дорого:
масштаб.
серверы.
интеграция.
поддержка.
данные.
ответственность.

Главная особенность AURORA в ее универсальности. AURORA сохраняя принцип работы, может быть использован в исследовании космоса, 
предсказывании катастроф, в безопасности складов.

В рамках проверки AURORA.

был создан прототип на основе arduino uno r3.

список запчастей:
1.Arduino Uno R3
2.HC‑SR04 Ultrasonic Distance Sensor
3.LDR Light Sensor 4.Module
Breadboard Starter Kit w/ Jumper Wires
5.Assorted 5mm LED Pack
6.Resistor Kit (220Ω, 10kΩ)

код:

/*
  Clean Object Analyzer (Arduino UNO)
  Components:
    - HC-SR04 ultrasonic distance sensor
    - LDR light sensor module (Analog AO)
    - 3 LEDs + 220 ohm resistors

  Behavior:
    - Detect object presence by distance threshold
    - Compute filtered light + distance
    - Match against a small in-code "database" (signature ranges)
    - Non-blocking loop (millis scheduling)
    - Serial commands for calibration and diagnostics

  Serial commands (9600 baud):
    h            - help
    s            - snapshot (distance + light)
    b            - print database
    t <cm>       - set object presence threshold (cm). Example: t 25
    i <n>        - set stable frames required for detection (n). Example: i 4
*/

#include <Arduino.h>

// --------------------------- Pins ---------------------------
namespace Pins {
  constexpr uint8_t kTrig = 9;
  constexpr uint8_t kEcho = 10;

  constexpr uint8_t kLdrAnalog = A0;

  constexpr uint8_t kLedGreen  = 3;  // FOUND
  constexpr uint8_t kLedYellow = 4;  // PRESENT but NOT FOUND
  constexpr uint8_t kLedRed    = 5;  // ERROR / NO SENSOR DATA
}

// --------------------------- Utilities ---------------------------
template <typename T>
static T clampValue(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

struct RgbState {
  bool green = false;
  bool yellow = false;
  bool red = false;
};

// Simple exponential moving average (EMA) filter.
class EMAFilter {
public:
  explicit EMAFilter(float alpha = 0.25f) : alpha_(clampValue(alpha, 0.01f, 1.0f)) {}
  void reset(float value) { has_ = true; y_ = value; }
  float update(float x) {
    if (!has_) { reset(x); return y_; }
    y_ = alpha_ * x + (1.0f - alpha_) * y_;
    return y_;
  }
  float value() const { return y_; }

private:
  float alpha_;
  bool has_ = false;
  float y_ = 0.0f;
};

// --------------------------- Sensor: HC-SR04 ---------------------------
class UltrasonicHC_SR04 {
public:
  UltrasonicHC_SR04(uint8_t trigPin, uint8_t echoPin)
    : trig_(trigPin), echo_(echoPin) {}

  void begin() const {
    pinMode(trig_, OUTPUT);
    pinMode(echo_, INPUT);
    digitalWrite(trig_, LOW);
  }

  // Returns distance in cm. If no echo: returns NAN.
  float readDistanceCm(uint32_t echoTimeoutUs = 30000UL) const {
    // Trigger pulse: 10us HIGH
    digitalWrite(trig_, LOW);
    delayMicroseconds(2);
    digitalWrite(trig_, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_, LOW);

    const unsigned long duration = pulseIn(echo_, HIGH, echoTimeoutUs);
    if (duration == 0) return NAN;

    // Distance(cm) = duration(us) * 0.0343 / 2
    return (duration * 0.0343f) / 2.0f;
  }

private:
  uint8_t trig_;
  uint8_t echo_;
};

// --------------------------- Sensor: LDR Analog ---------------------------
class AnalogLDR {
public:
  explicit AnalogLDR(uint8_t analogPin) : pin_(analogPin) {}

  void begin() const {
    // Analog pin setup not required; kept for symmetry.
  }

  // Returns 0..1023
  int readRaw() const {
    return analogRead(pin_);
  }

private:
  uint8_t pin_;
};

// --------------------------- Signature Database ---------------------------
struct Signature {
  const char* name;

  // Distance window (cm)
  float distMin;
  float distMax;

  // Light window (0..1023)
  int lightMin;
  int lightMax;
};

class SignatureDB {
public:
  SignatureDB(const Signature* items, size_t count) : items_(items), count_(count) {}

  int match(float distCm, int light) const {
    for (size_t i = 0; i < count_; i++) {
      const Signature& s = items_[i];
      if (distCm >= s.distMin && distCm <= s.distMax &&
          light  >= s.lightMin && light  <= s.lightMax) {
        return (int)i;
      }
    }
    return -1;
  }

  void printTo(Stream& out) const {
    out.println(F("DB signatures:"));
    for (size_t i = 0; i < count_; i++) {
      const Signature& s = items_[i];
      out.print(F("  #"));
      out.print(i);
      out.print(F(" '"));
      out.print(s.name);
      out.print(F("' dist["));
      out.print(s.distMin, 1);
      out.print(F(".."));
      out.print(s.distMax, 1);
      out.print(F("]cm light["));
      out.print(s.lightMin);
      out.print(F(".."));
      out.print(s.lightMax);
      out.println(F("]"));
    }
  }

private:
  const Signature* items_;
  size_t count_;
};

// --------------------------- LED Controller ---------------------------
class LedPanel {
public:
  void begin() const {
    pinMode(Pins::kLedGreen, OUTPUT);
    pinMode(Pins::kLedYellow, OUTPUT);
    pinMode(Pins::kLedRed, OUTPUT);
    set({false, false, false});
  }

  void set(const RgbState& s) const {
    digitalWrite(Pins::kLedGreen,  s.green  ? HIGH : LOW);
    digitalWrite(Pins::kLedYellow, s.yellow ? HIGH : LOW);
    digitalWrite(Pins::kLedRed,    s.red    ? HIGH : LOW);
  }
};

// --------------------------- Analyzer ---------------------------
struct Measurement {
  float distCm = NAN;
  int lightRaw = 0;

  float distFiltered = NAN;
  float lightFiltered = 0.0f;

  bool valid() const { return !isnan(distCm); }
};

class ObjectAnalyzer {
public:
  ObjectAnalyzer(UltrasonicHC_SR04& us, AnalogLDR& ldr, const SignatureDB& db)
    : us_(us), ldr_(ldr), db_(db), distFilter_(0.35f), lightFilter_(0.20f) {}

  void setPresenceThresholdCm(float cm) { presenceThresholdCm_ = clampValue(cm, 2.0f, 400.0f); }
  float presenceThresholdCm() const { return presenceThresholdCm_; }

  void setStableFramesRequired(uint8_t n) { stableFramesRequired_ = clampValue<uint8_t>(n, 1, 20); }
  uint8_t stableFramesRequired() const { return stableFramesRequired_; }

  Measurement sample() {
    Measurement m;
    m.distCm = us_.readDistanceCm();
    m.lightRaw = ldr_.readRaw();

    // Update filters only if distance is valid
    if (!isnan(m.distCm)) {
      m.distFiltered = distFilter_.update(m.distCm);
      m.lightFiltered = lightFilter_.update((float)m.lightRaw);
    } else {
      m.distFiltered = NAN;
      m.lightFiltered = lightFilter_.value(); // keep last
    }
    last_ = m;
    return m;
  }

  enum class State { kNoObject, kPresentNotFound, kFound, kError };

  struct Decision {
    State state = State::kError;
    int matchedIndex = -1;
  };

  Decision decide(const Measurement& m) {
    Decision d;

    if (!m.valid()) {
      stablePresentCount_ = 0;
      stableFoundCount_ = 0;
      d.state = State::kError;
      return d;
    }

    const bool present = (m.distFiltered <= presenceThresholdCm_);

    if (!present) {
      stablePresentCount_ = 0;
      stableFoundCount_ = 0;
      d.state = State::kNoObject;
      return d;
    }

    // present
    stablePresentCount_ = (stablePresentCount_ < 255) ? (stablePresentCount_ + 1) : 255;

    const int lightInt = (int)lroundf(m.lightFiltered);
    const int idx = db_.match(m.distFiltered, lightInt);

    if (idx >= 0) {
      stableFoundCount_ = (stableFoundCount_ < 255) ? (stableFoundCount_ + 1) : 255;
    } else {
      stableFoundCount_ = 0;
    }

    // Require stability before declaring FOUND
    if (stablePresentCount_ >= stableFramesRequired_ && stableFoundCount_ >= stableFramesRequired_) {
      d.state = State::kFound;
      d.matchedIndex = idx;
    } else {
      d.state = State::kPresentNotFound;
      d.matchedIndex = idx; // may be -1 while stabilizing
    }

    return d;
  }

  const Measurement& last() const { return last_; }

private:
  UltrasonicHC_SR04& us_;
  AnalogLDR& ldr_;
  const SignatureDB& db_;

  EMAFilter distFilter_;
  EMAFilter lightFilter_;

  float presenceThresholdCm_ = 25.0f; // object if closer than this
  uint8_t stableFramesRequired_ = 4;  // anti-noise stability

  uint8_t stablePresentCount_ = 0;
  uint8_t stableFoundCount_ = 0;

  Measurement last_;
};

// --------------------------- Configuration: DB ---------------------------
// Заполни эти диапазоны после "калибровки" (см. Serial 's').
const Signature kSignatures[] = {
  // name,         distMin, distMax, lightMin, lightMax
  { "WHITE_CLOSE",   5.0f,   12.0f,     650,     1023 },
  { "DARK_CLOSE",    5.0f,   12.0f,       0,      380 },
  { "MID_RANGE",    12.0f,   25.0f,     350,      750 },
};
SignatureDB gDb(kSignatures, sizeof(kSignatures) / sizeof(kSignatures[0]));

// --------------------------- Globals ---------------------------
UltrasonicHC_SR04 gUltrasonic(Pins::kTrig, Pins::kEcho);
AnalogLDR gLdr(Pins::kLdrAnalog);
LedPanel gLeds;
ObjectAnalyzer gAnalyzer(gUltrasonic, gLdr, gDb);

// Scheduler
constexpr uint32_t kSamplePeriodMs = 80;   // sensor sampling rate
constexpr uint32_t kReportPeriodMs = 250;  // serial reporting rate

uint32_t gNextSampleMs = 0;
uint32_t gNextReportMs = 0;

// --------------------------- Serial Helpers ---------------------------
static void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  h            help"));
  Serial.println(F("  s            snapshot (distance+light)"));
  Serial.println(F("  b            print database"));
  Serial.println(F("  t <cm>       set presence threshold, e.g. t 25"));
  Serial.println(F("  i <n>        set stable frames required, e.g. i 4"));
}

static void printSnapshot(const Measurement& m) {
  Serial.print(F("distRaw(cm)="));
  if (isnan(m.distCm)) Serial.print(F("NAN"));
  else Serial.print(m.distCm, 2);

  Serial.print(F(" distF(cm)="));
  if (isnan(m.distFiltered)) Serial.print(F("NAN"));
  else Serial.print(m.distFiltered, 2);

  Serial.print(F(" lightRaw="));
  Serial.print(m.lightRaw);

  Serial.print(F(" lightF="));
  Serial.println((int)lroundf(m.lightFiltered));
}

static bool tryReadLine(String& outLine) {
  static String buf;
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      outLine = buf;
      buf = "";
      return true;
    }
    buf += c;
    if (buf.length() > 80) buf.remove(0, buf.length() - 80); // keep bounded
  }
  return false;
}

static void handleCommandLine(const String& line) {
  if (line.length() == 0) return;

  // Simple tokenization: cmd + optional integer
  char cmd = line.charAt(0);

  if (cmd == 'h') {
    printHelp();
    return;
  }

  if (cmd == 's') {
    printSnapshot(gAnalyzer.last());
    return;
  }

  if (cmd == 'b') {
    gDb.printTo(Serial);
    return;
  }

  if (cmd == 't') {
    float v = line.substring(1).toFloat();
    if (v > 0) {
      gAnalyzer.setPresenceThresholdCm(v);
      Serial.print(F("presenceThresholdCm="));
      Serial.println(gAnalyzer.presenceThresholdCm(), 1);
    } else {
      Serial.println(F("Usage: t <cm>"));
    }
    return;
  }

  if (cmd == 'i') {
    int v = line.substring(1).toInt();
    if (v > 0) {
      gAnalyzer.setStableFramesRequired((uint8_t)v);
      Serial.print(F("stableFramesRequired="));
      Serial.println(gAnalyzer.stableFramesRequired());
    } else {
      Serial.println(F("Usage: i <n>"));
    }
    return;
  }

  Serial.println(F("Unknown command. Type 'h'."));
}

// --------------------------- Setup / Loop ---------------------------
void setup() {
  Serial.begin(9600);

  gUltrasonic.begin();
  gLdr.begin();
  gLeds.begin();

  printHelp();
  gDb.printTo(Serial);

  gNextSampleMs = millis();
  gNextReportMs = millis();
}

void loop() {
  // 1) Serial input (non-blocking)
  String line;
  if (tryReadLine(line)) handleCommandLine(line);

  const uint32_t now = millis();

  // 2) Sample sensors on schedule
  if ((int32_t)(now - gNextSampleMs) >= 0) {
    gNextSampleMs += kSamplePeriodMs;

    Measurement m = gAnalyzer.sample();
    ObjectAnalyzer::Decision d = gAnalyzer.decide(m);

    // 3) Drive LEDs based on decision
    RgbState leds;

    switch (d.state) {
      case ObjectAnalyzer::State::kNoObject:
        leds = {false, false, false};
        break;

      case ObjectAnalyzer::State::kPresentNotFound:
        // present but not found (or stabilizing)
        leds = {false, true, false};
        break;

      case ObjectAnalyzer::State::kFound:
        leds = {true, false, false};
        break;

      case ObjectAnalyzer::State::kError:
      default:
        leds = {false, false, true};
        break;
    }

    gLeds.set(leds);
  }

  // 4) Report status on schedule (optional)
  if ((int32_t)(now - gNextReportMs) >= 0) {
    gNextReportMs += kReportPeriodMs;

    const Measurement& m = gAnalyzer.last();
    if (!m.valid()) {
      Serial.println(F("[ERR] No ultrasonic echo"));
      return;
    }

    // Lightweight status line
    Serial.print(F("d="));
    Serial.print(isnan(m.distFiltered) ? -1 : m.distFiltered, 1);
    Serial.print(F("cm l="));
    Serial.print((int)lroundf(m.lightFiltered));

    // Show match
    int idx = gDb.match(m.distFiltered, (int)lroundf(m.lightFiltered));
    if (m.distFiltered <= gAnalyzer.presenceThresholdCm()) {
      if (idx >= 0) {
        Serial.print(F(" -> FOUND "));
        Serial.println(kSignatures[idx].name);
      } else {
        Serial.println(F(" -> PRESENT"));
      }
    } else {
      Serial.println(F(" -> NONE"));
    }
  }
}

прототип используя принцип работы AURORA, проверяет освещенность и расстояние до объекта. тем самым фиксируя "отклонения".
Прототип это не готовая версия AURORA, а лишь способ проверить работает ли метод.

В итоге Aurora — это инновационный, масштабируемый и полностью реализуемый метод, 
который помогает находить опасность ещё на ранней стадии и обеспечивает надежную защиту без лишних затрат на инфраструктуру.

