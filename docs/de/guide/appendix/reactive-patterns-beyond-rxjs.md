---
description: "Reaktive Programmierungsprinzipien außerhalb von ReactiveX in Embedded-Systemen und Steuerungstechnik: Ereignisgesteuerte Architekturen, Zustandsmaschinen, Pub/Sub-Pattern und Actor-Modell."
---

# Reaktive Patterns ohne ReactiveX

Die Philosophie der reaktiven Programmierung wurde bereits lange vor ReactiveX (RxJS) in Embedded-Systemen und der Steuerungstechnik praktiziert.

Diese Seite systematisiert **Methoden, die die Prinzipien der reaktiven Programmierung ohne ReactiveX umsetzen** und verdeutlicht deren Beziehung zu RxJS.

## Das Wesen der reaktiven Programmierung

Der Kern der reaktiven Programmierung liegt in den folgenden drei Prinzipien.

1. **Datenfluss (Data Flow)** - Daten werden als zeitlich veränderliche Streams behandelt
2. **Ereignisgesteuert (Event-Driven)** - Bei Ereigniseintritt wird automatisch verarbeitet
3. **Deklarative Beschreibung (Declarative)** - Beschreibt "was zu tun ist", "wann/wie" wird abstrahiert

Diese Prinzipien werden auch in vielen Methoden außerhalb von ReactiveX umgesetzt.

::: info Das Wesen von ReactiveX
ReactiveX hat Reaktivität nicht **erfunden**, sondern bestehende Praktiken in einer **einheitlichen Abstraktionsschicht standardisiert**.
:::

## Reaktive Methoden außerhalb von ReactiveX

Wir stellen sieben repräsentative reaktive Methoden aus Embedded-Systemen und Steuerungstechnik vor.

| # | Methode | Übersicht | Typische Tools/Frameworks |
|---|------|------|--------------------------|
| 1 | **Ereignisgesteuerte Architektur** | Asynchrone Ereignisverarbeitung mit ISR/Queue | RTOS (FreeRTOS, Zephyr) |
| 2 | **Zustandsmaschinen (FSM/HSM)** | Zustandsübergänge bei Ereignissen | QPC, SCXML, Yakindu |
| 3 | **Datenfluss-Programmierung** | Knoten werden durch Datenfluss angetrieben | Simulink, LabVIEW, SCADE |
| 4 | **Signalbasierte Steuerung** | Wertänderungen werden systemweit propagiert | AUTOSAR COM Stack, Simulink |
| 5 | **Reaktive Steuerungssysteme** | Verhaltensauswahl bei Umgebungsänderungen | Behavior Tree, ROS2 |
| 6 | **Flow-Graph-Bibliotheken** | Explizite Parallelverarbeitung von Datenabhängigkeiten | Intel TBB, GNU Radio, StreamIt |
| 7 | **Funktionale reaktive Programmierung** | Funktionale Behandlung zeitlich veränderlicher Werte | Haskell Yampa, Elm, Dunai |

## 1. Ereignisgesteuerte Architektur (Event-Driven Architecture)

Eine Struktur, bei der Interrupt Service Routines (ISR) Ereignisse erfassen und Tasks über Message Queues benachrichtigen.

### C-Implementierungsbeispiel

```c
// Ereignis-Queue (Global)
typedef struct {
    EventType type;
    void* data;
} Event;

Event eventQueue[EVENT_QUEUE_SIZE];
int queueHead = 0;
int queueTail = 0;

// Interrupt Service Routine (ISR)
void ISR_SensorUpdate() {
    // Daten vom Sensor lesen
    SensorData* data = readSensor();

    // In Ereignis-Queue pushen
    Event e = { EVENT_SENSOR_NEW_DATA, data };
    EventQueue_push(e);
}

// Haupt-Task
void Task_MainLoop() {
    Event e;
    while (1) {
        if (EventQueue_pop(&e)) {
            switch (e.type) {
                case EVENT_SENSOR_NEW_DATA:
                    processSensorData((SensorData*)e.data);
                    break;
                case EVENT_TIMER_EXPIRED:
                    handleTimeout();
                    break;
                // ... andere Ereignisverarbeitung
            }
        }
    }
}
```

### Entsprechung zu RxJS

| Ereignisgesteuertes Modell | RxJS |
|-----------------|------|
| `EventQueue` | `Observable` |
| `Task_MainLoop` | `subscribe()` |
| `ISR_SensorUpdate` | `next()` |
| Ereignistyp | Typ des Stream-Wertes |

::: tip Merkmale der ereignisgesteuerten Architektur
- Weit verbreitet in RTOS (Real-Time Operating System)
- Klare Trennung von Interrupt- und Task-Verarbeitung
- Asynchrone Verarbeitung durch Queuing
:::

## 2. Zustandsmaschinen (State Machine / FSM / HSM)

Finite State Machines (FSM) oder Hierarchical State Machines (HSM) sind Muster für Zustandsübergänge basierend auf Ereigniseingaben.

### Zustandsmaschinen-Beispiel (C)

```c
typedef enum {
    STATE_IDLE,
    STATE_RUNNING,
    STATE_ERROR,
    STATE_SHUTDOWN
} State;

typedef enum {
    EVENT_START,
    EVENT_STOP,
    EVENT_ERROR_DETECTED,
    EVENT_RESET
} Event;

State currentState = STATE_IDLE;

void stateMachine(Event event) {
    switch (currentState) {
        case STATE_IDLE:
            if (event == EVENT_START) {
                currentState = STATE_RUNNING;
                startOperation();
            }
            break;

        case STATE_RUNNING:
            if (event == EVENT_STOP) {
                currentState = STATE_IDLE;
                stopOperation();
            } else if (event == EVENT_ERROR_DETECTED) {
                currentState = STATE_ERROR;
                handleError();
            }
            break;

        case STATE_ERROR:
            if (event == EVENT_RESET) {
                currentState = STATE_IDLE;
                resetSystem();
            }
            break;
    }
}
```

### RxJS-Entsprechung

```typescript
import { Subject, scan } from 'rxjs';

type State = 'IDLE' | 'RUNNING' | 'ERROR' | 'SHUTDOWN';
type Event = 'START' | 'STOP' | 'ERROR_DETECTED' | 'RESET';

const events$ = new Subject<Event>();

const state$ = events$.pipe(
  scan((state: State, event: Event): State => {
    switch (state) {
      case 'IDLE':
        return event === 'START' ? 'RUNNING' : state;
      case 'RUNNING':
        if (event === 'STOP') return 'IDLE';
        if (event === 'ERROR_DETECTED') return 'ERROR';
        return state;
      case 'ERROR':
        return event === 'RESET' ? 'IDLE' : state;
      default:
        return state;
    }
  }, 'IDLE' as State)
);

state$.subscribe(state => console.log('Aktueller Zustand:', state));

// Ereignisse auslösen
events$.next('START');   // → RUNNING
events$.next('STOP');    // → IDLE
```

## 3. Datenfluss-Programmierung (Dataflow Programming)

Eine visuelle Programmiermethode, bei der Knoten durch Datenfluss angetrieben werden.

### Typische Tools
- **MATLAB Simulink** - Steuerungssystem-Design und Simulation
- **LabVIEW (National Instruments)** - Mess- und Steuerungssystem-Entwicklung
- **SCADE (Esterel Technologies)** - Sicherheitskritische Systeme

### RxJS-Entsprechung

```typescript
import { interval } from 'rxjs';
import { map, filter, tap } from 'rxjs';

// Sensor-Stream
const sensor$ = interval(100).pipe(
  map(() => Math.random() * 100)
);

// Datenfluss-Pipeline
sensor$
  .pipe(
    map(value => lowPassFilter(value)),
    map(value => value > 50 ? value : 0),
    filter(value => value > 0),
    tap(value => actuate(value))
  )
  .subscribe();
```

## 4. Signalbasierte Steuerung (Signal-Based Control)

Ein Muster, das Wertänderungen systemweit propagiert.

### RxJS-Entsprechung (BehaviorSubject)

```typescript
import { BehaviorSubject } from 'rxjs';

interface VehicleSignals {
  speed: number;
  temperature: number;
  doorOpen: boolean;
}

const vehicleSignals$ = new BehaviorSubject<VehicleSignals>({
  speed: 0,
  temperature: 20,
  doorOpen: false
});

function updateSpeed(newSpeed: number) {
  const current = vehicleSignals$.value;
  vehicleSignals$.next({ ...current, speed: newSpeed });
}

vehicleSignals$.subscribe(signals => {
  if (signals.speed > 120) {
    console.log('⚠️ Geschwindigkeitsüberschreitung');
  }
});
```

## 5. Reaktive Steuerungssysteme

Methoden für Verhaltensauswahl bei Umgebungsänderungen.

### RxJS-Entsprechung

```typescript
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const lidarData$ = fromEvent<LaserScan>(lidarSensor, 'scan');

lidarData$
  .pipe(
    map(scan => Math.min(...scan.ranges)),
    filter(minDistance => minDistance < 0.5)
  )
  .subscribe(() => {
    console.warn('⚠️ Hindernis erkannt! Stoppe');
    stopRobot();
  });
```

## 6. Flow-Graph-Bibliotheken

Bibliotheken für explizite Datenabhängigkeits-Verarbeitung in Multithread-Umgebungen.

### RxJS-Entsprechung

```typescript
import { of } from 'rxjs';
import { map } from 'rxjs';

of(5)
  .pipe(
    map(x => x * 2),
    map(x => x + 10)
  )
  .subscribe(result => {
    console.log('Ergebnis:', result); // → Ergebnis: 20
  });
```

## 7. Funktionale reaktive Programmierung (FRP)

### RxJS-Entsprechung

```typescript
import { interval } from 'rxjs';
import { map, scan } from 'rxjs';

const simpleSF$ = interval(100).pipe(
  map(x => x * 2),
  scan((acc, value) => acc + value, 0),
  map(x => x + 10)
);

simpleSF$.subscribe(result => console.log(result));
```

## Zusammenfassung

Die Philosophie der reaktiven Programmierung wurde schon vor ReactiveX in vielen Bereichen praktiziert.

### Wichtige Punkte

1. **ReactiveX ist ein Integrator** - Gemeinsame Sprache für domänenübergreifende Methoden
2. **Optimale Lösungen pro Bereich** - Jede Methode hat geeignete Anwendungsbereiche
3. **Konzeptuelle Gemeinsamkeiten** - Ereignisgesteuert, Datenfluss, deklarativ sind gemeinsam
4. **Synergieeffekte beim Lernen** - Tiefes Verständnis einer Methode erleichtert andere

## Verwandte Seiten

- [Embedded-Entwicklung und reaktive Programmierung](/de/guide/appendix/embedded-reactive-programming)
- [RxJS-Einführung](/de/guide/introduction)
- [Was ist ein Observable](/de/guide/observables/what-is-observable)
- [Was ist ein Subject](/de/guide/subjects/what-is-subject)

## Referenzen

- [QPC (Quantum Platform)](https://www.state-machine.com/qpc/)
- [Intel TBB Flow Graph](https://www.intel.com/content/www/us/en/docs/onetbb/developer-guide-api-reference/2021-14/flow-graph.html)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [AUTOSAR Classic Platform](https://www.autosar.org/standards/classic-platform/)
