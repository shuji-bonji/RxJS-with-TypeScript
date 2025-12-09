---
description: Die Steuerungs-Creation-Functions von RxJS, scheduled und using, werden erklärt. scheduled kontrolliert das Ausführungs-Timing von Observables durch Angabe eines Schedulers, während using Ressourcen wie WebSocket oder Dateihandles automatisch im Einklang mit dem Observable-Lebenszyklus verwaltet. Sie können auch für Tests und Performance-Optimierung eingesetzt werden.
---

# Steuerungs-Creation-Functions

In RxJS werden Creation Functions bereitgestellt, um das Ausführungs-Timing und die Ressourcenverwaltung von Observables präzise zu steuern. In diesem Abschnitt werden die beiden Funktionen `scheduled()` und `using()` ausführlich erklärt.

## Was sind Steuerungs-Creation-Functions?

Steuerungs-Creation-Functions sind eine Gruppe von Funktionen zur feineren Steuerung des Verhaltens von Observables. Sie unterstützen fortgeschrittene Anwendungsfälle wie die Steuerung des Ausführungs-Timings (Scheduler) und das Lifecycle-Management von Ressourcen.

### Hauptmerkmale

- **Steuerung des Ausführungs-Timings**: Umschaltung zwischen synchroner und asynchroner Ausführung mithilfe von Schedulern
- **Ressourcenverwaltung**: Automatische Ressourcenfreigabe im Einklang mit dem Observable-Lebenszyklus
- **Testbarkeit**: Tests werden durch den Wechsel von Schedulern erleichtert
- **Performance-Optimierung**: Vermeidung von UI-Blockierungen durch Steuerung des Ausführungs-Timings

## Liste der Steuerungs-Creation-Functions

| Funktion | Beschreibung | Hauptverwendung |
|------|------|---------|
| [scheduled()](/de/guide/creation-functions/control/scheduled) | Erzeugt Observable mit angegebenem Scheduler | Steuerung des Ausführungs-Timings, Tests |
| [using()](/de/guide/creation-functions/control/using) | Observable mit Ressourcensteuerung | Verwaltung von Ressourcen wie WebSocket, Dateihandles |

## Grundlagen von scheduled()

`scheduled()` ist eine Funktion, die es ermöglicht, beim Erzeugen eines Observables aus einer vorhandenen Datenquelle (Array, Promise, Iterable usw.) explizit einen Scheduler anzugeben.

### Grundlegende Verwendung

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array asynchron ausgeben
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Abonnement starten');
observable$.subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});
console.log('Abonnement beenden');

// Ausgabe:
// Abonnement starten
// Abonnement beenden
// Wert: 1
// Wert: 2
// Wert: 3
// Abgeschlossen
```

> [!NOTE]
> Bei Verwendung von `asyncScheduler` wird die Ausgabe der Werte asynchron. Dadurch wird der Abonnement-Prozess ausgeführt, ohne den Hauptthread zu blockieren.

## Grundlagen von using()

`using()` ist eine Funktion, die Ressourcen automatisch erstellt und freigibt im Einklang mit dem Lebenszyklus eines Observables. Sie erstellt Ressourcen beim Start des Abonnements und gibt sie automatisch beim Ende des Abonnements frei (`complete` oder `unsubscribe`).

### Grundlegende Verwendung

```typescript
import { using, interval, Subscription, take } from 'rxjs';
const resource$ = using(
  // Ressourcen-Factory: wird beim Start des Abonnements ausgeführt
  () => {
    console.log('Ressource erstellen');
    return new Subscription(() => console.log('Ressource freigeben'));
  },
  // Observable-Factory: Observable mit der Ressource erstellen
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Wert:', value),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Ressource erstellen
// Wert: 0
// Wert: 1
// Wert: 2
// Abgeschlossen
// Ressource freigeben
```

> [!IMPORTANT]
> `using()` gibt Ressourcen automatisch beim Ende des Abonnements frei, wodurch Memory Leaks verhindert werden können.

## Vergleich: scheduled() vs using()

| Merkmal | scheduled() | using() |
|------|-------------|---------|
| Hauptzweck | Steuerung des Ausführungs-Timings | Lifecycle-Management von Ressourcen |
| Scheduler | ✅ Explizit angebbar | ❌ Nicht angebbar |
| Ressourcenverwaltung | ❌ Manuelle Verwaltung erforderlich | ✅ Automatische Verwaltung |
| Verwendungsszenario | Tests, UI-Optimierung | WebSocket, Dateihandles |
| Komplexität | Einfach | Etwas komplex |

## Richtlinien zur Auswahl

### Wann scheduled() gewählt werden sollte

1. **Wenn Ausführungs-Timing gesteuert werden soll**
   - Synchrone Verarbeitung in asynchrone umwandeln
   - UI-Blockierung vermeiden

2. **Wenn Zeitsteuerung für Tests erforderlich ist**
   - Zeit mit TestScheduler steuern
   - Asynchrone Verarbeitung synchron testen

3. **Bestehende Datenquelle in Observable umwandeln**
   - Array, Promise, Iterable in Observable konvertieren
   - Scheduler explizit angeben

### Wann using() gewählt werden sollte

1. **Wenn automatische Ressourcenfreigabe erforderlich ist**
   - Verwaltung von WebSocket-Verbindungen
   - Verwaltung von Dateihandles
   - Automatisches Cleanup von Timern

2. **Wenn Memory Leaks verhindert werden sollen**
   - Vergessen der Ressourcenfreigabe vermeiden
   - Zuverlässiges Cleanup beim Ende des Abonnements

3. **Komplexe Ressourcenverwaltung**
   - Mehrere Ressourcen gemeinsam verwalten
   - Abhängigkeiten von Ressourcen verwalten

## Praktische Verwendungsbeispiele

### Verwendungsbeispiel für scheduled()

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Große Datenmengen asynchron verarbeiten (UI nicht blockieren)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Hier schwere Verarbeitung ausführen
  // UI wird nicht blockiert
});

// In Tests synchron ausführen
const sync$ = scheduled(largeArray, queueScheduler);
```

### Verwendungsbeispiel für using()

```typescript
import { using, timer } from 'rxjs';

// WebSocket-Verbindung automatisch verwalten
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('WebSocket-Verbindung starten');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('WebSocket-Verbindung beenden');
      }
    };
  },
  () => timer(0, 1000) // Nachrichten jede Sekunde empfangen
);
```

## Arten von Schedulern (für scheduled())

| Scheduler | Beschreibung | Verwendungsszenario |
|---------------|------|---------|
| `queueScheduler` | Synchrone Ausführung (Queue-Methode) | Standard, synchrone Verarbeitung |
| `asyncScheduler` | Asynchrone Ausführung (setTimeout) | UI-Optimierung, langwierige Verarbeitung |
| `asapScheduler` | Schnellste asynchrone Ausführung (Promise) | Hochprioritäre asynchrone Verarbeitung |
| `animationFrameScheduler` | Animation Frame | Animation, UI-Rendering |

> [!TIP]
> Weitere Informationen zu Schedulern finden Sie unter [Arten von Schedulern](/de/guide/schedulers/types).

## Häufig gestellte Fragen

### F1: Was ist der Unterschied zwischen scheduled() und from()?

**A:** `from()` verwendet intern den Standard-Scheduler (synchron). Mit `scheduled()` kann der Scheduler explizit angegeben werden, wodurch das Ausführungs-Timing feiner gesteuert werden kann.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - synchrone Ausführung
const sync$ = from([1, 2, 3]);

// scheduled() - asynchrone Ausführung
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### F2: Wann sollte using() verwendet werden?

**A:** Es sollte verwendet werden, wenn Sie das Vergessen der Ressourcenfreigabe vermeiden möchten. Besonders effektiv in folgenden Fällen:
- Netzwerkverbindungen wie WebSocket, EventSource
- Dateihandles, Datenbankverbindungen
- Verarbeitung, die manuelles `clearInterval()` oder `clearTimeout()` erfordert

### F3: Warum werden Tests mit scheduled() einfacher?

**A:** Durch Verwendung von TestScheduler kann der Zeitablauf virtuell gesteuert werden. Asynchrone Verarbeitung kann synchron getestet werden, wodurch die Testausführungszeit erheblich verkürzt wird.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Best Practices

### 1. UI-Blockierung mit scheduled() vermeiden

```typescript
// ❌ Schlechtes Beispiel: Große Datenmengen synchron verarbeiten
from(largeArray).subscribe(processHeavyTask);

// ✅ Gutes Beispiel: Asynchrone Verarbeitung mit asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Ressourcen mit using() zuverlässig freigeben

```typescript
// ❌ Schlechtes Beispiel: Manuelle Ressourcenverwaltung
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Ressourcenleck durch vergessenes unsubscribe

// ✅ Gutes Beispiel: Automatische Verwaltung mit using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Geeigneten Scheduler in Tests verwenden

```typescript
// ✅ Gutes Beispiel: TestScheduler in Tests
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Gutes Beispiel: asyncScheduler in Produktion
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Zusammenfassung

Steuerungs-Creation-Functions sind fortgeschrittene Funktionen zur feinen Steuerung des Verhaltens von Observables.

**scheduled():**
- Explizite Steuerung des Ausführungs-Timings (synchron/asynchron)
- Praktisch für Zeitsteuerung in Tests
- Effektiv zur Vermeidung von UI-Blockierungen

**using():**
- Automatisches Management des Ressourcen-Lebenszyklus
- Verhindert Memory Leaks
- Optimal für Verbindungsverwaltung wie WebSocket

Durch angemessene Verwendung können robustere und leistungsfähigere RxJS-Anwendungen erstellt werden.

## Nächste Schritte

Für detaillierte Verwendung jeder Funktion siehe folgende Seiten:

- [Details zu scheduled()](/de/guide/creation-functions/control/scheduled) - Observable mit angegebenem Scheduler erzeugen
- [Details zu using()](/de/guide/creation-functions/control/using) - Observable mit Ressourcensteuerung

## Referenzressourcen

- [RxJS Offizielle Dokumentation - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Offizielle Dokumentation - using](https://rxjs.dev/api/index/function/using)
- [RxJS Offizielle Dokumentation - Scheduler](https://rxjs.dev/guide/scheduler)
- [Arten von Schedulern](/de/guide/schedulers/types)
