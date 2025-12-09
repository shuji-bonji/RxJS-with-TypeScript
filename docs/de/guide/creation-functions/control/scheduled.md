---
description: "Mit der RxJS Creation Function scheduled() können Sie einen Scheduler angeben, um Observables zu erzeugen und das Ausführungs-Timing zu steuern. asyncScheduler, queueScheduler und mehr, Performance-Optimierung, typsichere Implementierung mit TypeScript."
---

# scheduled()

[📘 RxJS Offizielle Dokumentation - scheduled](https://rxjs.dev/api/index/function/scheduled)

`scheduled()` ist eine Creation Function, mit der Sie beim Erzeugen eines Observables aus Datenquellen wie Arrays, Promises oder Iterables explizit einen Scheduler angeben können. Sie ermöglicht eine feine Steuerung des Ausführungs-Timings (synchron/asynchron) und ist nützlich für Tests und UI-Performance-Optimierung.

## Grundlegende Verwendung

### Einfache Array-zu-Observable-Konvertierung

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

> [!IMPORTANT]
> **Unterschied zwischen synchron und asynchron**
>
> Bei Verwendung von `asyncScheduler` wird die Ausgabe der Werte asynchron. Daher erfolgt die Ausgabe in der Reihenfolge: "Abonnement starten" → "Abonnement beenden" → "Wert: 1".

### Vergleich mit from()

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - standardmäßig synchron
console.log('=== from() ===');
from([1, 2, 3]).subscribe(val => console.log('Wert:', val));
console.log('Abonnement beenden');

// Ausgabe:
// === from() ===
// Wert: 1
// Wert: 2
// Wert: 3
// Abonnement beenden

// scheduled() - explizit asynchron
console.log('=== scheduled() ===');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Wert:', val));
console.log('Abonnement beenden');

// Ausgabe:
// === scheduled() ===
// Abonnement beenden
// Wert: 1
// Wert: 2
// Wert: 3
```

## Arten von Schedulern

RxJS bietet mehrere Scheduler, die je nach Verwendungszweck ausgewählt werden können.

| Scheduler | Ausführungs-Timing | Basis-Technologie | Hauptverwendung |
|---------------|--------------|-----------|---------|
| `queueScheduler` | Synchron (Queue-Methode) | Sofortige Ausführung | Standard, synchrone Verarbeitung |
| `asyncScheduler` | Asynchron | `setTimeout` | UI-Optimierung, langwierige Verarbeitung |
| `asapScheduler` | Schnellste asynchrone Ausführung | `Promise` (microtask) | Hochprioritäre asynchrone Verarbeitung |
| `animationFrameScheduler` | Animation Frame | `requestAnimationFrame` | Animation, UI-Rendering |

### queueScheduler (synchrone Ausführung)

```typescript
import { scheduled, queueScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], queueScheduler).subscribe(val => console.log('Wert:', val));
console.log('Ende');

// Ausgabe:
// Start
// Wert: 1
// Wert: 2
// Wert: 3
// Ende
```

### asyncScheduler (asynchrone Ausführung)

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asyncScheduler).subscribe(val => console.log('Wert:', val));
console.log('Ende');

// Ausgabe:
// Start
// Ende
// Wert: 1
// Wert: 2
// Wert: 3
```

### asapScheduler (Microtask)

```typescript
import { scheduled, asapScheduler } from 'rxjs';

console.log('Start');
scheduled([1, 2, 3], asapScheduler).subscribe(val => console.log('Wert:', val));
console.log('Ende');

// Ausgabe:
// Start
// Ende
// Wert: 1
// Wert: 2
// Wert: 3
```

> [!TIP]
> **asyncScheduler vs asapScheduler**
>
> - `asyncScheduler`: `setTimeout`-basiert (Macrotask)
> - `asapScheduler`: `Promise`-basiert (Microtask)
>
> `asapScheduler` wird früher ausgeführt, aber beide sind asynchron.

### animationFrameScheduler (Animation)

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Werte bei jedem Animation Frame aktualisieren
const positions = [0, 50, 100, 150, 200];
const animation$ = scheduled(positions, animationFrameScheduler).pipe(
  map(pos => `Position: ${pos}px`)
);

animation$.subscribe(position => {
  console.log(position);
  // DOM-Aktualisierung hier durchführen
});

// Ausgabe: (bei jedem Animation Frame)
// Position: 0px
// Position: 50px
// Position: 100px
// Position: 150px
// Position: 200px
```

## Praktische Muster

### Verarbeitung großer Datenmengen ohne UI-Blockierung

```typescript
import { scheduled, asyncScheduler, map, bufferCount } from 'rxjs';
// 1 Million Datensätze verarbeiten
const largeArray = Array.from({ length: 1000000 }, (_, i) => i);

// ❌ Schlechtes Beispiel: Synchrone Verarbeitung (UI wird blockiert)
// from(largeArray).subscribe(processData);

// ✅ Gutes Beispiel: Asynchrone Verarbeitung (UI wird nicht blockiert)
scheduled(largeArray, asyncScheduler).pipe(
  bufferCount(1000), // Batch-Verarbeitung von 1000 Stück
  map(batch => batch.reduce((sum, val) => sum + val, 0))
).subscribe({
  next: sum => console.log('Batch-Summe:', sum),
  complete: () => console.log('Verarbeitung abgeschlossen')
});

console.log('UI bleibt weiterhin responsiv');
```

### Kombination mit Promises

```typescript
import { scheduled, asyncScheduler, mergeMap } from 'rxjs';
interface User {
  id: number;
  name: string;
}

const userIds = [1, 2, 3, 4, 5];

// Mehrere Benutzer asynchron abrufen
scheduled(userIds, asyncScheduler).pipe(
  mergeMap(id =>
    fetch(`https://api.example.com/users/${id}`).then(res => res.json())
  )
).subscribe({
  next: (user: User) => console.log('Benutzer:', user),
  error: error => console.error('Fehler:', error),
  complete: () => console.log('Alle Benutzer abgerufen')
});
```

### Erzeugung aus Iterables

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Set mit Scheduling konvertieren
const uniqueNumbers = new Set([1, 2, 3, 4, 5]);
const observable$ = scheduled(uniqueNumbers, asyncScheduler);

observable$.subscribe({
  next: val => console.log('Wert:', val),
  complete: () => console.log('Abgeschlossen')
});

// Map mit Scheduling konvertieren
const userMap = new Map([
  [1, 'Alice'],
  [2, 'Bob'],
  [3, 'Charlie']
]);

scheduled(userMap, asyncScheduler).subscribe({
  next: ([id, name]) => console.log(`ID: ${id}, Name: ${name}`),
  complete: () => console.log('Abgeschlossen')
});
```

## Nutzung in Tests

Mit `scheduled()` können Sie Tests mit Zeitsteuerung schreiben, indem Sie es mit TestScheduler kombinieren.

### Grundlegender Test

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled } from 'rxjs';

describe('scheduled()', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('gibt Array nacheinander aus', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler);
      const expected = '(abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

### Test asynchroner Verarbeitung

```typescript
import { scheduled, asyncScheduler, delay } from 'rxjs';
import { TestScheduler } from 'rxjs/testing';

describe('Test asynchroner Verarbeitung', () => {
  let testScheduler: TestScheduler;

  beforeEach(() => {
    testScheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('verzögerte Verarbeitung virtuell testen', () => {
    testScheduler.run(({ expectObservable }) => {
      const source$ = scheduled([1, 2, 3], testScheduler).pipe(
        delay(1000, testScheduler)
      );

      // Ausgabe nach 1000ms (virtuelle Zeit)
      const expected = '1000ms (abc|)';
      const values = { a: 1, b: 2, c: 3 };

      expectObservable(source$).toBe(expected, values);
    });
  });
});
```

> [!TIP]
> **Vorteile von TestScheduler**
>
> - Tests ohne tatsächliches Warten auf Zeit
> - Asynchrone Verarbeitung synchron testen
> - Testausführungszeit erheblich verkürzen

## Häufige Verwendungsbeispiele

### 1. Datenabruf mit Paginierung

```typescript
import { scheduled, asyncScheduler, mergeMap, toArray } from 'rxjs';
interface Page {
  page: number;
  data: any[];
}

// Liste der Seitennummern
const pages = [1, 2, 3, 4, 5];

// Jede Seite asynchron abrufen
const allData$ = scheduled(pages, asyncScheduler).pipe(
  mergeMap(page =>
    fetch(`https://api.example.com/items?page=${page}`)
      .then(res => res.json())
  ),
  toArray() // Daten aller Seiten zusammenführen
);

allData$.subscribe({
  next: data => console.log('Alle Daten:', data),
  complete: () => console.log('Abruf abgeschlossen')
});
```

### 2. Batch-Verarbeitung

```typescript
import { scheduled, asyncScheduler, bufferCount, mergeMap, delay } from 'rxjs';
// Große Anzahl von Tasks in 1000er-Batches verarbeiten
const tasks = Array.from({ length: 10000 }, (_, i) => `Task-${i}`);

scheduled(tasks, asyncScheduler).pipe(
  bufferCount(1000), // In 1000er-Batches aufteilen
  mergeMap(batch => {
    console.log(`Batch-Verarbeitung: ${batch.length} Stück`);
    // Batch-Verarbeitung ausführen
    return processBatch(batch);
  })
).subscribe({
  complete: () => console.log('Alle Batch-Verarbeitungen abgeschlossen')
});

function processBatch(batch: string[]): Promise<void> {
  // Batch-Verarbeitungslogik
  return Promise.resolve();
}
```

### 3. Animation-Implementierung

```typescript
import { scheduled, animationFrameScheduler, map } from 'rxjs';
// Werte von 0 bis 100 erzeugen
const frames = Array.from({ length: 100 }, (_, i) => i);

// Bei jedem Animation Frame ausführen
const animation$ = scheduled(frames, animationFrameScheduler).pipe(
  map(frame => ({
    progress: frame / 100,
    position: frame * 5 // Bewegung von 0px bis 500px
  }))
);

animation$.subscribe({
  next: ({ progress, position }) => {
    const element = document.getElementById('animated-box');
    if (element) {
      element.style.transform = `translateX(${position}px)`;
      console.log(`Fortschritt: ${(progress * 100).toFixed(0)}%`);
    }
  },
  complete: () => console.log('Animation abgeschlossen')
});
```

### 4. Priorisierte Task-Verarbeitung

```typescript
import { scheduled, asapScheduler, asyncScheduler } from 'rxjs';

// Hochprioritäts-Tasks (asapScheduler = Microtask)
const highPriorityTasks = ['Dringender Task 1', 'Dringender Task 2'];
const highPriority$ = scheduled(highPriorityTasks, asapScheduler);

// Niedrigprioritäts-Tasks (asyncScheduler = Macrotask)
const lowPriorityTasks = ['Normaler Task 1', 'Normaler Task 2'];
const lowPriority$ = scheduled(lowPriorityTasks, asyncScheduler);

console.log('Task-Start');

highPriority$.subscribe(task => console.log('Hohe Priorität:', task));
lowPriority$.subscribe(task => console.log('Niedrige Priorität:', task));

console.log('Task-Registrierung abgeschlossen');

// Ausgabe:
// Task-Start
// Task-Registrierung abgeschlossen
// Hohe Priorität: Dringender Task 1
// Hohe Priorität: Dringender Task 2
// Niedrige Priorität: Normaler Task 1
// Niedrige Priorität: Normaler Task 2
```

## Optionen von scheduled()

`scheduled()` hat folgende Signatur:

```typescript
function scheduled<T>(
  input: ObservableInput<T>,
  scheduler: SchedulerLike
): Observable<T>
```

### Unterstützte Eingabetypen

- **Array**: `T[]`
- **Promise**: `Promise<T>`
- **Iterable**: `Iterable<T>` (Set, Map, Generator usw.)
- **Observable**: `Observable<T>`
- **ArrayLike**: `ArrayLike<T>`

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Array
scheduled([1, 2, 3], asyncScheduler);

// Promise
scheduled(Promise.resolve('Ergebnis'), asyncScheduler);

// Set
scheduled(new Set([1, 2, 3]), asyncScheduler);

// Generator
function* generator() {
  yield 1;
  yield 2;
  yield 3;
}
scheduled(generator(), asyncScheduler);
```

## Häufige Fehler und Lösungen

### 1. Vergessen, Scheduler anzugeben

**Fehlerbeispiel:**
```typescript
// ❌ Fehler: Zweites Argument erforderlich
const observable$ = scheduled([1, 2, 3]);
```

**Lösung:**
```typescript
// ✅ Korrekt: Scheduler angeben
const observable$ = scheduled([1, 2, 3], asyncScheduler);
```

### 2. Verwendung von animationFrameScheduler in Browser-Umgebung

**Problem:**
In Node.js-Umgebungen existiert `requestAnimationFrame` nicht, was zu Fehlern führt.

**Lösung:**
```typescript
import { scheduled, animationFrameScheduler, asyncScheduler } from 'rxjs';

// Prüfung auf Browser-Umgebung
const scheduler = typeof window !== 'undefined'
  ? animationFrameScheduler
  : asyncScheduler;

const observable$ = scheduled([1, 2, 3], scheduler);
```

### 3. Verwechslung von synchroner und asynchroner Verarbeitung

**Problem:**
```typescript
// Erwartet asynchrone Ausführung, aber tatsächlich synchron
scheduled([1, 2, 3], queueScheduler).subscribe(val => {
  console.log(val);
});
console.log('Abgeschlossen'); // ← 1, 2, 3 werden davor ausgegeben
```

**Lösung:**
```typescript
// Explizit asynchron angeben
scheduled([1, 2, 3], asyncScheduler).subscribe(val => {
  console.log(val);
});
console.log('Abgeschlossen'); // ← 1, 2, 3 werden danach ausgegeben
```

## Vergleich mit from()

| Merkmal | from() | scheduled() |
|------|--------|-------------|
| Scheduler-Angabe | ❌ Nicht möglich (nur Standard) | ✅ Explizit angebbar |
| Synchron/Asynchron-Steuerung | ❌ Keine Steuerung | ✅ Steuerbar |
| Testbarkeit | Normal | ✅ Zeitsteuerung mit TestScheduler möglich |
| Einfachheit | ✅ Einfach | Etwas komplex |
| Verwendungsszenario | Grundlegende Konvertierung | Wenn Ausführungs-Timing-Steuerung erforderlich |

> [!TIP]
> **Auswahlkriterien**
>
> - **Grundsätzlich `from()` verwenden**: Wenn keine Scheduler-Steuerung erforderlich
> - **`scheduled()` verwenden bei**:
>   - UI-Blockierung vermeiden
>   - Zeitsteuerung für Tests erforderlich
>   - Animation-Implementierung
>   - Priorisierte Task-Verarbeitung

## Best Practices

### 1. asyncScheduler für Verarbeitung großer Datenmengen verwenden

```typescript
// ✅ Gutes Beispiel: UI nicht blockieren
scheduled(largeArray, asyncScheduler).pipe(
  map(processHeavyTask)
).subscribe();
```

### 2. TestScheduler in Tests verwenden

```typescript
// ✅ Gutes Beispiel: Zeit virtuell steuern
testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

### 3. animationFrameScheduler für Animationen verwenden

```typescript
// ✅ Gutes Beispiel: Mit Browser-Repaint-Timing synchronisieren
scheduled(frames, animationFrameScheduler).subscribe(updateUI);
```

### 4. Umgebungsabhängige Scheduler-Auswahl

```typescript
// ✅ Gutes Beispiel: Je nach Umgebung umschalten
const scheduler = process.env.NODE_ENV === 'test'
  ? queueScheduler
  : asyncScheduler;

const source$ = scheduled(data, scheduler);
```

## Zusammenfassung

`scheduled()` ist eine Creation Function, die Observables mit explizit angegebenem Scheduler erzeugt.

**Hauptmerkmale:**
- Explizite Steuerung des Ausführungs-Timings (synchron/asynchron)
- Auswahl aus mehreren Schedulern möglich
- Einfache Tests mit TestScheduler
- Effektiv zur Vermeidung von UI-Blockierungen

**Verwendungsszenarien:**
- Asynchrone Verarbeitung großer Datenmengen
- Animation-Implementierung
- Zeitsteuerung in Tests
- Priorisierte Task-Verarbeitung

**Wichtige Punkte:**
- Immer Scheduler angeben
- Je nach Umgebung geeigneten Scheduler wählen
- Unterschied zu from() verstehen

**Empfohlene Verwendung:**
- UI-Optimierung: `asyncScheduler`
- Animation: `animationFrameScheduler`
- Tests: `TestScheduler`
- Hohe Priorität: `asapScheduler`

## Verwandte Seiten

- [using()](/de/guide/creation-functions/control/using) - Observable mit Ressourcensteuerung
- [Steuerungs-Creation-Functions](/de/guide/creation-functions/control/) - Vergleich zwischen scheduled() und using()
- [Arten von Schedulern](/de/guide/schedulers/types) - Details zu Schedulern
- [from()](/de/guide/creation-functions/basic/from) - Grundlegende Observable-Erzeugung

## Referenzressourcen

- [RxJS Offizielle Dokumentation - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [RxJS Offizielle Dokumentation - Scheduler](https://rxjs.dev/guide/scheduler)
- [RxJS Offizielle Dokumentation - TestScheduler](https://rxjs.dev/api/testing/TestScheduler)
