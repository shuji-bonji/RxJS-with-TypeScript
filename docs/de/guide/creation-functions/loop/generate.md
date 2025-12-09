---
description: "generate() ermöglicht while/for-ähnliche Schleifen mit Zustand, Bedingung und Iteration. Ideal für Fibonacci, Backoff und komplexe Sequenzen."
---

# generate() - Universelle Schleifengenerierung

`generate()` ist eine Creation Function zur Implementierung flexibler Schleifenverarbeitung als Observable durch Angabe von Anfangszustand, Fortsetzungsbedingung, Zustandsaktualisierung und Ergebnisauswahl.

## Übersicht

`generate()` ermöglicht die deklarative Beschreibung flexibler Schleifenverarbeitungen wie while- oder for-Schleifen. Wird verwendet, wenn komplexere Bedingungen oder Zustandsverwaltung als bei `range()` erforderlich sind.

**Signatur**:
```typescript
function generate<T, S>(
  initialState: S,
  condition: (state: S) => boolean,
  iterate: (state: S) => S,
  resultSelector?: (state: S) => T,
  scheduler?: SchedulerLike
): Observable<T>
```

**Parameter**:
- `initialState`: Anfangszustand der Schleife
- `condition`: Funktion zur Prüfung der Fortsetzungsbedingung (Schleife endet bei `false`)
- `iterate`: Funktion zum Übergang zum nächsten Zustand (Zustandsaktualisierung)
- `resultSelector`: Funktion zur Auswahl des zu emittierenden Werts aus dem Zustand (bei Auslassung wird Zustand selbst emittiert)
- `scheduler`: Scheduler für die Wertemission (bei Auslassung synchron)

**Offizielle Dokumentation**: [📘 RxJS Offiziell: generate()](https://rxjs.dev/api/index/function/generate)

## Grundlegende Verwendung

### Muster 1: Einfacher Zähler

Grundlegendste Verwendung.

```typescript
import { generate } from 'rxjs';

// Zählt von 1 bis 5
generate(
  1,              // Anfangszustand
  x => x <= 5,    // Fortsetzungsbedingung
  x => x + 1      // Zustandsaktualisierung
).subscribe({
  next: value => console.log('Wert:', value),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe:
// Wert: 1
// Wert: 2
// Wert: 3
// Wert: 4
// Wert: 5
// Abgeschlossen
```

Dieser Code ist äquivalent zu folgender while-Schleife:

```typescript
let x = 1;
while (x <= 5) {
  console.log('Wert:', x);
  x = x + 1;
}
console.log('Abgeschlossen');
```

### Muster 2: Werttransformation mit resultSelector

Zustand und emittierter Wert können getrennt werden.

```typescript
import { generate } from 'rxjs';

// Interner Zustand ist Zähler, aber emittiert wird der quadrierte Wert
generate(
  1,              // Anfangszustand: 1
  x => x <= 5,    // Fortsetzungsbedingung: x <= 5
  x => x + 1,     // Zustandsaktualisierung: x + 1
  x => x * x      // Ergebnisauswahl: emittiert x^2
).subscribe(console.log);

// Ausgabe: 1, 4, 9, 16, 25
```

### Muster 3: Komplexes Zustandsobjekt

Als Zustand kann ein komplexes Objekt verwendet werden.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

// Berechnet kumulative Summe
generate<number, State>(
  { count: 1, sum: 0 },           // Anfangszustand
  state => state.count <= 5,      // Fortsetzungsbedingung
  state => ({                     // Zustandsaktualisierung
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => state.sum              // Ergebnisauswahl
).subscribe(console.log);

// Ausgabe: 0, 1, 3, 6, 10
// (0, 0+1, 0+1+2, 0+1+2+3, 0+1+2+3+4)
```

## Wichtige Eigenschaften

### 1. While-Schleifen-ähnliches Verhalten

`generate()` ermöglicht flexible Kontrolle wie eine while-Schleife.

```typescript
import { generate } from "rxjs";

// while-Schleife
let i = 1;
while (i <= 10) {
  console.log(i);
  i = i * 2;
}

// Gleiche Funktionalität mit generate()
generate(
  1,              // let i = 1;
  i => i <= 10,   // while (i <= 10)
  i => i * 2      // i = i * 2;
).subscribe(console.log);

// Ausgabe: 1, 2, 4, 8
```

### 2. Synchrone Emission

Standardmäßig werden alle Werte **synchron** unmittelbar nach Subscription emittiert.

```typescript
import { generate } from 'rxjs';

console.log('Vor Subscription');

generate(1, x => x <= 3, x => x + 1).subscribe(val => console.log('Wert:', val));

console.log('Nach Subscription');

// Ausgabe:
// Vor Subscription
// Wert: 1
// Wert: 2
// Wert: 3
// Nach Subscription
```

### 3. Vorsicht vor Endlosschleifen

Wenn die Bedingung immer `true` ist, entsteht eine Endlosschleife.

```typescript
import { generate, take } from 'rxjs';
// ❌ Gefährlich: Endlosschleife (Browser friert ein)
// generate(0, x => true, x => x + 1).subscribe(console.log);

// ✅ Sicher: Anzahl mit take() begrenzen
generate(
  0,
  x => true,  // Immer true
  x => x + 1
).pipe(
  take(10)    // Nur erste 10 holen
).subscribe(console.log);

// Ausgabe: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

> [!WARNING]
> **Vorsicht vor Endlosschleifen**:
> - Bei Bedingungen, die immer `true` werden, entsteht Endlosschleife
> - Emissionszahl mit `take()`, `takeWhile()`, `takeUntil()` begrenzen
> - Oder angemessene Abbruchbedingung in Bedingungsfunktion setzen

## Praktische Anwendungsfälle

### 1. Fibonacci-Folge

Beispiel für komplexe Zustandsübergänge.

```typescript
import { generate, take } from 'rxjs';
interface FibState {
  current: number;
  next: number;
}

// Erste 10 Glieder der Fibonacci-Folge
generate<number, FibState>(
  { current: 0, next: 1 },           // Anfangszustand: F(0)=0, F(1)=1
  state => true,                     // Unendlich generieren
  state => ({                        // Zustandsaktualisierung
    current: state.next,
    next: state.current + state.next
  }),
  state => state.current             // Aktuellen Wert emittieren
).pipe(
  take(10)                           // Erste 10 Glieder
).subscribe(console.log);

// Ausgabe: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

### 2. Exponentieller Backoff

Generierung exponentieller Wartezeiten für Retry-Verarbeitung.

```typescript
import { generate } from 'rxjs';

interface RetryState {
  attempt: number;
  delay: number;
}

// Generiert Backoff-Verzögerungszeiten (1s, 2s, 4s, 8s, 16s)
generate<number, RetryState>(
  { attempt: 0, delay: 1000 },       // Anfangszustand: 1s
  state => state.attempt < 5,        // Maximal 5 Mal
  state => ({                        // Zustandsaktualisierung
    attempt: state.attempt + 1,
    delay: state.delay * 2           // Verzögerungszeit verdoppeln
  }),
  state => state.delay               // Verzögerungszeit emittieren
).subscribe(delay => {
  console.log(`Wiederholung nach ${delay / 1000} Sekunden`);
});

// Ausgabe:
// Wiederholung nach 1 Sekunden
// Wiederholung nach 2 Sekunden
// Wiederholung nach 4 Sekunden
// Wiederholung nach 8 Sekunden
// Wiederholung nach 16 Sekunden
```

### 3. Paginierungskontrolle

Abruf wird fortgesetzt, solange nächste Seite existiert.

```typescript
import { generate, of, Observable, concatMap, delay } from 'rxjs';
interface PageState {
  page: number;
  hasNext: boolean;
}

interface PageData {
  page: number;
  items: string[];
  hasNext: boolean;
}

// Funktion zur Simulation des Seitenabrufs
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Element${page}-1`, `Element${page}-2`, `Element${page}-3`],
    hasNext: page < 10 // Bis Seite 10
  }).pipe(
    delay(500) // Simuliert API-Aufruf
  );
}

// Abruf solange Seite existiert (in Praxis hasNext aus API-Response holen)
generate<number, PageState>(
  { page: 1, hasNext: true },        // Anfangszustand
  state => state.hasNext,            // Solange nächste Seite existiert
  state => ({                        // Zustandsaktualisierung
    page: state.page + 1,
    hasNext: state.page < 10         // Angenommen bis Seite 10
  }),
  state => state.page                // Seitennummer emittieren
).pipe(
  concatMap(page => fetchPage(page)) // Jede Seite sequenziell abrufen
).subscribe(
  data => console.log(`Seite ${data.page} abgerufen:`, data.items),
  err => console.error('Fehler:', err),
  () => console.log('Alle Seiten abgerufen')
);

// Ausgabe:
// Seite 1 abgerufen: ['Element1-1', 'Element1-2', 'Element1-3']
// Seite 2 abgerufen: ['Element2-1', 'Element2-2', 'Element2-3']
// ...
// Seite 10 abgerufen: ['Element10-1', 'Element10-2', 'Element10-3']
// Alle Seiten abgerufen
```

### 4. Benutzerdefinierter Timer

Emittiert Ereignisse in unregelmäßigen Intervallen.

```typescript
import { generate, of, concatMap, delay } from 'rxjs';
interface TimerState {
  count: number;
  delay: number;
}

// Timer mit graduell zunehmender Verzögerung
generate<string, TimerState>(
  { count: 0, delay: 1000 },         // Anfangszustand: 1s
  state => state.count < 5,          // Bis 5 Mal
  state => ({                        // Zustandsaktualisierung
    count: state.count + 1,
    delay: state.delay + 500         // Verzögerung um 500ms erhöhen
  }),
  state => `Ereignis${state.count + 1}`
).pipe(
  concatMap((message, index) => {
    const delayTime = 1000 + index * 500;
    console.log(`Emission nach ${delayTime}ms Wartezeit`);
    return of(message).pipe(delay(delayTime));
  })
).subscribe(console.log);

// Ausgabe:
// Emission nach 1000ms Wartezeit
// Ereignis1 (nach 1s)
// Emission nach 1500ms Wartezeit
// Ereignis2 (nach 2.5s)
// Emission nach 2000ms Wartezeit
// Ereignis3 (nach 4.5s)
// ...
```

### 5. Fakultätsberechnung

Mathematische Berechnungen als Stream ausdrücken.

```typescript
import { generate } from 'rxjs';

interface FactorialState {
  n: number;
  result: number;
}

// Berechnet 5! (5! = 5 × 4 × 3 × 2 × 1 = 120)
generate<number, FactorialState>(
  { n: 5, result: 1 },               // Anfangszustand
  state => state.n > 0,              // Solange n > 0
  state => ({                        // Zustandsaktualisierung
    n: state.n - 1,
    result: state.result * state.n
  }),
  state => state.result              // Zwischenergebnisse emittieren
).subscribe(console.log);

// Ausgabe: 5, 20, 60, 120, 120
// (1*5, 5*4, 20*3, 60*2, 120*1)
```

## Vergleich mit anderen Creation Functions

### generate() vs range()

```typescript
import { generate, range } from 'rxjs';

// range() - Einfache Sequenznummern
range(1, 5).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// generate() - Gleiche Funktionalität expliziter
generate(
  1,
  x => x <= 5,
  x => x + 1
).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// Wahre Stärke von generate(): Komplexe Schritte
generate(
  1,
  x => x <= 100,
  x => x * 2  // Verdoppelt bei jeder Iteration
).subscribe(console.log);
// Ausgabe: 1, 2, 4, 8, 16, 32, 64
// (mit range() nicht möglich)
```

### generate() vs defer()

```typescript
import { generate, defer, of } from 'rxjs';

// generate() - Schleifenverarbeitung
generate(1, x => x <= 3, x => x + 1).subscribe(console.log);
// Ausgabe: 1, 2, 3

// defer() - Generierung bei Subscription (keine Schleife)
defer(() => of(1, 2, 3)).subscribe(console.log);
// Ausgabe: 1, 2, 3

// Unterschied: generate() hat Zustand, defer nur verzögerte Auswertung
```

> [!TIP]
> **Auswahlkriterien**:
> - **Einfache Sequenznummern** → `range()`
> - **Komplexe Bedingungen oder Schritte** → `generate()`
> - **Dynamische Entscheidung bei Subscription** → `defer()`
> - **Fibonacci, Fakultät etc.** → `generate()`

## Asynchronisierung mit Scheduler

Bei der Verarbeitung großer Datenmengen kann asynchrone Ausführung durch Angabe eines Schedulers erfolgen.

```typescript
import { generate, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// 1 Million Schleifendurchläufe asynchron
generate(
  1,
  x => x <= 1000000,
  x => x + 1
).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Fortschritt: ${val}`);
    }
  },
  complete: () => console.log('Abgeschlossen')
});

console.log('Nach Subscription (wird sofort ausgeführt da asynchron)');

// Ausgabe:
// Start
// Nach Subscription (wird sofort ausgeführt da asynchron)
// Fortschritt: 100000
// Fortschritt: 200000
// ...
// Abgeschlossen
```

## Performance-Hinweise

`generate()` emittiert Werte synchron, daher ist bei großen Datenmengen oder komplexen Berechnungen Vorsicht geboten.

> [!WARNING]
> **Performance-Optimierung**:
> ```typescript
> // ❌ Schlechtes Beispiel: Komplexe Berechnung synchron (UI wird blockiert)
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).subscribe(console.log);
>
> // ✅ Gutes Beispiel 1: Asynchronisierung mit Scheduler
> generate(
>   1,
>   x => x <= 1000000,
>   x => expensiveCalculation(x)
> ).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Gutes Beispiel 2: Anzahl mit take() begrenzen
> generate(
>   1,
>   x => true,  // Endlosschleife
>   x => x + 1
> ).pipe(
>   take(100)   // Nur erste 100
> ).subscribe(console.log);
> ```

## Fehlerbehandlung

`generate()` selbst emittiert keine Fehler, aber in Pipeline oder Zustandsaktualisierungsfunktion können Fehler auftreten.

```typescript
import { generate, of, map, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => x + 1
).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Fehler bei 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Fehler aufgetreten:', error.message);
    return of(-1); // Standardwert zurückgeben
  })
).subscribe(console.log);

// Ausgabe: 2, 4, 6, 8, -1
```

### Fehler in Zustandsaktualisierungsfunktion

Tritt ein Fehler in der Zustandsaktualisierungsfunktion auf, geht das Observable in Fehlerzustand.

```typescript
import { generate, EMPTY, catchError } from 'rxjs';
generate(
  1,
  x => x <= 10,
  x => {
    if (x === 5) {
      throw new Error('Fehler bei Zustandsaktualisierung');
    }
    return x + 1;
  }
).pipe(
  catchError(error => {
    console.error('Fehler:', error.message);
    return EMPTY; // Leeres Observable zurückgeben
  })
).subscribe({
  next: console.log,
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe: 1, 2, 3, 4, Fehler: Fehler bei Zustandsaktualisierung, Abgeschlossen
```

## Typsicherheit mit TypeScript

`generate()` kann Zustandstyp und emittierten Werttyp trennen.

```typescript
import { generate } from 'rxjs';

interface State {
  count: number;
  sum: number;
}

interface Result {
  index: number;
  average: number;
}

// Zustand: State, emittierter Wert: Result
const stats$ = generate<Result, State>(
  { count: 1, sum: 0 },
  state => state.count <= 5,
  state => ({
    count: state.count + 1,
    sum: state.sum + state.count
  }),
  state => ({
    index: state.count,
    average: state.sum / state.count
  })
);

stats$.subscribe(result => {
  console.log(`[${result.index}] Durchschnitt: ${result.average}`);
});

// Ausgabe:
// [1] Durchschnitt: 0
// [2] Durchschnitt: 0.5
// [3] Durchschnitt: 1
// [4] Durchschnitt: 1.5
// [5] Durchschnitt: 2
```

## Zusammenfassung

`generate()` ist eine leistungsstarke Creation Function zur deklarativen Beschreibung komplexer Schleifenverarbeitungen.

> [!IMPORTANT]
> **Eigenschaften von generate()**:
> - ✅ Flexible Schleifenkontrolle wie while/for-Schleife
> - ✅ Komplexe Zustandsverwaltung möglich
> - ✅ Optimal für mathematische Berechnungen wie Fibonacci, Fakultät
> - ✅ Zustand und emittierter Wert können getrennt werden
> - ⚠️ Vorsicht vor Endlosschleifen (mit `take()` begrenzen)
> - ⚠️ Bei großen Datenmengen Asynchronisierung erwägen
> - ⚠️ Für einfache Sequenznummern `range()` verwenden

## Verwandte Themen

- [range()](/de/guide/creation-functions/loop/range) - Einfache Sequenzgenerierung
- [defer()](/de/guide/creation-functions/conditional/defer) - Dynamische Generierung bei Subscription
- [expand()](/de/guide/operators/transformation/expand) - Rekursive Expansion (Higher-Order-Operator)
- [scan()](/de/guide/operators/transformation/scan) - Kumulative Berechnung

## Referenzressourcen

- [RxJS Offiziell: generate()](https://rxjs.dev/api/index/function/generate)
- [Learn RxJS: generate](https://www.learnrxjs.io/learn-rxjs/operators/creation/generate)
