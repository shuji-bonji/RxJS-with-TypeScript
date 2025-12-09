---
description: "range() emittiert aufeinanderfolgende Ganzzahlen ab Startwert. Deklarative for-Schleifen-Alternative für Sequenzen, Stapelverarbeitung und Tests."
---

# range() - Zahlenbereichsgenerierung

`range()` ist eine for-Schleifen-ähnliche Creation Function, die eine bestimmte Anzahl aufeinanderfolgender Ganzzahlen ab einem Startwert emittiert.

## Übersicht

`range()` emittiert aufeinanderfolgende Ganzzahlen als Observable, indem Sie einen Startwert und eine Anzahl angeben. Als deklarative Alternative zur traditionellen `for`-Schleife wird es für Sequenzgenerierung und Stapelverarbeitung verwendet.

**Signatur**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parameter**:
- `start`: Startwert (beginnt mit diesem Wert)
- `count`: Anzahl der zu emittierenden Werte (bei Auslassung von 0 bis unterhalb von `start`)
- `scheduler`: Scheduler für die Wertemission (bei Auslassung synchron)

**Offizielle Dokumentation**: [📘 RxJS Offiziell: range()](https://rxjs.dev/api/index/function/range)

## Grundlegende Verwendung

### Muster 1: Startwert und Anzahl angeben

Die häufigste Verwendungsweise.

```typescript
import { range } from 'rxjs';

// Generiert 5 Sequenznummern ab 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
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

### Muster 2: Sequenz ab 0

Mit Startwert 0 können Sie Sequenznummern wie Array-Indizes generieren.

```typescript
import { range } from 'rxjs';

// 10 Sequenznummern ab 0 (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Ausgabe: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Muster 3: Start mit negativen Zahlen

Auch negative Zahlen sind als Startwert möglich.

```typescript
import { range } from 'rxjs';

// 5 Sequenznummern ab -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Ausgabe: -3, -2, -1, 0, 1
```

## Wichtige Eigenschaften

### 1. Synchrone Emission

`range()` emittiert standardmäßig alle Werte **synchron** unmittelbar nach der Subscription.

```typescript
import { range } from 'rxjs';

console.log('Vor Subscription');

range(1, 3).subscribe(value => console.log('Wert:', value));

console.log('Nach Subscription');

// Ausgabe:
// Vor Subscription
// Wert: 1
// Wert: 2
// Wert: 3
// Nach Subscription
```

### 2. Sofortiger Abschluss

Nach Emission aller Werte wird sofort `complete` benachrichtigt.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Abgeschlossen!')
});

// Ausgabe: 1, 2, 3, Abgeschlossen!
```

### 3. Äquivalenz zur for-Schleife

`range(start, count)` ist äquivalent zu folgender for-Schleife:

```typescript
// Imperative for-Schleife
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// Deklaratives range()
range(start, count).subscribe(console.log);
```

## Praktische Anwendungsfälle

### 1. Stapelverarbeitung

Wird bei sequenzieller Ausführung mehrerer Tasks verwendet.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Funktion zur Simulation der Datenverarbeitung
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simuliert 100ms Verarbeitungszeit
    map(i => `Verarbeitungsergebnis von Element${i}`)
  );
}

// 10 Daten sequenziell verarbeiten (mit 1 Sekunde Verzögerung zwischen Verarbeitungen)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Verarbeitung abgeschlossen: ${result}`),
  complete: () => console.log('Alle Verarbeitungen abgeschlossen')
});

// Ausgabe:
// Verarbeitung abgeschlossen: Verarbeitungsergebnis von Element1 (nach ca. 1.1s)
// Verarbeitung abgeschlossen: Verarbeitungsergebnis von Element2 (nach ca. 2.1s)
// ...
// Verarbeitung abgeschlossen: Verarbeitungsergebnis von Element10 (nach ca. 10.1s)
// Alle Verarbeitungen abgeschlossen
```

### 2. Paginierung

Sequenzieller Abruf von Daten mehrerer Seiten.

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Funktion zur Simulation des Seitenabrufs
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Element${page}-1`, `Element${page}-2`, `Element${page}-3`]
  }).pipe(
    delay(500) // Simuliert API-Aufruf
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Seite ${data.page}:`, data.items),
  complete: () => console.log('Alle Seiten abgerufen')
});

// Ausgabe:
// Seite 1: ['Element1-1', 'Element1-2', 'Element1-3']
// Seite 2: ['Element2-1', 'Element2-2', 'Element2-3']
// Seite 3: ['Element3-1', 'Element3-2', 'Element3-3']
// Seite 4: ['Element4-1', 'Element4-2', 'Element4-3']
// Seite 5: ['Element5-1', 'Element5-2', 'Element5-3']
// Alle Seiten abgerufen
```

### 3. Array-Index-Verarbeitung

Verwendung als indexbasierte Schleife bei der Verarbeitung jedes Array-Elements.

```typescript
import { range, map } from 'rxjs';
const items = ['Apfel', 'Banane', 'Kirsche', 'Dattel', 'Holunder'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Ausgabe:
// [0] Apfel
// [1] Banane
// [2] Kirsche
// [3] Dattel
// [4] Holunder
```

### 4. Testdatengenerierung

Praktisch bei der Generierung von Mock-Daten für Unit-Tests.

```typescript
import { range, map, toArray } from 'rxjs';
// Mock-Generierung von Benutzerdaten
range(1, 100).pipe(
  map(id => ({
    id,
    name: `User${id}`,
    email: `user${id}@example.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} Benutzer generiert`);
  // In Tests verwenden
});
```

### 5. Retry-Zähler

Kontrolle der Anzahl der Wiederholungen bei Fehlern.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Funktion zur Simulation des Datenabrufs (scheitert zufällig)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% Erfolgswahrscheinlichkeit

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Datenabruf fehlgeschlagen'))
        : of('Datenabruf erfolgreich')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Versuch ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError(error => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Maximale Wiederholungen erreicht'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Exponentieller Backoff
        );
      }),
      toArray()
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Ergebnis:', result),
  error: err => console.error('Fehler:', err.message)
});

// Beispielausgabe:
// Versuch 1/3
// Versuch 2/3
// Ergebnis: Datenabruf erfolgreich
```

## Asynchronisierung mit Scheduler

Bei der Verarbeitung großer Datenmengen kann asynchrone Ausführung durch Angabe eines Schedulers erfolgen.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Start');

// 1 Million Zahlen asynchron emittieren
range(1, 1000000).pipe(
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

> [!TIP]
> **Scheduler-Nutzung**:
> - Verhindert UI-Blockierung bei großen Datenmengen
> - Zeitkontrolle in Tests (TestScheduler)
> - Event-Loop-Kontrolle in Node.js-Umgebung

Weitere Details siehe [Scheduler-Typen und Unterscheidung](/de/guide/schedulers/types).

## Vergleich mit anderen Creation Functions

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - Aufeinanderfolgende Ganzzahlen
range(1, 3).subscribe(console.log);
// Ausgabe: 1, 2, 3

// of() - Beliebige Werte aufzählen
of(1, 2, 3).subscribe(console.log);
// Ausgabe: 1, 2, 3

// Unterschied: range() nur Sequenznummern, of() kann beliebige Werte angeben
of(1, 10, 100).subscribe(console.log);
// Ausgabe: 1, 10, 100 (mit range() nicht möglich)
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - Generiert Sequenznummern
range(1, 5).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// from() - Aus Array generieren (Array muss vorher erstellt werden)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// Vorteil von range(): Kein vorheriges Array im Speicher erforderlich
range(1, 1000000); // Speichereffizient
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Array im Speicher
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - Einfache Sequenznummern
range(1, 5).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// generate() - Komplexere Schreibweise für das Gleiche
generate(
  1,                    // Anfangswert
  x => x <= 5,          // Fortsetzungsbedingung
  x => x + 1            // Iteration
).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// Vorteil von generate(): Komplexe Bedingungen und Zustandsverwaltung möglich
generate(
  1,
  x => x <= 100,
  x => x * 2  // Verdoppelt bei jeder Iteration
).subscribe(console.log);
// Ausgabe: 1, 2, 4, 8, 16, 32, 64
// (mit range() nicht möglich)
```

> [!TIP]
> **Auswahlkriterien**:
> - **Sequenznummern benötigt** → `range()`
> - **Beliebige Werte aufzählen** → `of()`
> - **Vorhandenes Array/Promise** → `from()`
> - **Komplexe Bedingungen/Schritte** → `generate()`

## Performance-Hinweise

`range()` emittiert Werte synchron, daher ist bei großen Datenmengen Vorsicht geboten.

> [!WARNING]
> **Umgang mit großen Datenmengen**:
> ```typescript
> // ❌ Schlechtes Beispiel: 1 Million Werte synchron emittiert (UI wird blockiert)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Gutes Beispiel 1: Asynchronisierung mit Scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Gutes Beispiel 2: Aufteilung durch Pufferung
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} Elemente verarbeitet`));
> ```

## Unterscheidung zu from()-Array

```typescript
import { range, from } from 'rxjs';

// Wenn Sequenznummern benötigt werden → range() ist prägnant
range(0, 10).subscribe(console.log);

// Kein Bedarf, erst Array zu erstellen und dann umzuwandeln (ineffizient)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// Wenn vorhandenes Array existiert → from() verwenden
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Fehlerbehandlung

`range()` selbst emittiert keine Fehler, aber in der Pipeline können Fehler auftreten.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
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

## Zusammenfassung

`range()` ist eine einfache und leistungsstarke Creation Function zur Generierung aufeinanderfolgender Ganzzahlen.

> [!IMPORTANT]
> **Eigenschaften von range()**:
> - ✅ Optimal für Sequenzgenerierung (Alternative zu for-Schleife)
> - ✅ Praktisch für Stapelverarbeitung, Paginierung, Testdatengenerierung
> - ✅ Speichereffizient (kein vorheriges Array-Erstellen)
> - ⚠️ Bei großen Datenmengen Asynchronisierung erwägen
> - ⚠️ Für komplexe Bedingungen `generate()` verwenden

## Verwandte Themen

- [generate()](/de/guide/creation-functions/loop/generate) - Universelle Schleifengenerierung
- [of()](/de/guide/creation-functions/basic/of) - Beliebige Werte aufzählen
- [from()](/de/guide/creation-functions/basic/from) - Aus Array oder Promise umwandeln
- [interval()](/de/guide/creation-functions/basic/interval) - Periodische Wertemission

## Referenzressourcen

- [RxJS Offiziell: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
