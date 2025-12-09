---
description: Schleifengenerierungs Creation Functions - range und generate für for/while-ähnliche Wiederholungsverarbeitung als Observable-Streams mit TypeScript.
---

# Schleifengenerierungs Creation Functions

Creation Functions zur Darstellung von Schleifenverarbeitungen wie for- oder while-Schleifen als Observable.

## Was sind Schleifengenerierungs Creation Functions

Schleifengenerierungs Creation Functions ermöglichen die reaktive Implementierung von Wiederholungsverarbeitungen. Durch den Ersatz traditioneller imperativer Schleifen (`for`-Anweisungen, `while`-Anweisungen) durch deklarative Observable-Streams wird eine flexible Verarbeitung in Kombination mit RxJS-Operator-Ketten möglich.

In der folgenden Tabelle können Sie die Eigenschaften und Anwendungsfälle der einzelnen Creation Functions überprüfen.

## Wichtige Schleifengenerierungs Creation Functions

| Function | Beschreibung | Anwendungsfälle |
|----------|--------------|-----------------|
| **[range](/de/guide/creation-functions/loop/range)** | Generiert einen Zahlenbereich (wie for-Schleife) | Sequenzgenerierung, Stapelverarbeitung |
| **[generate](/de/guide/creation-functions/loop/generate)** | Universelle Schleifengenerierung (wie while-Schleife) | Bedingte Wiederholung, komplexe Zustandsübergänge |

## Auswahlkriterien

Die Auswahl von Schleifengenerierungs Creation Functions erfolgt nach folgenden Gesichtspunkten:

### 1. Generierungsmuster

- **Zahlenfolge**: `range()` - Einfache Sequenzgenerierung durch Angabe von Start- und Endwert
- **Komplexe Bedingungen**: `generate()` - Freie Kontrolle über Anfangswert, Bedingung, Iteration und Ergebnisauswahl

### 2. Schleifentyp

- **for-Schleifen-ähnlich**: `range()` - `for (let i = start; i <= end; i++)`
- **while-Schleifen-ähnlich**: `generate()` - `while (condition) { ... }`

### 3. Flexibilität

- **Einfach ausreichend**: `range()` - Wenn eine Zahlenfolge benötigt wird
- **Erweiterte Kontrolle erforderlich**: `generate()` - Benutzerdefinierte Zustandsverwaltung, bedingte Verzweigung, Schrittsteuerung

## Praktische Anwendungsbeispiele

### range() - Sequenzgenerierung

Für einfache Sequenzgenerierung ist `range()` optimal.

```typescript
import { range, map } from 'rxjs';
// Generiert Sequenz von 1 bis 5
range(1, 5).subscribe(console.log);
// Ausgabe: 1, 2, 3, 4, 5

// Anwendung in Stapelverarbeitung
range(0, 10).pipe(
  map(i => `Verarbeitung${i + 1}`)
).subscribe(console.log);
// Ausgabe: Verarbeitung1, Verarbeitung2, ..., Verarbeitung10
```

### generate() - Bedingte Schleife

Für komplexe Bedingungen oder benutzerdefinierte Zustandsverwaltung verwenden Sie `generate()`.

```typescript
import { generate } from 'rxjs';

// Fibonacci-Folge generieren (erste 10 Elemente)
generate(
  { current: 0, next: 1, count: 0 },  // Anfangszustand
  state => state.count < 10,           // Fortsetzungsbedingung
  state => ({                          // Zustandsaktualisierung
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Ergebnisauswahl
).subscribe(console.log);
// Ausgabe: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Vergleich mit imperativen Schleifen

Vergleich zwischen traditionellen imperativen Schleifen und RxJS Schleifengenerierungs Creation Functions.

### Imperative for-Schleife

```typescript
// Traditionelle for-Schleife
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Deklaratives range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Vorteile des deklarativen Ansatzes**:
> - Verbesserte Lesbarkeit durch Pipeline-Verarbeitung
> - Einheitliche Fehlerbehandlung
> - Einfache Kombination mit asynchroner Verarbeitung
> - Einfaches Abbrechen oder Unterbrechen (mit `takeUntil()` etc.)

## Umwandlung von Cold zu Hot

Wie in der obigen Tabelle gezeigt, **erzeugen alle Schleifengenerierungs Creation Functions Cold Observables**. Bei jeder Subscription beginnt eine unabhängige Ausführung.

Durch Verwendung von Multicasting-Operatoren (`share()`, `shareReplay()` etc.) können Sie jedoch **Cold Observables in Hot Observables umwandeln**.

### Praxisbeispiel: Berechnungsergebnisse teilen

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Unabhängige Berechnung pro Subscription
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Berechnung:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Subscriber1:', val));
cold$.subscribe(val => console.log('Subscriber2:', val));
// → Berechnung wird 2-mal ausgeführt (2000 Berechnungen)

// 🔥 Hot - Berechnungsergebnisse zwischen Subscribern teilen
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Berechnung:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Subscriber1:', val));
hot$.subscribe(val => console.log('Subscriber2:', val));
// → Berechnung wird nur 1-mal ausgeführt (1000 Berechnungen)
```

> [!TIP]
> **Fälle, in denen Hot-Umwandlung erforderlich ist**:
> - Teure Berechnungen an mehreren Stellen verwenden
> - Stapelverarbeitungsergebnisse über mehrere Komponenten teilen
> - Paginierungsergebnisse in mehreren UI-Komponenten anzeigen
>
> Weitere Details siehe [Basis-Erstellungs - Cold zu Hot Umwandlung](/de/guide/creation-functions/basic/#cold-zu-hot-umwandlung).

## Kombination mit asynchroner Verarbeitung

Schleifengenerierungs Creation Functions entfalten ihre volle Leistungsfähigkeit in Kombination mit asynchroner Verarbeitung.

### Sequenzielle API-Aufrufe

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
    items: [`Daten${page}-1`, `Daten${page}-2`, `Daten${page}-3`]
  }).pipe(
    delay(300) // Simuliert API-Aufruf
  );
}

// Seiten 1 bis 10 sequenziell abrufen (mit 1 Sekunde Verzögerung zwischen Anfragen)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Seite ${data.page} abgerufen:`, data.items),
  err => console.error('Fehler:', err)
);
```

### Anwendung in Retry-Logik

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Funktion zur Simulation des Datenabrufs (scheitert zufällig)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40% Erfolgswahrscheinlichkeit

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Datenabruf fehlgeschlagen'))
        : of('Datenabruf erfolgreich')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Maximal 3 Wiederholungen
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Wiederholung ${index + 1}/3`);
          // Exponentieller Backoff: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Ergebnis:', result),
  error: err => console.error('Fehler:', err.message)
});

// Beispielausgabe:
// Wiederholung 1/3
// Wiederholung 2/3
// Ergebnis: Datenabruf erfolgreich
```

## Beziehung zu Pipeable Operators

Schleifengenerierungs Creation Functions haben keine direkten Entsprechungen als Pipeable Operators. Sie werden immer als Creation Functions verwendet.

Durch Kombination mit folgenden Operatoren sind jedoch erweiterte Verarbeitungen möglich:

| Kombinations-Operator | Zweck |
|----------------------|-------|
| `map()` | Jeden Wert transformieren |
| `filter()` | Nur Werte durchlassen, die Bedingung erfüllen |
| `take()`, `skip()` | Anzahl der Werte kontrollieren |
| `concatMap()`, `mergeMap()` | Asynchrone Verarbeitung für jeden Wert |
| `toArray()` | Alle Werte in Array sammeln |

## Performance-Hinweise

Schleifengenerierungs Creation Functions emittieren Werte synchron, daher ist bei großen Datenmengen Vorsicht geboten.

> [!WARNING]
> **Umgang mit großen Datenmengen**:
> - `range(1, 1000000)` emittiert alle Werte synchron und verbraucht Speicher
> - Bei Bedarf mit `bufferCount()` oder `windowCount()` puffern
> - Oder `scheduled()` verwenden, um asynchrone Ausführung mit Scheduler anzugeben

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Mit asynchronem Scheduler ausführen
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Nächste Schritte

Um die detaillierte Funktionsweise und praktischen Beispiele jeder Creation Function zu lernen, klicken Sie auf die Links in der obigen Tabelle.

Durch das Studium von [Basis-Erstellungs Creation Functions](/de/guide/creation-functions/basic/), [Kombinations Creation Functions](/de/guide/creation-functions/combination/), [Auswahl/Partitionierungs Creation Functions](/de/guide/creation-functions/selection/) und [Bedingte Verzweigungs Creation Functions](/de/guide/creation-functions/conditional/) können Sie ein Gesamtverständnis der Creation Functions erlangen.
