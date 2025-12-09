---
description: "from() - Erstellungsfunktion zum Erstellen von Observables aus Arrays, Promises, Iterables, Strings, etc., einfaches Streamen von bestehenden Datenstrukturen, typsichere Konvertierungsmuster in TypeScript, async/await Kombination und Unterschiede zu of() werden ausführlich erklärt."
---

# from() - Konvertieren von Array, Promise, etc.

`from()` ist eine Erstellungsfunktion, die ein Observable aus einem Array, Promise, Iterable oder einem Observable-ähnlichen Objekt erzeugt.

## Überblick

`from()` konvertiert eine bestehende Datenstruktur (Array, Promise, Iterable, etc.) in einen Observable Stream. Insbesondere wird es häufig verwendet, um asynchrone Verarbeitung (Promise) in die RxJS-Welt zu integrieren.

**Signatur**:
```typescript
function from<T>(input: ObservableInput<T>, scheduler?: SchedulerLike): Observable<T>
```

**Offizielle Dokumentation**: [📘 RxJS Official: from()](https://rxjs.dev/api/index/function/from)

## Grundlegende Verwendung

`from()` akzeptiert eine Vielzahl von Eingabetypen.

```typescript
import { from } from 'rxjs';

// Aus Array erstellen
const array$ = from([1, 2, 3]);
array$.subscribe({
  next: value => console.log('Array-Wert:', value),
  complete: () => console.log('Array abgeschlossen')
});

// Aus Promise erstellen
const promise$ = from(Promise.resolve('Promise-Ergebnis'));
promise$.subscribe({
  next: value => console.log('Promise-Ergebnis:', value),
  complete: () => console.log('Promise abgeschlossen')
});

// Aus Iterable erstellen
const iterable$ = from(new Set([1, 2, 3]));
iterable$.subscribe({
  next: value => console.log('Iterable-Wert:', value),
  complete: () => console.log('Iterable abgeschlossen')
});

// Ausgabe:
// Array-Wert: 1
// Array-Wert: 2
// Array-Wert: 3
// Array abgeschlossen
// Iterable-Wert: 1
// Iterable-Wert: 2
// Iterable-Wert: 3
// Iterable abgeschlossen
// Promise-Ergebnis: Promise-Ergebnis
// Promise abgeschlossen
```

## Wichtige Eigenschaften

### 1. Jedes Element des Arrays wird separat ausgegeben

Wenn `from()` ein Array empfängt, gibt es jedes Element des Arrays einzeln nacheinander aus.

```typescript
import { from } from 'rxjs';

from([10, 20, 30]).subscribe(value => console.log(value));

// Ausgabe:
// 10
// 20
// 30
```

> [!IMPORTANT]
> **Unterschiede zu `of()`**:
> - `of([1, 2, 3])` → gibt das Array selbst als einen einzigen Wert aus
> - `from([1, 2, 3])` → gibt jedes Element `1`, `2`, `3` einzeln aus

### 2. Automatisch Promise verarbeiten

Bei der Übergabe eines Promise wird der aufgelöste Wert ausgegeben und sofort vervollständigt.

```typescript
import { from } from 'rxjs';

const fetchData = (): Promise<string> => {
  return new Promise(resolve => {
    setTimeout(() => resolve('Daten erfolgreich abgerufen'), 1000);
  });
};

from(fetchData()).subscribe({
  next: value => console.log(value),
  complete: () => console.log('Abgeschlossen')
});

// Nach 1 Sekunde Ausgabe:
// Daten erfolgreich abgerufen
// Abgeschlossen
```

> [!WARNING]
> Wenn das Promise abgelehnt wird, gibt Observable einen Fehler aus.
> ```typescript
> import { from } from "rxjs";
> from(Promise.reject('Fehler')).subscribe({
>   error: err => console.error('Fehler aufgetreten:', err)
> });
> ```

### 3. Unterstützt Iterable

Neben Arrays unterstützt es iterable Objekte wie `Set`, `Map` und `Generator`.

```typescript
import { from } from 'rxjs';

// Set
from(new Set(['A', 'B', 'C'])).subscribe(console.log);
// Ausgabe: A, B, C

// Map (Schlüssel-Wert-Paare)
from(new Map([['key1', 'value1'], ['key2', 'value2']])).subscribe(console.log);
// Ausgabe: ['key1', 'value1'], ['key2', 'value2']

// Generator
function* numberGenerator() {
  yield 1;
  yield 2;
  yield 3;
}
from(numberGenerator()).subscribe(console.log);
// Ausgabe: 1, 2, 3
```

### 4. Cold Observable

`from()` ist ein **Cold Observable**. Jedes Abonnement initiiert eine unabhängige Ausführung.

```typescript
import { from } from 'rxjs';

const numbers$ = from([1, 2, 3]);

numbers$.subscribe(val => console.log('Abonnent A:', val));
numbers$.subscribe(val => console.log('Abonnent B:', val));

// Jeder Abonnent verarbeitet das Array unabhängig
// Ausgabe:
// Abonnent A: 1
// Abonnent A: 2
// Abonnent A: 3
// Abonnent B: 1
// Abonnent B: 2
// Abonnent B: 3
```

> [!NOTE]
> **Cold Observable Merkmale**
> - Jedes Abonnement startet eine unabhängige Ausführung
> - Jeder Abonnent erhält seinen eigenen Datenstrom
> - Promise wird auch pro Abonnement ausgewertet
>
> Weitere Informationen finden Sie unter [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

## Unterschied zwischen from() vs. of()

Der wichtigste Unterschied zwischen den beiden ist die Art und Weise, wie Arrays behandelt werden.

```typescript
import { from, of } from 'rxjs';

const array = [1, 2, 3];

// of() - Array wird als ein Wert ausgegeben
of(array).subscribe(value => {
  console.log('of():', value); // [1, 2, 3]
});

// from() - Jedes Element des Arrays wird einzeln ausgegeben
from(array).subscribe(value => {
  console.log('from():', value); // 1, 2, 3
});
```

| Erstellungsfunktion | Array-Behandlung | Verwendung |
|-------------------|-----------|------|
| `of([1, 2, 3])` | Array selbst veröffentlichen | Array als Daten behandeln |
| `from([1, 2, 3])` | Jedes Element einzeln veröffentlichen | Array-Elemente einzeln verarbeiten |

## Praktische Anwendungsfälle

### 1. Streaming-API-Aufrufe

Stream Promise-basierte HTTP-Clients wie die Fetch-API und axios.

```typescript
import { from, Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

interface User {
  id: number;
  name: string;
  email: string;
}

function fetchUser(id: number): Observable<User> {
  return from(
    fetch(`https://jsonplaceholder.typicode.com/users/${id}`)
      .then(response => response.json())
  ).pipe(
    catchError(error => {
      console.error('API Error:', error);
      return of({ id: 0, name: 'Unknown', email: '' });
    })
  );
}

fetchUser(1).subscribe(user => console.log('User:', user));
```

### 2. Sequentielle Verarbeitung von Array-Elementen

Führen Sie die asynchrone Verarbeitung auf jedem Element des Arrays sequentiell aus.

```typescript
import { from } from 'rxjs';
import { concatMap, delay } from 'rxjs';

const urls = [
  'https://jsonplaceholder.typicode.com/posts/1',
  'https://jsonplaceholder.typicode.com/posts/2',
  'https://jsonplaceholder.typicode.com/posts/3'
];

from(urls).pipe(
  concatMap(url =>
    from(fetch(url).then(res => res.json())).pipe(
      delay(500) // Rate-Limiting
    )
  )
).subscribe(data => console.log('Abgerufen:', data));
```

### 3. Asynchrone Iteratoren verarbeiten

Asynchrone Iteratoren (async generator) werden ebenfalls unterstützt.

```typescript
import { from } from 'rxjs';

async function* asyncGenerator() {
  yield await Promise.resolve(1);
  yield await Promise.resolve(2);
  yield await Promise.resolve(3);
}

from(asyncGenerator()).subscribe(value => console.log(value));
// Ausgabe: 1, 2, 3
```

### 4. Integration von Ereignis-Emittern

Streams von Node.js EventEmitter und benutzerdefinierten Ereignissystemen.

```typescript
import { from } from 'rxjs';

// Benutzerdefiniertes iterierbares Objekt
class DataSource {
  *[Symbol.iterator]() {
    yield 'Daten A';
    yield 'Daten B';
    yield 'Daten C';
  }
}

from(new DataSource()).subscribe(console.log);
// Ausgabe: Daten A, Daten B, Daten C
```

## Verwendung in der Pipeline

`from()` ist nützlich, um vorhandene Daten als Ausgangspunkt für die Verarbeitung in der Pipeline zu verwenden.

```typescript
import { from } from 'rxjs';
import { map, filter, reduce } from 'rxjs';

interface Product {
  id: number;
  name: string;
  price: number;
}

const products: Product[] = [
  { id: 1, name: 'Produkt A', price: 1000 },
  { id: 2, name: 'Produkt B', price: 2000 },
  { id: 3, name: 'Produkt C', price: 500 }
];

from(products).pipe(
  filter(product => product.price >= 1000),
  map(product => product.price),
  reduce((sum, price) => sum + price, 0)
).subscribe(total => console.log('Gesamtbetrag:', total));
// Ausgabe: Gesamtbetrag: 3000
```

## Häufige Fehler

### 1. Missverstehen, wann ein Promise ausgeführt wird

```typescript
// ❌ Irrtum - Promise wird zum Zeitpunkt der Erstellung ausgeführt
const promise = fetch('https://jsonplaceholder.typicode.com/posts/1'); // Bereits gestartet
from(promise).subscribe(console.log); // Nicht zum Zeitpunkt des Abonnements

// ✅ Richtig - verwenden Sie defer(), wenn Sie zum Zeitpunkt der Subskription ausführen wollen
import { defer, from } from 'rxjs';

const deferred$ = defer(() =>
  from(fetch('https://jsonplaceholder.typicode.com/posts/1'))
);
deferred$.subscribe(console.log); // Ausführen zum Zeitpunkt der Subskription
```

> [!WARNING]
> **Promise wird nicht verzögert ausgewertet**
>
> Promise beginnt mit der Ausführung, wenn es erstellt wird. `from(promise)` umhüllt nur ein Promise, das bereits läuft. Wenn Sie es im Abonnement ausführen wollen, verwenden Sie `defer(() => from(promise))`.

### 2. Array und of() verwechseln

```typescript
import { from, map, of } from "rxjs";

// ❌ Anders als beabsichtigt - das gesamte Array wird ausgegeben
of([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Ausgabe: [1, 2, 3] (Array selbst)

// ✅ Richtig - jedes Element wird separat verarbeitet
from([1, 2, 3]).pipe(
  map(n => n * 2)
).subscribe(console.log);
// Ausgabe: 2, 4, 6
```

## Leistungsüberlegungen

Die Leistung von `from()` hängt von der Art der Eingabe ab.

> [!TIP]
> **Optimierungstipps**:
> - Wenn Sie große Datenmengen verarbeiten (Tausende bis Zehntausende von Elementen), begrenzen Sie die Anzahl der Konkurrenzen bei der Kombination mit `concatMap` oder `mergeMap`.
> - Bei der Verarbeitung von Promise-Arrays sollten Sie `forkJoin` oder `combineLatest` verwenden.

```typescript
import { from } from 'rxjs';
import { mergeMap } from 'rxjs';

const urls = [...Array(100)].map((_, i) => `https://jsonplaceholder.typicode.com/posts/${i + 1}`);

from(urls).pipe(
  mergeMap(
    url => from(fetch(url).then(res => res.json())),
    5 // Parallele Ausführungen auf 5 begrenzen
  )
).subscribe(data => console.log(data));
```

## Verwandte Erstellungsfunktionen

| Funktion | Unterschiede | Verwendung |
|----------|------|----------|
| **[of()](/de/guide/creation-functions/basic/of)** | Argumente in Reihenfolge veröffentlichen | Wert so veröffentlichen, wie er ist |
| **[fromEvent()](/de/guide/creation-functions/basic/fromEvent)** | Streaming Events | Behandlung von DOM Events und EventEmitters |
| **[defer()](/de/guide/creation-functions/conditional/defer)** | Aufschieben der Erzeugung bei Abonnement | Promise muss verzögert ausgeführt werden |
| **ajax()** | Nur für HTTP-Kommunikation | HTTP-Anfragen innerhalb von RxJS abschließen |

## Zusammenfassung

- `from()` erzeugt Observable aus Array, Promise und Iterable
- Jedes Element eines Arrays wird separat ausgegeben (anders als bei `of()`)
- Verarbeitet automatisch Promise und gibt das Ergebnis aus
- Ideal für die Integration von asynchroner Verarbeitung in die RxJS-Welt
- Beachten Sie, dass Promise zum Zeitpunkt der Erstellung ausgeführt wird (verwenden Sie `defer()` für eine verzögerte Ausführung)

## Nächste Schritte

- [fromEvent() - Ereignis in Observable umwandeln](/de/guide/creation-functions/basic/fromEvent)
- [defer() - Die Erstellung bei der Anmeldung aufschieben](/de/guide/creation-functions/conditional/defer)
- [Zurück zu Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)
