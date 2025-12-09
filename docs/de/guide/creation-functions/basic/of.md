---
description: "of() - RxJS-Erstellungsfunktion, die die angegebenen Werte der Reihe nach ausgibt; der einfachste Weg, ein Observable zu erstellen, ideal für Testdaten und Mock-Erstellung; Implementierungsmuster, die TypeScript-Typinferenz, Arrays und Objekte nutzen."
---

# of() - sequentielle Veröffentlichung von Werten

`of()` ist die einfachste Erstellungsfunktion, die die angegebenen Werte nacheinander ausgibt.

## Überblick

Die Funktion `of()` gibt die als Argumente übergebenen Werte der Reihe nach aus, wenn sie abonniert werden, und wird sofort beendet, wenn alle Werte ausgegeben wurden. Sie wird häufig zur Erstellung von Testcode und Mock-Daten verwendet.

**Signatur**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Offizielle Dokumentation**: [📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

## Grundlegende Verwendung

`of()` erlaubt die Übergabe von mehreren Werten, die durch Kommas getrennt sind.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Wert:', value),
  error: err => console.error('Fehler:', err),
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

## Wichtige Merkmale

### 1. Synchron ausgegeben

`of()` gibt alle Werte **synchron** bei der Anmeldung aus.

```typescript
import { of } from 'rxjs';

console.log('Vor dem Abonnieren');

of('A', 'B', 'C').subscribe(value => console.log('Wert:', value));

console.log('Nach dem Abonnieren');

// Ausgabe:
// Vor dem Abonnieren
// Wert: A
// Wert: B
// Wert: C
// Nach dem Abonnieren
```

### 2. Sofortiger Abschluss

Meldet `complete` sofort nach Veröffentlichung aller Werte.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Abgeschlossen!')
});

// Ausgabe: 1, 2, 3, Abgeschlossen!
```

### 3. Jede Art von Wert kann ausgegeben werden

Es können Werte beliebigen Typs ausgegeben werden, von primitiven Typen bis hin zu Objekten und Arrays.

```typescript
import { of } from 'rxjs';

// Primitive Typen
of(42, 'hello', true).subscribe(console.log);

// Objekte
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Arrays (Array selbst wird als ein Wert ausgegeben)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Ausgabe: [1, 2, 3], [4, 5, 6]
```

### 4. Cold Observable

`of()` ist ein **Cold Observable**. Jedes Abonnement initiiert eine unabhängige Ausführung.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Erstes Abonnement
values$.subscribe(val => console.log('Abonnent A:', val));

// Zweites Abonnement (unabhängig ausgeführt)
values$.subscribe(val => console.log('Abonnent B:', val));

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
> - Wenn die gemeinsame Nutzung von Daten erforderlich ist, muss sie Hot gemacht werden, z.B. mit `share()`
>
> Für weitere Informationen siehe [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

## Unterschied zwischen of() und from()

`of()` und `from()` haben ein unterschiedliches Verhalten beim Umgang mit Arrays. Dies ist ein häufiger Punkt der Verwirrung.

```typescript
import { of, from } from 'rxjs';

// of() - Array wird als ein Wert ausgegeben
of([1, 2, 3]).subscribe(console.log);
// Ausgabe: [1, 2, 3]

// from() - Jedes Element des Arrays wird einzeln ausgegeben
from([1, 2, 3]).subscribe(console.log);
// Ausgabe: 1, 2, 3
```

> [!IMPORTANT]
> **Kriterien für unterschiedliche Verwendung**:
> - Array selbst veröffentlichen → `of([1, 2, 3])`
> - Jedes Element des Arrays separat veröffentlichen → `from([1, 2, 3])`

## Praktische Anwendungsfälle

### 1. Erstellung von Testdaten und Mocks

Die Funktion `of()` wird am häufigsten zur Erstellung von Mock-Daten in Testcode verwendet.

```typescript
import { of } from 'rxjs';

// Mock-Benutzerdaten
function getMockUser$() {
  return of({
    id: 1,
    name: 'Test User',
    email: 'test@example.com'
  });
}

// In Tests verwenden
getMockUser$().subscribe(user => {
  console.log('User:', user.name); // User: Test User
});
```

### 2. Bereitstellung von Standardwerten

Wird verwendet, um Fallback-Werte bei Fehlern oder Standardwerte bereitzustellen.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('Invalid ID'));
  }
  return of({ id, data: 'some data' });
}

fetchData(-1).pipe(
  catchError(err => {
    console.error('Fehler:', err.message);
    return of({ id: 0, data: 'default data' }); // Standardwert
  })
).subscribe(result => console.log(result));
// Ausgabe: Fehler: Invalid ID
//         { id: 0, data: 'default data' }
```

### 3. Ausgabe mehrerer Werte in Stufen

Wird verwendet, um mehrere Schritte nacheinander auszuführen.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Loading...', 'Processing...', 'Done!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Ausgabe (jede Sekunde):
// Loading...
// Processing...
// Done!
```

### 4. Rückgabewerte bei bedingter Verzweigung

In Kombination mit `iif()` und `switchMap()`, gibt einen Wert entsprechend einer Bedingung zurück.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Welcome back!'),
  of('Please log in')
).subscribe(console.log);
// Ausgabe: Welcome back!
```

## Verwendung in Pipelines

`of()` wird als Startpunkt für eine Pipeline verwendet oder um Daten auf dem Weg zu injizieren.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Nur gerade Zahlen
  map(n => n * 10)           // 10-fach
).subscribe(console.log);
// Ausgabe: 20, 40
```

## Häufige Fehler

### 1. Übergabe eines Arrays als solches

```typescript
// ❌ Falsch - Das gesamte Array wird als ein einziger Wert ausgegeben
of([1, 2, 3]).subscribe(console.log);
// Ausgabe: [1, 2, 3]

// ✅ Richtig - verwenden Sie from(), wenn Sie jedes Element einzeln ausgeben wollen
from([1, 2, 3]).subscribe(console.log);
// Ausgabe: 1, 2, 3

// ✅ Oder verwenden Sie die Spread-Syntax
of(...[1, 2, 3]).subscribe(console.log);
// Ausgabe: 1, 2, 3
```

### 2. Verwechselt mit asynchroner Verarbeitung

`of()` wird synchron ausgegeben. Beachten Sie, dass es sich nicht um einen asynchronen Prozess handelt.

```typescript
// ❌ Dies ist nicht asynchron
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() wird sofort ausgeführt und sein Promise-Objekt wird ausgegeben

// ✅ Wenn Sie ein Promise streamen wollen, verwenden Sie from()
from(fetchDataFromAPI()).subscribe(console.log);
```

## Überlegungen zur Leistung

`of()` ist sehr leichtgewichtig und hat wenig Performance-Overhead. Wenn Sie jedoch eine große Anzahl von Werten ausgeben, sollten Sie Folgendes beachten.

> [!TIP]
> Wenn Sie eine große Anzahl von Werten (Tausende oder mehr) sequentiell ausgeben, sollten Sie `from()` oder `range()` verwenden.

## Verwandte Erstellungsfunktionen

| Funktion | Unterschiede | Verwendung |
|----------|------|----------|
| **[from()](/de/guide/creation-functions/basic/from)** | Konvertiert von Array oder Promise | Stream iterable oder Promise |
| **range()** | Erzeugt einen Zahlenbereich | Gibt eine Folge von Zahlen aus |
| **EMPTY** | Beendet sofort, ohne etwas auszugeben | Wenn ein leerer Stream erforderlich ist |

## Zusammenfassung

- `of()` ist die einfachste Erstellungsfunktion, die die angegebenen Werte nacheinander ausgibt
- Die Ausgabe erfolgt synchron mit dem Abonnement und wird sofort abgeschlossen
- Ideal für Testdaten und die Erstellung von Mocks
- Wenn ein Array übergeben wird, wird das Array selbst ausgegeben (im Gegensatz zu `from()`)
- Verwenden Sie `from()` für asynchrone Verarbeitung

## Nächste Schritte

- [from() - Konvertierung von Array, Promise, etc.](/de/guide/creation-functions/basic/from)
- [Kombination Erstellungsfunktionen](/de/guide/creation-functions/combination/)
- [Zurück zu Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)
