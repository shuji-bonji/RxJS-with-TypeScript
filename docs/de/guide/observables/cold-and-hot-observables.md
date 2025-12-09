---
description: "Detaillierte Erklärung der Unterschiede zwischen Cold und Hot Observables. Unabhängigkeit von Datenströmen bei jeder Subscription, Hot-Konvertierung durch share und shareReplay, Multicasting mit BehaviorSubject, und typsichere Implementierungsmuster in TypeScript."
---
# Cold Observable und Hot Observable

Eines der wichtigen Konzepte bei der Verwendung von RxJS ist die Unterscheidung zwischen "Cold Observable" und "Hot Observable". Das Verständnis dieses Unterschieds ist unerlässlich, um den effizienten Einsatz von Observables zu beherrschen.

## Warum ist das Verständnis von Cold/Hot wichtig?

Wenn Sie den Unterschied zwischen Cold und Hot nicht verstehen, werden Sie mit folgenden Problemen konfrontiert:

- **Unbeabsichtigte doppelte Ausführung** - API-Aufrufe werden mehrfach ausgeführt
- **Speicherlecks** - Subscriptions können nicht ordnungsgemäß verwaltet werden
- **Leistungsprobleme** - Unnötige Verarbeitung wird wiederholt
- **Dateninkonsistenz** - Erwartete Daten werden nicht empfangen

## Unterschied zwischen Cold und Hot (Vergleichstabelle)

Lassen Sie uns zunächst einen Überblick gewinnen.

| Vergleichspunkt | Cold Observable | Hot Observable |
|----------|------------------|----------------|
| **Ausführung ohne Subscription** | Wird nicht ausgeführt (wird erst bei Subscription ausgeführt) | Wird ausgeführt (sendet Werte auch ohne subscribe) |
| **Zeitpunkt der Datenausgabe** | Beginnt bei `subscribe()` | Beginnt zum Zeitpunkt des Senders (unabhängig von Subscription) |
| **Wiederverwendung der Ausführung** | Wird jedes Mal neu ausgeführt | Bestehender Stream wird von mehreren geteilt |
| **Datenkonsistenz** | Jede Subscription erhält unabhängige Werte | Wer mitten einsteigt, kann vergangene Werte nicht empfangen |
| **Hauptverwendungsbeispiele** | HTTP-Requests, asynchrone Verarbeitung | UI-Events, WebSocket, Echtzeitkommunikation |
| **Verwendungssituation** | Wenn jede Verarbeitung unabhängig ist | Zustandsfreigabe, Event-Broadcasting |

**Entscheidungskriterium:** Soll die Verarbeitung für jeden Subscriber neu ausgeführt werden? Oder soll der Stream geteilt werden?

## Kriterien zur Unterscheidung von Cold und Hot

Um tatsächlich zu erkennen, ob ein Observable Cold oder Hot ist, können Sie anhand folgender Kriterien urteilen:

| Beurteilungspunkt | Cold | Hot |
|-------------|------|-----|
| **Wird die Ausführungslogik bei jeder Subscription neu ausgeführt?** | ✅ Jedes Mal neu ausgeführt | ❌ Ausführung wird geteilt |
| **Fließen Daten vor der Subscription?** | ❌ Wartet bis Subscription erfolgt | ✅ Fließt unabhängig von Subscription |
| **Erhalten mehrere Subscriptions dieselben Daten?** | ❌ Unabhängige Daten | ✅ Teilen dieselben Daten |

### Praktische Unterscheidungsmethode

Mit folgendem Test können Sie einfach beurteilen:

```typescript
const observable$ = /* Zu untersuchendes Observable */;

observable$.subscribe(/* Subscription 1 */);
observable$.subscribe(/* Subscription 2 */);

// ✅ Cold: console.log im Observable wird 2x ausgeführt
//         (Ausführungslogik wird bei jeder Subscription neu ausgeführt)
// ✅ Hot:  console.log im Observable wird nur 1x ausgeführt
//         (Ausführung wird geteilt)
```

**Konkretes Beispiel:**

```typescript
import { Observable, Subject } from 'rxjs';

// Cold Observable
const cold$ = new Observable(subscriber => {
  console.log('Cold: Ausführung gestartet');
  subscriber.next(Math.random());
});

cold$.subscribe(v => console.log('Subscription 1:', v));
cold$.subscribe(v => console.log('Subscription 2:', v));
// Ausgabe:
// Cold: Ausführung gestartet  ← 1. Mal
// Subscription 1: 0.123...
// Cold: Ausführung gestartet  ← 2. Mal (wird neu ausgeführt)
// Subscription 2: 0.456...

// Hot Observable
const hot$ = new Subject();

hot$.subscribe(v => console.log('Subscription 1:', v));
hot$.subscribe(v => console.log('Subscription 2:', v));
hot$.next(1); // Datenausgabe erfolgt nur 1x
// Ausgabe:
// Subscription 1: 1
// Subscription 2: 1  ← Teilen dieselben Daten
```

## Klassifizierungstabelle Creation Functions nach Cold/Hot

Alle wichtigen Creation Functions werden nach Cold/Hot klassifiziert. So sehen Sie auf einen Blick, welche Funktion welches Observable erzeugt.

| Kategorie | Creation Function | Cold/Hot | Bemerkungen |
|---------|-------------------|----------|------|
| **Grundlegende Erstellung** | `of()` | ❄️ Cold | Gibt Werte bei jeder Subscription neu aus |
| | `from()` | ❄️ Cold | Führt Array/Promise bei jeder Subscription neu aus |
| | `fromEvent()` | ❄️ Cold | Fügt bei jeder Subscription unabhängigen Listener hinzu [^fromEvent] |
| | `interval()` | ❄️ Cold | Unabhängiger Timer bei jeder Subscription |
| | `timer()` | ❄️ Cold | Unabhängiger Timer bei jeder Subscription |
| **Schleifengenerierung** | `range()` | ❄️ Cold | Generiert Bereich bei jeder Subscription neu |
| | `generate()` | ❄️ Cold | Führt Schleife bei jeder Subscription neu aus |
| **HTTP-Kommunikation** | `ajax()` | ❄️ Cold | Neue HTTP-Anfrage bei jeder Subscription |
| | `fromFetch()` | ❄️ Cold | Neue Fetch-Anfrage bei jeder Subscription |
| **Kombination** | `concat()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| | `merge()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| | `combineLatest()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| | `zip()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| | `forkJoin()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| **Auswahl/Partitionierung** | `race()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| | `partition()` | ❄️ Cold | Erbt Eigenschaften der ursprünglichen Observables [^combination] |
| **Bedingungsverzweigung** | `iif()` | ❄️ Cold | Erbt Eigenschaften des je nach Bedingung gewählten Observables |
| | `defer()` | ❄️ Cold | Führt Factory-Funktion bei jeder Subscription aus |
| **Steuerung** | `scheduled()` | ❄️ Cold | Erbt Eigenschaften des ursprünglichen Observables |
| | `using()` | ❄️ Cold | Erstellt Ressource bei jeder Subscription |
| **Subject-Typ** | `new Subject()` | 🔥 Hot | Immer Hot |
| | `new BehaviorSubject()` | 🔥 Hot | Immer Hot |
| | `new ReplaySubject()` | 🔥 Hot | Immer Hot |
| | `new AsyncSubject()` | 🔥 Hot | Immer Hot |
| **WebSocket** | `webSocket()` | 🔥 Hot | Teilt WebSocket-Verbindung |

[^fromEvent]: `fromEvent()` ist Cold, da es bei jeder Subscription einen unabhängigen Event-Listener hinzufügt. Allerdings wird es leicht mit Hot verwechselt, da das Event selbst unabhängig von der Subscription auftritt.

[^combination]: Kombinierte Creation Functions sind Cold, wenn die Quell-Observables Cold sind, und Hot, wenn sie Hot sind. Normalerweise werden häufig Cold-Observables kombiniert.

> [!IMPORTANT] Wichtiges Prinzip
> **Fast alle Creation Functions erzeugen Cold Observables.**
> Nur folgende erzeugen Hot:
> - Subject-Typen (Subject, BehaviorSubject, ReplaySubject, AsyncSubject)
> - webSocket()

## Cold Observable

### Eigenschaften

- **Bei jeder Subscription wird ein neuer Datenstrom erstellt**
- **Beginnt erst mit Datenausgabe, wenn subscribed wird (verzögerte Ausführung)**
- **Alle Subscriber erhalten alle Daten vom Anfang des Observables**

Bei Cold Observables wird bei jedem subscribe ein neuer Ausführungskontext erstellt.
Dies ist geeignet für HTTP-Requests und asynchrone Verarbeitung, bei denen jedes Mal eine neue Verarbeitung erforderlich ist.

### Codebeispiel

```typescript
import { Observable } from 'rxjs';

// Beispiel für Cold Observable
const cold$ = new Observable<number>(subscriber => {
  console.log('Erstellung der Datenquelle - neue Subscription');
  const randomValue = Math.random();
  subscriber.next(randomValue);
  subscriber.complete();
});

// 1. Subscription
console.log('--- 1. Subscription ---');
cold$.subscribe(value => console.log('Subscriber 1:', value));

// 2. Subscription (unterschiedliche Daten werden generiert)
console.log('--- 2. Subscription ---');
cold$.subscribe(value => console.log('Subscriber 2:', value));
```

#### Ausführungsergebnis
```sh
--- 1. Subscription ---
Erstellung der Datenquelle - neue Subscription
Subscriber 1: 0.259632...
--- 2. Subscription ---
Erstellung der Datenquelle - neue Subscription  ← Wird neu ausgeführt
Subscriber 2: 0.744322...  ← Unterschiedlicher Wert
```

> [!TIP] Wichtiger Punkt
> Bei jeder Subscription wird "Erstellung der Datenquelle" ausgeführt und unterschiedliche Werte werden generiert.

### Häufige Cold Observables (Erkennungsmethode)

Folgende Observables sind normalerweise Cold:

```typescript
import { of, from, interval, timer } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// Creation Functions
of(1, 2, 3)                    // Cold
from([1, 2, 3])                // Cold
from(fetch('/api/data'))       // Cold

// Zeit-Operatoren
interval(1000)                 // Cold
timer(1000)                    // Cold

// HTTP Requests
ajax('/api/users')             // Cold
```

> [!TIP] Regel
> Creation Functions, Zeit-Operatoren, HTTP-Requests sind grundsätzlich Cold

## Hot Observable

### Eigenschaften

- **Sendet Werte auch ohne subscribe (wird unabhängig von Subscription ausgeführt)**
- **Empfängt nur Daten ab dem Zeitpunkt der Subscription**
- **Eine Datenquelle wird von mehreren Subscribern geteilt**

Bei Hot Observables ist der Zeitpunkt der Stream-Ausgabe unabhängig von der Subscription, und Subscriber treten mitten ein.

### Codebeispiel

```typescript
import { Subject } from 'rxjs';

// Beispiel für Hot Observable (mit Subject)
const hot$ = new Subject<number>();

// Erste Subscription
console.log('--- Subscriber 1 Start ---');
hot$.subscribe(value => console.log('Subscriber 1:', value));

// Datenausgabe
hot$.next(1);
hot$.next(2);

// Zweite Subscription (spätere Subscription)
console.log('--- Subscriber 2 Start ---');
hot$.subscribe(value => console.log('Subscriber 2:', value));

// Weitere Datenausgabe
hot$.next(3);
hot$.next(4);

hot$.complete();
```

#### Ausführungsergebnis
```sh
--- Subscriber 1 Start ---
Subscriber 1: 1
Subscriber 1: 2
--- Subscriber 2 Start ---
Subscriber 1: 3
Subscriber 2: 3  ← Subscriber 2 tritt ab 3 ein (kann 1, 2 nicht empfangen)
Subscriber 1: 4
Subscriber 2: 4
```

> [!TIP] Wichtiger Punkt
> Subscriber 2 ist mitten eingetreten, daher kann er vergangene Werte (1, 2) nicht empfangen.

### Häufige Hot Observables (Erkennungsmethode)

Folgende Observables sind immer Hot:

```typescript
import { Subject, BehaviorSubject, ReplaySubject } from 'rxjs';
import { webSocket } from 'rxjs/webSocket';

// Subject-Typen (immer Hot)
new Subject()                  // Hot
new BehaviorSubject(0)         // Hot
new ReplaySubject(1)           // Hot

// WebSocket (immer Hot)
webSocket('ws://localhost:8080') // Hot
```

> [!TIP] Regel
> **Nur Subject-Typen und webSocket() erzeugen Hot**

> [!WARNING] fromEvent() ist Cold
> `fromEvent(button, 'click')` wird leicht mit Hot verwechselt, ist aber tatsächlich **Cold**. Es fügt bei jeder Subscription einen unabhängigen Event-Listener hinzu. Das Event selbst tritt unabhängig von der Subscription auf, aber jeder Subscriber hat einen unabhängigen Listener.

## Methoden zur Konvertierung von Cold Observable zu Hot

In RxJS werden hauptsächlich folgende Methoden verwendet, um Cold Observables in Hot zu konvertieren:

- `share()` - Einfache Hot-Konvertierung (empfohlen)
- `shareReplay()` - Hot-Konvertierung mit Cache vergangener Werte
- ~~`multicast()`~~ - Veraltet (deprecated in RxJS v7, entfernt in v8)

### share() Operator

`share()` ist die gängigste Methode, um Cold Observables in Hot Observables zu konvertieren.

```typescript
import { interval } from 'rxjs';
import { share, take } from 'rxjs';

// HTTP-Aufruf simulieren
const makeHttpRequest = () => {
  console.log('HTTP-Aufruf ausgeführt!');
  return interval(1000).pipe(take(3));
};

// ❌ Cold Observable (keine Freigabe)
const cold$ = makeHttpRequest();

cold$.subscribe(val => console.log('Subscriber 1:', val));
cold$.subscribe(val => console.log('Subscriber 2:', val));
// → HTTP-Aufruf wird 2x ausgeführt

// ✅ Hot Observable (mit share)
const shared$ = makeHttpRequest().pipe(share());

shared$.subscribe(val => console.log('Geteilter Subscriber 1:', val));
shared$.subscribe(val => console.log('Geteilter Subscriber 2:', val));
// → HTTP-Aufruf nur 1x, Ergebnis wird geteilt
```

**Ausführungsergebnis (Cold):**
```sh
HTTP-Aufruf ausgeführt!  ← 1. Mal
Subscriber 1: 0
HTTP-Aufruf ausgeführt!  ← 2. Mal (Duplikat!)
Subscriber 2: 0
...
```

**Ausführungsergebnis (Hot):**
```sh
HTTP-Aufruf ausgeführt!  ← Nur 1x
Geteilter Subscriber 1: 0
Geteilter Subscriber 2: 0  ← Teilt denselben Stream
...
```

> [!NOTE] Anwendungsfälle
> - Verwendung desselben API-Ergebnisses in mehreren Komponenten
> - Vermeidung von Duplikaten bei Seiteneffekten (HTTP-Aufrufe etc.)

### shareReplay() Operator

`shareReplay()` ist eine erweiterte Version von `share()` und **cached vergangene Werte**, um sie neuen Subscribern wiederzugeben.

```typescript
import { interval } from 'rxjs';
import { shareReplay, take } from 'rxjs';

const request$ = interval(1000).pipe(
  take(3),
  shareReplay(2)  // Cached die letzten 2 Werte
);

// 1. Subscription
request$.subscribe(val => console.log('Subscriber 1:', val));

// 3,5 Sekunden später 2. Subscription (nach Stream-Abschluss)
setTimeout(() => {
  console.log('--- Subscriber 2 Start (nach Abschluss) ---');
  request$.subscribe(val => console.log('Subscriber 2:', val));
}, 3500);
```

#### Ausführungsergebnis
```sh
Subscriber 1: 0
Subscriber 1: 1
Subscriber 1: 2
--- Subscriber 2 Start (nach Abschluss) ---
Subscriber 2: 1  ← Gecachte Werte (letzte 2)
Subscriber 2: 2  ← Gecachte Werte
```

> [!NOTE] Anwendungsfälle
> - Caching von API-Ergebnissen
> - Freigabe des Anfangszustands (nur neuester 1 Wert gecacht)
> - Bereitstellung vergangener Daten für verspätete Subscriber

> [!WARNING] Hinweis zu shareReplay
> `shareReplay()` behält den Cache auch bei 0 Subscriptions bei, was zu Speicherlecks führen kann. Siehe [Kapitel 10: Fehlerhafte Verwendung von shareReplay](/de/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用) für Details.

### Über multicast()

> [!NOTE]
> `multicast()` ist flexibel, wurde aber in RxJS v7 als veraltet markiert und in v8 entfernt. Verwenden Sie jetzt `share()` oder `shareReplay()`. Siehe [share() Operator-Erklärung](/de/guide/operators/multicasting/share) für Details.

## Praktisches Beispiel: API-Cache-Service

Ein häufiges Muster in realen Anwendungen: Wenn mehrere Komponenten dieselben API-Daten benötigen.

```typescript
import { Observable, of, throwError } from 'rxjs';
import { catchError, shareReplay, delay, tap } from 'rxjs';

// Einfacher Cache-Service
class UserService {
  private cache$: Observable<User[]> | null = null;

  getUsers(): Observable<User[]> {
    // Wenn Cache vorhanden, diesen zurückgeben
    if (this.cache$) {
      console.log('Rückgabe aus Cache');
      return this.cache$;
    }

    // Neue Anfrage erstellen und cachen
    console.log('Neue Anfrage ausgeführt');
    this.cache$ = this.fetchUsersFromAPI().pipe(
      catchError(err => {
        this.cache$ = null;  // Cache bei Fehler löschen
        return throwError(() => err);
      }),
      shareReplay(1)  // Letztes Ergebnis cachen
    );

    return this.cache$;
  }

  private fetchUsersFromAPI(): Observable<User[]> {
    // Echte API-Anfrage simulieren
    return of([
      { id: 1, name: 'Max Mustermann' },
      { id: 2, name: 'Erika Musterfrau' }
    ]).pipe(
      delay(1000),
      tap(() => console.log('Daten von API empfangen'))
    );
  }

  clearCache(): void {
    this.cache$ = null;
    console.log('Cache gelöscht');
  }
}

interface User {
  id: number;
  name: string;
}

// Verwendungsbeispiel
const userService = new UserService();

// Komponente 1: Daten anfordern
userService.getUsers().subscribe(users =>
  console.log('Komponente 1:', users)
);

// Komponente 2: Daten nach 2 Sekunden anfordern
setTimeout(() => {
  userService.getUsers().subscribe(users =>
    console.log('Komponente 2:', users)
  );
}, 2000);

// Cache löschen und erneut anfordern
setTimeout(() => {
  userService.clearCache();
  userService.getUsers().subscribe(users =>
    console.log('Komponente 3:', users)
  );
}, 4000);
```

#### Ausführungsergebnis
```sh
Neue Anfrage ausgeführt
Daten von API empfangen
Komponente 1: [{id: 1, name: 'Max Mustermann'}, {id: 2, name: 'Erika Musterfrau'}]
Rückgabe aus Cache  ← Kein API-Aufruf
Komponente 2: [{id: 1, name: 'Max Mustermann'}, {id: 2, name: 'Erika Musterfrau'}]
Cache gelöscht
Neue Anfrage ausgeführt  ← Erneuter API-Aufruf
Daten von API empfangen
Komponente 3: [{id: 1, name: 'Max Mustermann'}, {id: 2, name: 'Erika Musterfrau'}]
```

**Punkte:**
- `shareReplay(1)` cached die letzte Antwort
- Mehrere Komponenten teilen Daten (API-Aufruf nur 1x)
- Cache wird bei Fehler oder beim Löschen ordnungsgemäß verworfen

## Wann verwenden?

<div class="comparison-cards">

::: tip Cold
#### Wann zu verwenden
- Wenn jeder Subscriber ein eigenes Dataset benötigt
- Wenn ein neu startender Prozess oder eine Aktion dargestellt wird
- Wenn Duplikate bei Seiteneffekten kein Problem sind

#### Beispiele
- Neuen POST-Request bei jeder Formularübermittlung senden
- Unterschiedlicher Timer für jeden Benutzer erforderlich
- Unabhängige Berechnung bei jeder Subscription ausführen
:::

::: tip Hot
#### Wann zu verwenden
- Wenn Daten zwischen mehreren Komponenten geteilt werden
- Wenn Ressourcen gespart werden sollen (z.B. Anzahl der HTTP-Aufrufe reduzieren)
- Wenn ein Event-Stream dargestellt wird
- State-Management oder Kommunikation zwischen Services

#### Beispiele
- In der gesamten Anwendung geteilte Konfigurationsinformationen
- Login-Status des Benutzers
- Echtzeit-Nachrichten (WebSocket)
- DOM-Events (Click, Scroll etc.)
:::

</div>

## Zusammenfassung

Das Verständnis und die richtige Verwendung von Cold und Hot Observables ist eine wichtige Fähigkeit für den Aufbau effizienter RxJS-Anwendungen.

::: tip Wichtige Punkte
- **Cold Observable**: Stream, der erst bei Subscription startet (unabhängige Ausführung bei jeder Subscription)
- **Hot Observable**: Teilt bereits laufenden Stream (dieselbe Ausführung bei mehreren Subscriptions)
- **share()**: Einfachste Methode, Cold in Hot zu konvertieren
- **shareReplay()**: Konvertiert in Hot mit Cache vergangener Werte (praktisch für Freigabe von API-Ergebnissen)
:::

::: tip Kriterien für Designentscheidungen
- Müssen Daten zwischen mehreren Subscribern geteilt werden?
- Müssen vergangene Werte gecacht und neuen Subscribern bereitgestellt werden?
- Wie werden Duplikate bei Seiteneffekten (HTTP-Requests etc.) verwaltet?
:::

Basierend auf diesen Überlegungen können Sie durch Auswahl des geeigneten Observable-Typs und Operators effiziente und robuste reaktive Anwendungen erstellen.

## Verwandte Abschnitte

- **[share() Operator](/de/guide/operators/multicasting/share)** - Detaillierte Erklärung von share()
- **[Fehlerhafte Verwendung von shareReplay](/de/guide/anti-patterns/common-mistakes#4-sharereplay-の誤用)** - Häufige Fehler und Gegenmaßnahmen
- **[Subject](/de/guide/subjects/what-is-subject)** - Verständnis von Hot Subject

<style scoped>
.comparison-cards {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 1rem;
  margin-bottom: 2rem;
}

@media (max-width: 768px) {
  .comparison-cards {
    grid-template-columns: 1fr;
  }
}
</style>
