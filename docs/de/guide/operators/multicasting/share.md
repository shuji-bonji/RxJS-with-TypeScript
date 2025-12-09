---
description: "Implementierung von Multicasting mit dem share()-Operator. Mehrere Subscriber teilen dasselbe Observable und reduzieren redundante Verarbeitung (API-Aufrufe, Berechnungen). Unterschiede zu shareReplay(), Cold/Hot-Konvertierung und typensichere Implementierung mit TypeScript."
---

# share - Observable mit mehreren Subscribern teilen

Der `share()`-Operator ist der einfachste Weg, Multicasting in RxJS zu implementieren.
Mehrere Subscriber können dieselbe Datenquelle teilen, wodurch redundante Verarbeitung (API-Requests, Berechnungen etc.) reduziert wird.

[📘 RxJS Offizielle Dokumentation - `share()`](https://rxjs.dev/api/index/function/share)

## 🔰 Grundlegende Verwendung

```typescript
import { interval, share, take, tap } from 'rxjs';

// Observable mit Intervall-Zählung
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Quelle: ${value}`)),
  share() // Multicasting aktivieren
);

// Erster Subscriber
console.log('Observer 1 Subscription gestartet');
const subscription1 = source$.subscribe(value =>
  console.log(`Observer 1: ${value}`)
);

// 2.5 Sekunden später zweiten Subscriber hinzufügen
setTimeout(() => {
  console.log('Observer 2 Subscription gestartet');
  source$.subscribe(value =>
    console.log(`Observer 2: ${value}`)
  );

  // 2.5 Sekunden später Subscriber 1 abmelden
  setTimeout(() => {
    console.log('Observer 1 Subscription beendet');
    subscription1.unsubscribe();
  }, 2500);
}, 2500);
```

### Ausführungsergebnis

```
Observer 1 Subscription gestartet
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2 Subscription gestartet
Quelle: 2
Observer 1: 2
Observer 2: 2
Quelle: 3
Observer 1: 3
Observer 2: 3
Observer 1 Subscription beendet
Quelle: 4
Observer 2: 4
```

**Wichtige Punkte**:
- Die Quellverarbeitung (`tap`) wird nur einmal ausgeführt
- Alle Subscriber erhalten dieselben Werte
- Später beigetretene Subscriber erhalten nur Werte nach ihrem Beitritt

## 💡 Funktionsweise von share()

`share()` ist der Standard-Multicasting-Operator in RxJS. Intern wird ein Subject verwendet, um an mehrere Subscriber zu broadcasten.

> [!NOTE]
> **Änderungen ab RxJS v7**: Früher wurde es als Kombination von `multicast()` und `refCount()` erklärt, aber diese Operatoren wurden in v7 als deprecated markiert und in v8 entfernt. Derzeit ist `share()` die standardmäßige Multicasting-Methode. Details siehe [RxJS Offizielle Dokumentation - Multicasting](https://rxjs.dev/deprecations/multicasting).

**Ablauf der Funktionsweise**:
- **Erste Subscription**: Verbindung zum Quell-Observable wird gestartet und internes Subject erstellt
- **Subscriber werden hinzugefügt**: Bestehende Verbindung wird geteilt (Broadcasting über Subject)
- **Alle Subscriber werden entfernt**: Verbindung zur Quelle wird getrennt (bei `resetOnRefCountZero: true`)
- **Erneute Subscription**: Start als neue Verbindung (abhängig von Reset-Einstellung)

## 🎯 Detaillierte Kontrolloptionen (RxJS 7+)

Ab RxJS 7 kann das Verhalten durch Übergabe von Optionen an `share()` fein gesteuert werden.

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(6),
  tap((value) => console.log(`Quelle: ${value}`)),
  share({
    resetOnError: true,       // Bei Fehler zurücksetzen
    resetOnComplete: true,     // Bei Abschluss zurücksetzen
    resetOnRefCountZero: true, // Bei null Subscribern zurücksetzen
  })
);
```

### Optionsdetails

| Option | Standard | Beschreibung |
|-----------|----------|------|
| `resetOnError` | `true` | Internen Zustand bei Fehler zurücksetzen |
| `resetOnComplete` | `true` | Internen Zustand bei Stream-Abschluss zurücksetzen |
| `resetOnRefCountZero` | `true` | Verbindung trennen bei null Subscribern |
| `connector` | `() => new Subject()` | Benutzerdefiniertes Subject angeben |

### Erweiterte Kontrolle mit connector-Option

Mit der `connector`-Option kann ein Verhalten äquivalent zu `shareReplay` erreicht werden.

```typescript
import { interval, ReplaySubject } from 'rxjs';
import { take, share, tap } from 'rxjs';

// ReplaySubject verwenden, um den letzten Wert zu puffern
const source$ = interval(1000).pipe(
  take(5),
  tap(value => console.log(`Quelle: ${value}`)),
  share({
    connector: () => new ReplaySubject(1),
    resetOnError: false,
    resetOnComplete: false,
    resetOnRefCountZero: false
  })
);

// Erster Subscriber
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// 2.5 Sekunden später abonnieren (erhält den letzten Wert)
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 2500);
```

**Ausführungsergebnis**:
```
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2: 1  // ← Erhält den vorherigen Wert trotz spätem Beitritt
Quelle: 2
Observer 1: 2
Observer 2: 2
...
```

> [!TIP]
> Diese Methode kann als Alternative zu `shareReplay(1)` verwendet werden. Durch Setzen von `resetOnRefCountZero: false` wird die Verbindung aufrechterhalten, auch wenn die Referenzzählung null wird, wodurch das Problem des "permanenten Caches" von `shareReplay` vermieden werden kann.

## 📊 Vergleich mit und ohne share()

### ❌ Ohne share() (Cold Observable)

```typescript
import { interval, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Quelle: ${value}`))
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Ausführungsergebnis**:
```
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Quelle: 0    // ← Neuer Stream wird gestartet
Observer 2: 0
Quelle: 2
Observer 1: 2
Quelle: 1
Observer 2: 1
Quelle: 2
Observer 2: 2
```

Jeder Subscriber hat einen unabhängigen Stream, die Quellverarbeitung wird redundant ausgeführt.

### ✅ Mit share() (Hot Observable)

```typescript
import { interval, share, take, tap } from 'rxjs';

const source$ = interval(1000).pipe(
  take(3),
  tap(value => console.log(`Quelle: ${value}`)),
  share()
);

// Subscriber 1
source$.subscribe(value => console.log(`Observer 1: ${value}`));

// Subscriber 2
setTimeout(() => {
  source$.subscribe(value => console.log(`Observer 2: ${value}`));
}, 1500);
```

**Ausführungsergebnis**:
```
Quelle: 0
Observer 1: 0
Quelle: 1
Observer 1: 1
Observer 2: 1  // ← Teilt denselben Stream
Quelle: 2
Observer 1: 2
Observer 2: 2
```

## 💼 Praktische Anwendungsfälle

### Vermeidung redundanter API-Requests

```typescript
import { ajax } from 'rxjs/ajax';
import { share, tap } from 'rxjs';

// Observable zum Abrufen von Benutzerinformationen
const getUser$ = ajax.getJSON('https://jsonplaceholder.typicode.com/users/1').pipe(
  tap(() => console.log('API-Request ausgeführt')),
  share() // Redundante Requests in mehreren Komponenten verhindern
);

// Komponente 1
getUser$.subscribe(user => console.log('Komponente 1:', user));

// Komponente 2 (fast gleichzeitiger Request)
getUser$.subscribe(user => console.log('Komponente 2:', user));

// Ergebnis: API-Request wird nur einmal ausgeführt
```

### Teilen periodischer Datenabrufe

```typescript
import { timer, share, switchMap, tap } from 'rxjs';
import { ajax } from 'rxjs/ajax';

// TODO-Liste alle 5 Sekunden abrufen (API-Request wird geteilt)
const sharedTodos$ = timer(0, 5000).pipe(
  tap(() => console.log('API-Request ausgeführt')),
  switchMap(() => ajax.getJSON('https://jsonplaceholder.typicode.com/todos?_limit=3')),
  share() // API-Request für mehrere Subscriber teilen
);

// Mehrere Komponenten verwenden denselben Datenstrom
sharedTodos$.subscribe(todos => console.log('Komponente A:', todos));
sharedTodos$.subscribe(todos => console.log('Komponente B:', todos));

// Ergebnis: API-Request wird alle 5 Sekunden nur einmal ausgeführt, beide Komponenten erhalten dieselben Daten
```

## ⚠️ Vorsichtsmaßnahmen

1. **Timing beachten**: Später beigetretene Subscriber können vergangene Werte nicht erhalten
2. **Fehlerweiterleitung**: Wenn ein Fehler auftritt, betrifft dies alle Subscriber
3. **Speicherverwaltung**: Nicht ordnungsgemäß beendete Subscriptions können zu Memory Leaks führen

## 🔄 Verwandte Operatoren

- **[shareReplay()](/de/guide/operators/multicasting/shareReplay)** - Puffert vergangene Werte und stellt sie auch späteren Subscribern bereit
- **[Subject](/de/guide/subjects/what-is-subject)** - Grundlegende Klasse für Multicasting

> [!WARNING]
> **Deprecated Operatoren**: Alte Multicasting-APIs wie `publish()`, `multicast()`, `refCount()` wurden in RxJS v7 als deprecated markiert und in v8 entfernt. Verwenden Sie stattdessen `share()` oder `connectable()`/`connect()`.

## Zusammenfassung

Der `share()`-Operator ermöglicht:
- Teilen desselben Observables mit mehreren Subscribern
- Vermeidung redundanter Ausführung von API-Requests oder schweren Berechnungen
- Einfache grundlegende Multicasting-Funktionalität
- Fein abgestimmte Kontrolloptionen ab RxJS 7+

Wenn mehrere Komponenten dieselbe Datenquelle benötigen, kann `share()` die Performance erheblich verbessern.
