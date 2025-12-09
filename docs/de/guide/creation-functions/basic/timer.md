---
description: "timer() - Erstellungsfunktion, die nach einer angegebenen Zeit einen Wert ausgibt. Verwenden Sie timer(delay) für einmalige verzögerte Ausführung, timer(delay, period) für verzögerte regelmäßige Ausführung. Unterschiede zu interval(), TypeScript-Typinferenz, Verwendung als setTimeout-Alternative."
---

# timer() - Start nach Verzögerung

`timer()` ist eine Erstellungsfunktion, die nach einer angegebenen Verzögerungszeit mit der Ausgabe von Werten beginnt. Sie unterstützt sowohl einmalige als auch regelmäßige Ausgabe.

## Überblick

`timer()` ist eine flexible Erstellungsfunktion, die den Zeitpunkt der ersten Ausgabe steuern kann. Das Verhalten ändert sich je nach Anzahl der Argumente, und sowohl einmalige Ausgabe als auch regelmäßige Ausgabe wie bei `interval()` sind möglich.

**Signatur**:
```typescript
function timer(
  dueTime: number | Date,
  intervalOrScheduler?: number | SchedulerLike,
  scheduler?: SchedulerLike
): Observable<number>
```

**Offizielle Dokumentation**: [📘 RxJS Official: timer()](https://rxjs.dev/api/index/function/timer)

## Grundlegende Verwendung

Das Verhalten von `timer()` ändert sich je nach Anzahl der Argumente.

### Einmalige Ausgabe

Wenn nur das erste Argument angegeben wird, wird nach der angegebenen Zeit 0 ausgegeben und abgeschlossen.

```typescript
import { timer } from 'rxjs';

// Gibt 0 nach 3 Sekunden aus und schließt ab
const timer$ = timer(3000);

timer$.subscribe({
  next: value => console.log('Wert:', value),
  complete: () => console.log('Abgeschlossen')
});

// Ausgabe nach 3 Sekunden:
// Wert: 0
// Abgeschlossen
```

### Regelmäßige Ausgabe

Wenn das zweite Argument als Intervall angegeben wird, erfolgt nach der anfänglichen Verzögerung regelmäßige Ausgabe.

```typescript
import { timer } from 'rxjs';

// Startet nach 3 Sekunden, dann jede Sekunde einen Wert ausgeben
const timer$ = timer(3000, 1000);

timer$.subscribe(value => console.log('Wert:', value));

// Ausgabe:
// Wert: 0  (nach 3 Sekunden)
// Wert: 1  (nach 4 Sekunden)
// Wert: 2  (nach 5 Sekunden)
// ...(unendlich)
```

## Wichtige Merkmale

### 1. Flexible Verzögerungsangabe

Die Verzögerungszeit kann als Millisekundenwert oder `Date`-Objekt angegeben werden.

```typescript
import { timer } from 'rxjs';

// In Millisekunden angeben
timer(5000).subscribe(() => console.log('Nach 5 Sekunden'));

// Mit Date-Objekt angeben (zu einer bestimmten Zeit ausführen)
const targetTime = new Date(Date.now() + 10000); // 10 Sekunden später
timer(targetTime).subscribe(() => console.log('Zur angegebenen Zeit ausgeführt'));
```

### 2. Verhalten ändert sich mit/ohne zweites Argument

Ob abgeschlossen wird oder nicht, hängt davon ab, ob das zweite Argument angegeben wird.

```typescript
import { timer } from 'rxjs';

// Ohne zweites Argument - Einmalige Ausgabe und Abschluss
timer(1000).subscribe({
  next: value => console.log('Erstes Mal:', value),
  complete: () => console.log('Abgeschlossen')
});

// Mit zweitem Argument - Unendliche Ausgabe
timer(1000, 1000).subscribe({
  next: value => console.log('Wiederholung:', value),
  complete: () => console.log('Abgeschlossen (wird nicht angezeigt)')
});
```

> [!IMPORTANT]
> **Mit zweitem Argument wird nicht abgeschlossen**
>
> Wie bei `timer(1000, 1000)`, wenn das zweite Argument angegeben wird, erfolgt wie bei `interval()` unendliche Ausgabe. Abbestellen ist zwingend erforderlich.

### 3. Cold Observable

`timer()` ist eine Cold Observable, und für jedes Abonnement wird ein unabhängiger Timer erstellt.

```typescript
import { timer } from 'rxjs';

const timer$ = timer(1000);

console.log('Start');

// Abonnement 1
timer$.subscribe(() => console.log('Observer 1'));

// 500ms später Abonnement 2 hinzufügen
setTimeout(() => {
  timer$.subscribe(() => console.log('Observer 2'));
}, 500);

// Ausgabe:
// Start
// Observer 1  (nach 1 Sekunde)
// Observer 2  (nach 1,5 Sekunden - unabhängiger Timer)
```

> [!NOTE]
> **Cold Observable Merkmale**
> - Jedes Abonnement startet eine unabhängige Ausführung
> - Jeder Abonnent erhält seinen eigenen Datenstrom
> - Bei jedem Abonnement wird ein unabhängiger Timer gestartet. Wie bei interval(), verwenden Sie `share()` wenn Teilen erforderlich ist.
>
> Weitere Informationen finden Sie unter [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

## timer() vs interval() Unterschiede

Der Hauptunterschied zwischen beiden ist der Zeitpunkt der ersten Ausgabe.

```typescript
import { timer, interval } from 'rxjs';
import { take } from 'rxjs';

console.log('Start');

// interval() - Startet sofort (erster Wert nach 1 Sekunde)
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Keine Verzögerung (erster Wert sofort)
timer(0, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// timer() - Startet nach 2 Sekunden Verzögerung
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer(verzögert):', value);
});
```

| Erstellungsfunktion | Erste Ausgabe | Verwendung |
|-------------------|------------------|------|
| `interval(1000)` | Nach 1 Sekunde | Sofort regelmäßige Ausführung starten |
| `timer(0, 1000)` | Sofort | Wenn nur das erste Mal sofort ausgeführt werden soll |
| `timer(2000, 1000)` | Nach 2 Sekunden | Verzögerte regelmäßige Ausführung |
| `timer(2000)` | Nach 2 Sekunden (nur einmal) | Verzögerte Ausführung (einmalig) |

## Praktische Anwendungsfälle

### 1. Verzögerte Ausführung

Führt einmal nach einer bestimmten Zeit aus.

```typescript
import { from, timer } from 'rxjs';
import { switchMap } from 'rxjs';

function delayedApiCall() {
  return timer(2000).pipe(
    switchMap(() => from(
      fetch('https://jsonplaceholder.typicode.com/posts/1')
        .then(res => res.json())
    ))
  );
}

delayedApiCall().subscribe(data => {
  console.log('Daten nach 2 Sekunden abgerufen:', data);
});
```

### 2. Verzögertes Polling

Startet Polling nach einer bestimmten Zeit, nicht sofort.

```typescript
import { from, timer } from 'rxjs';
import { switchMap, retry } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

// Polling startet nach 5 Sekunden, dann alle 10 Sekunden
const polling$ = timer(5000, 10000).pipe(
  switchMap(() => from(
    fetch('https://jsonplaceholder.typicode.com/users/1')
      .then(res => res.json() as Promise<Status>)
  )),
  retry(3) // Bis zu 3 Wiederholungen bei Fehler
);

const subscription = polling$.subscribe(data => {
  console.log('Status-Update:', data);
});

// Bei Bedarf stoppen
// subscription.unsubscribe();
```

### 3. Timeout-Verarbeitung

Timeout wenn Verarbeitung nicht innerhalb einer bestimmten Zeit abgeschlossen wird.

```typescript
import { timer, race, from } from 'rxjs';
import { map } from 'rxjs';

function fetchWithTimeout(url: string, timeoutMs: number) {
  const request$ = from(fetch(url).then(res => res.json()));
  const timeout$ = timer(timeoutMs).pipe(
    map(() => {
      throw new Error('Timeout');
    })
  );

  // Der schnellere wird angenommen
  return race(request$, timeout$);
}

fetchWithTimeout('https://jsonplaceholder.typicode.com/posts/1', 3000).subscribe({
  next: data => console.log('Daten abgerufen:', data),
  error: err => console.error('Fehler:', err.message)
});
```

### 4. Automatisches Ausblenden von Benachrichtigungen

Blendet Benachrichtigungen nach einer bestimmten Zeit automatisch aus.

```typescript
import { timer, Subject, map } from 'rxjs';
import { switchMap, takeUntil } from 'rxjs';

interface Notification {
  id: number;
  message: string;
}

const notifications$ = new Subject<Notification>();
const dismiss$ = new Subject<number>();

notifications$.pipe(
  switchMap(notification => {
    console.log('Benachrichtigung anzeigen:', notification.message);

    // Nach 5 Sekunden automatisch ausblenden
    return timer(5000).pipe(
      takeUntil(dismiss$), // Abbrechen wenn manuell ausgeblendet
      map(() => notification.id)
    );
  })
).subscribe(id => {
  console.log('Benachrichtigung ausblenden:', id);
});

// Benachrichtigung anzeigen
notifications$.next({ id: 1, message: 'Neue Nachricht erhalten' });

// Zum manuellen Ausblenden
// dismiss$.next(1);
```

## Häufige Fehler

### 1. Vergessen abzubestellen mit zweitem Argument

```typescript
// ❌ Falsch - Mit zweitem Argument läuft unendlich
import { timer } from 'rxjs';

function startTimer() {
  timer(1000, 1000).subscribe(value => {
    console.log('Wert:', value); // Läuft ewig
  });
}

startTimer();

// ✅ Richtig - Abonnement halten und bei Bedarf abbestellen
import { timer, Subscription } from 'rxjs';
import { take } from 'rxjs';

let subscription: Subscription | null = null;

function startTimer() {
  subscription = timer(1000, 1000).pipe(
    take(10) // Automatisch nach 10 Mal beenden
  ).subscribe(value => {
    console.log('Wert:', value);
  });
}

function stopTimer() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startTimer();
```

### 2. Unterschied zu interval() nicht verstanden

```typescript
// ❌ Verwechslung - interval() startet sofort (erster Wert nach 1 Sekunde)
import { interval } from 'rxjs';

interval(1000).subscribe(value => {
  console.log('interval:', value); // 0 wird nach 1 Sekunde ausgegeben
});

// ✅ timer() - Wenn erster Wert sofort ohne Verzögerung ausgegeben werden soll
import { timer } from 'rxjs';

timer(0, 1000).subscribe(value => {
  console.log('timer:', value); // 0 wird sofort ausgegeben
});
```

## Leistungsüberlegungen

`timer()` ist leichtgewichtig, kann aber je nach Verwendung die Leistung beeinträchtigen.

> [!TIP]
> **Optimierungstipps**:
> - Kein zweites Argument für einmalige Ausführung angeben
> - Bei Nichtbedarf sicherstellen abzubestellen
> - Bei mehreren Observern `share()` zum Teilen verwenden
> - Vorsicht bei kurzen Intervallen (unter 100ms)

```typescript
import { timer } from 'rxjs';
import { share } from 'rxjs';

// ❌ Leistungsproblem - Mehrere unabhängige Timer
const timer$ = timer(0, 1000);

timer$.subscribe(value => console.log('Observer 1:', value));
timer$.subscribe(value => console.log('Observer 2:', value));
// Zwei Timer laufen parallel

// ✅ Optimierung - Einen Timer teilen
const sharedTimer$ = timer(0, 1000).pipe(share());

sharedTimer$.subscribe(value => console.log('Observer 1:', value));
sharedTimer$.subscribe(value => console.log('Observer 2:', value));
// Ein Timer wird geteilt
```

## Verwandte Erstellungsfunktionen

| Funktion | Unterschiede | Verwendung |
|----------|------|----------|
| **[interval()](/de/guide/creation-functions/basic/interval)** | Startet sofort (keine Verzögerung) | Regelmäßige Ausführung ohne Verzögerung |
| **[of()](/de/guide/creation-functions/basic/of)** | Synchron sofortige Ausgabe | Wenn asynchron nicht benötigt |
| **defer()** | Verzögert Verarbeitung bei Abonnement | Dynamische Wertgenerierung |

## Zusammenfassung

- `timer()` ist eine Erstellungsfunktion die nach Verzögerung mit der Ausgabe beginnt
- Ohne zweites Argument: Einmalige Ausgabe (wird abgeschlossen)
- Mit zweitem Argument: Regelmäßige Ausgabe (wird nicht abgeschlossen)
- Verzögerungszeit kann in Millisekunden oder als `Date`-Objekt angegeben werden
- Ideal für verzögerte Ausführung, verzögertes Polling, Timeout-Verarbeitung

## Nächste Schritte

- [interval() - Kontinuierliche Ausgabe in bestimmten Intervallen](/de/guide/creation-functions/basic/interval)
- [defer() - Erzeugung bei Abonnement verzögern](/de/guide/creation-functions/conditional/defer)
- [Zurück zu Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)
