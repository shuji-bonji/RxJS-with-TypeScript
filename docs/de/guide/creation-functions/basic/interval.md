---
description: "interval() - Erstellungsfunktion, die kontinuierlich Werte (fortlaufende Zahlen ab 0) in einem bestimmten Intervall ausgibt. Ideal für Polling, regelmäßige Ausführung und Animationssteuerung. Unterschiede zu timer(), Begrenzung mit take(), typsichere TypeScript-Implementierung und Abbestellmuster zur Vermeidung von Speicherlecks."
---

# interval() - Kontinuierliche Ausgabe in bestimmten Intervallen

`interval()` ist eine Erstellungsfunktion, die kontinuierlich Werte in einem bestimmten Zeitintervall ausgibt.

## Überblick

`interval()` gibt fortlaufende Zahlen ab 0 in dem angegebenen Millisekundenintervall aus. Es wird häufig für Polling-Verarbeitung und regelmäßige Aufgabenausführung verwendet.

**Signatur**:
```typescript
function interval(period: number = 0, scheduler: SchedulerLike = asyncScheduler): Observable<number>
```

**Offizielle Dokumentation**: [📘 RxJS Official: interval()](https://rxjs.dev/api/index/function/interval)

## Grundlegende Verwendung

`interval()` gibt Zahlen aus, die im angegebenen Intervall hochzählen.

```typescript
import { interval } from 'rxjs';

// Gibt jeden Sekunde einen Wert aus
const interval$ = interval(1000);

interval$.subscribe(value => {
  console.log('Wert:', value);
});

// Ausgabe (jede Sekunde):
// Wert: 0
// Wert: 1
// Wert: 2
// Wert: 3
// ...(unendlich)
```

## Wichtige Merkmale

### 1. Fortlaufende Zahlen ab 0

`interval()` beginnt immer bei 0 und gibt um 1 erhöhte Ganzzahlen aus.

```typescript
import { interval } from 'rxjs';
import { take } from 'rxjs';

interval(500).pipe(
  take(5) // Nur die ersten 5 Werte nehmen
).subscribe(value => console.log(value));

// Ausgabe (alle 500ms):
// 0
// 1
// 2
// 3
// 4
```

### 2. Endet nicht (unendlicher Stream)

`interval()` endet nicht automatisch, daher ist **Abbestellen zwingend erforderlich**.

```typescript
import { interval } from 'rxjs';

const subscription = interval(1000).subscribe(value => {
  console.log('Wert:', value);
});

// Nach 5 Sekunden abbestellen
setTimeout(() => {
  subscription.unsubscribe();
  console.log('Gestoppt');
}, 5000);
```

> [!WARNING]
> **Vergessen abzubestellen führt zu Speicherlecks**
>
> `interval()` gibt unendlich Werte aus. Das Vergessen des Abbestellens führt zu Speicherlecks und Leistungsproblemen. Rufen Sie unbedingt `unsubscribe()` auf oder verwenden Sie Operatoren wie `take()`, `takeUntil()`, `takeWhile()` für automatische Vervollständigung.

### 3. Cold Observable

`interval()` ist eine Cold Observable, und für jedes Abonnement wird ein unabhängiger Timer erstellt.

```typescript
import { interval } from 'rxjs';

const interval$ = interval(1000);

// Abonnement 1
interval$.subscribe(value => console.log('Observer 1:', value));

// 2 Sekunden später Abonnement 2 hinzufügen
setTimeout(() => {
  interval$.subscribe(value => console.log('Observer 2:', value));
}, 2000);

// Ausgabe:
// Observer 1: 0
// Observer 1: 1
// Observer 2: 0  ← Beginnt unabhängig bei 0
// Observer 1: 2
// Observer 2: 1
```

> [!NOTE]
> **Cold Observable Merkmale**
> - Jedes Abonnement startet eine unabhängige Ausführung
> - Jeder Abonnent erhält seinen eigenen Datenstrom
> - Bei jedem Abonnement wird ein unabhängiger Timer gestartet. Verwenden Sie `share()` wenn Daten geteilt werden müssen.
>
> Weitere Informationen finden Sie unter [Cold Observable und Hot Observable](/de/guide/observables/cold-and-hot-observables).

## interval() vs timer() Unterschiede

`interval()` und `timer()` sind ähnlich, haben aber einige wichtige Unterschiede.

```typescript
import { interval, timer } from 'rxjs';
import { take } from 'rxjs';

// interval() - Startet sofort, kontinuierliche Ausgabe
interval(1000).pipe(take(3)).subscribe(value => {
  console.log('interval:', value);
});

// timer() - Startet nach Verzögerung
timer(2000, 1000).pipe(take(3)).subscribe(value => {
  console.log('timer:', value);
});

// Ausgabe:
// interval: 0  (nach 1 Sekunde)
// interval: 1  (nach 2 Sekunden)
// timer: 0     (nach 2 Sekunden)
// interval: 2  (nach 3 Sekunden)
// timer: 1     (nach 3 Sekunden)
// timer: 2     (nach 4 Sekunden)
```

| Erstellungsfunktion | Startzeitpunkt | Verwendung |
|-------------------|--------------|------|
| `interval(1000)` | Startet sofort (erster Wert nach 1 Sekunde) | Regelmäßige Ausführung |
| `timer(2000, 1000)` | Startet nach angegebener Zeit | Verzögerte regelmäßige Ausführung |
| `timer(2000)` | Nur einmalige Ausgabe nach angegebener Zeit | Verzögerte Ausführung |

## Praktische Anwendungsfälle

### 1. API-Polling

Ruft in regelmäßigen Abständen eine API auf und aktualisiert Daten.

```typescript
import { from, interval } from 'rxjs';
import { switchMap, catchError } from 'rxjs';
import { of } from 'rxjs';

interface Status {
  status: string;
  timestamp: number;
}

function fetchStatus(): Promise<Status> {
  return fetch('https://jsonplaceholder.typicode.com/users/1')
    .then(res => res.json());
}

// Alle 5 Sekunden API pollen
const polling$ = interval(5000).pipe(
  switchMap(() => from(fetchStatus())),
  catchError(error => {
    console.error('API Error:', error);
    return of({ status: 'error', timestamp: Date.now() });
  })
);

const subscription = polling$.subscribe(data => {
  console.log('Status-Update:', data);
});

// Bei Bedarf stoppen
// subscription.unsubscribe();
```

### 2. Countdown-Timer

Implementiert einen Countdown mit Zeitlimit.

```typescript
import { interval } from 'rxjs';
import { map, takeWhile } from 'rxjs';

const countdown$ = interval(1000).pipe(
  map(count => 10 - count), // Countdown von 10 Sekunden
  takeWhile(time => time >= 0) // Automatisch bei 0 beenden
);

countdown$.subscribe({
  next: time => console.log(`Verbleibende Zeit: ${time} Sekunden`),
  complete: () => console.log('Zeit abgelaufen!')
});

// Ausgabe (jede Sekunde):
// Verbleibende Zeit: 10 Sekunden
// Verbleibende Zeit: 9 Sekunden
// ...
// Verbleibende Zeit: 0 Sekunden
// Zeit abgelaufen!
```

### 3. Echtzeit-Uhranzeige

Aktualisiert die aktuelle Zeit in Echtzeit.

```typescript
import { interval } from 'rxjs';
import { map } from 'rxjs';

// Element für Uhranzeige erstellen
const clockElement = document.createElement('div');
clockElement.id = 'clock';
clockElement.style.fontSize = '24px';
clockElement.style.fontFamily = 'monospace';
clockElement.style.padding = '20px';
document.body.appendChild(clockElement);

const clock$ = interval(1000).pipe(
  map(() => new Date().toLocaleTimeString())
);

clock$.subscribe(time => {
  clockElement.textContent = time;
});

// Ausgabe: Aktuelle Zeit wird jede Sekunde aktualisiert
```

## Häufige Fehler

### 1. Vergessen abzubestellen

```typescript
// ❌ Falsch - Läuft unendlich ohne Abbestellen
import { interval } from 'rxjs';

function startPolling() {
  interval(1000).subscribe(value => {
    console.log('Wert:', value); // Läuft ewig
  });
}

startPolling();

// ✅ Richtig - Abonnement halten und bei Bedarf abbestellen
import { interval, Subscription } from 'rxjs';

let subscription: Subscription | null = null;

function startPolling() {
  subscription = interval(1000).subscribe(value => {
    console.log('Wert:', value);
  });
}

function stopPolling() {
  if (subscription) {
    subscription.unsubscribe();
    subscription = null;
  }
}

startPolling();
// stopPolling() bei Bedarf aufrufen
```

### 2. Mehrere unabhängige Timer bei Mehrfachabonnement

```typescript
// ❌ Unbeabsichtigt - Zwei unabhängige Timer werden erstellt
import { interval } from 'rxjs';

const interval$ = interval(1000);

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Zwei Timer laufen parallel

// ✅ Richtig - Einen Timer teilen
import { interval } from 'rxjs';
import { share } from 'rxjs';

const interval$ = interval(1000).pipe(share());

interval$.subscribe(value => console.log('Observer 1:', value));
interval$.subscribe(value => console.log('Observer 2:', value));
// Ein Timer wird geteilt
```

## Leistungsüberlegungen

`interval()` ist leichtgewichtig, aber bei kurzen Intervallen ist Vorsicht bei der Leistung geboten.

> [!TIP]
> **Optimierungstipps**:
> - Unnötige Verarbeitung nicht ausführen (mit `filter()` eingrenzen)
> - Vorsicht bei kurzen Intervallen (unter 100ms)
> - Sicherstellen dass abbestellt wird
> - Bei mehreren Observern `share()` zum Teilen verwenden

```typescript
import { interval } from 'rxjs';
import { filter, share } from 'rxjs';

// ❌ Leistungsproblem - Schwere Verarbeitung alle 100ms
interval(100).subscribe(() => {
  heavyCalculation();
});

// ✅ Optimierung - Nur bei Bedarf verarbeiten
interval(100).pipe(
  filter(count => count % 10 === 0), // Einmal pro Sekunde (jedes 10. Mal)
  share() // Zwischen mehreren Observern teilen
).subscribe(() => {
  heavyCalculation();
});
```

## Verwandte Erstellungsfunktionen

| Funktion | Unterschiede | Verwendung |
|----------|------|----------|
| **[timer()](/de/guide/creation-functions/basic/timer)** | Startet nach Verzögerung, oder nur einmalige Ausgabe | Verzögerte Ausführung oder einmalige Verarbeitung |
| **[fromEvent()](/de/guide/creation-functions/basic/fromEvent)** | Ereignisgesteuert | Verarbeitung basierend auf Benutzeraktionen |
| **range()** | Gibt Zahlenbereich sofort aus | Wenn keine Zeitsteuerung benötigt wird |

## Zusammenfassung

- `interval()` gibt kontinuierlich Werte in bestimmten Intervallen aus
- Gibt fortlaufende Ganzzahlen ab 0 aus
- Endet nicht automatisch, Abbestellen ist zwingend erforderlich
- Funktioniert als Cold Observable (unabhängiger Timer pro Abonnement)
- Ideal für Polling, regelmäßige Ausführung, Countdowns

## Nächste Schritte

- [timer() - Start nach Verzögerung](/de/guide/creation-functions/basic/timer)
- [fromEvent() - Ereignis in Observable umwandeln](/de/guide/creation-functions/basic/fromEvent)
- [Zurück zu Grundlegende Erstellungsfunktionen](/de/guide/creation-functions/basic/)
