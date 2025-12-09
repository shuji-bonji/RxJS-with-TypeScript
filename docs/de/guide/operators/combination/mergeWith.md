---
description: "mergeWith ist ein Pipeable Operator, der das ursprüngliche Observable und andere Observables gleichzeitig abonniert und parallel verbindet. Kann zum Zusammenführen mehrerer Ereignisquellen für Echtzeit-Verarbeitung verwendet werden. Erklärt Unterschiede zu merge() und typsichere Implementierung in TypeScript."
---

# mergeWith - Mehrere Streams gleichzeitig in der Pipeline verbinden

Der `mergeWith`-Operator **abonniert gleichzeitig** das ursprüngliche Observable und andere angegebene Observables
und führt die von jedem ausgegebenen Werte in Echtzeit zusammen.
Dies ist die Pipeable Operator-Version der Creation Function `merge`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

const source1$ = interval(1000).pipe(
  map(val => `Stream 1: ${val}`),
  take(3)
);

const source2$ = interval(1500).pipe(
  map(val => `Stream 2: ${val}`),
  take(2)
);

source1$
  .pipe(mergeWith(source2$))
  .subscribe(console.log);

// Ausgabebeispiel:
// Stream 1: 0
// Stream 2: 0
// Stream 1: 1
// Stream 1: 2
// Stream 2: 1
```

- Alle Observables werden gleichzeitig abonniert und die Werte fließen **in der Reihenfolge ihrer Ausgabe**.
- Die Reihenfolge ist nicht garantiert und **hängt vom Ausgabezeitpunkt jedes Observables ab**.

[🌐 RxJS Official Documentation - `mergeWith`](https://rxjs.dev/api/operators/mergeWith)


## 💡 Typische Anwendungsmuster

- **Mehrere Ereignisquellen zusammenführen**: Integration von Benutzeraktionen und automatischen Updates
- **Parallele Datenabfragen verbinden**: Antworten von mehreren APIs zu einem einzigen Stream aggregieren
- **Echtzeit-Updates zusammenführen**: WebSocket und Polling integrieren


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das Benutzerklick-Ereignisse und einen automatischen Update-Timer integriert, um Benachrichtigungen anzuzeigen.

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>mergeWith Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Manuelles Update';
document.body.appendChild(button);

// Klick-Stream
const manualUpdate$ = fromEvent(button, 'click').pipe(
  map(() => '👆 Manuelles Update wurde ausgeführt')
);

// Automatischer Update-Timer (alle 5 Sekunden)
const autoUpdate$ = interval(5000).pipe(
  map(val => `🔄 Automatisches Update #${val + 1}`),
  take(3)
);

// Beide integrieren und anzeigen
manualUpdate$
  .pipe(mergeWith(autoUpdate$))
  .subscribe((value) => {
    const timestamp = new Date().toLocaleTimeString();
    const item = document.createElement('div');
    item.textContent = `[${timestamp}] ${value}`;
    output.appendChild(item);
  });
```

- Beim Klicken auf den Button wird sofort das manuelle Update angezeigt,
- Alle 5 Sekunden wird auch das automatische Update parallel ausgeführt.
- Beide Ereignisse werden in Echtzeit integriert.


## 🔄 Unterschied zur Creation Function `merge`

### Grundlegender Unterschied

| | `merge` (Creation Function) | `mergeWith` (Pipeable Operator) |
|:---|:---|:---|
| **Verwendungsort** | Als unabhängige Funktion | Innerhalb einer `.pipe()`-Kette |
| **Schreibweise** | `merge(obs1$, obs2$, obs3$)` | `obs1$.pipe(mergeWith(obs2$, obs3$))` |
| **Erster Stream** | Alle gleichwertig behandelt | Als Hauptstream behandelt |
| **Vorteil** | Einfach und lesbar | Leicht mit anderen Operatoren kombinierbar |

### Konkrete Beispiele für die Auswahl

**Wenn nur einfaches Merging → Creation Function empfohlen**

```ts
import { merge, fromEvent } from 'rxjs';
import { map } from 'rxjs';

const clicks$ = fromEvent(document, 'click').pipe(map(() => 'Klick'));
const moves$ = fromEvent(document, 'mousemove').pipe(map(() => 'Mausbewegung'));
const keypress$ = fromEvent(document, 'keypress').pipe(map(() => 'Tasteneingabe'));

// Einfach und lesbar
merge(clicks$, moves$, keypress$).subscribe(console.log);
// Ausgabe: Wird in der Reihenfolge angezeigt, in der eines der Ereignisse auftritt
```

**Wenn Transformation zum Hauptstream hinzugefügt werden soll → Pipeable Operator empfohlen**

```ts
import { fromEvent, interval } from 'rxjs';
import { mergeWith, map, filter, throttleTime } from 'rxjs';

const userClicks$ = fromEvent(document, 'click');
const autoRefresh$ = interval(30000); // Alle 30 Sekunden

// ✅ Pipeable Operator-Version - vollständig in einer Pipeline
userClicks$
  .pipe(
    throttleTime(1000),           // Verhindern von Mehrfachklicks
    map(() => ({ source: 'user', timestamp: Date.now() })),
    mergeWith(
      autoRefresh$.pipe(
        map(() => ({ source: 'auto', timestamp: Date.now() }))
      )
    ),
    filter(event => event.timestamp > Date.now() - 60000)  // Nur innerhalb der letzten Minute
  )
  .subscribe(event => {
    console.log(`${event.source}-Update: ${new Date(event.timestamp).toLocaleTimeString()}`);
  });

// ❌ Creation Function-Version - wird umständlich
import { merge } from 'rxjs';
merge(
  userClicks$.pipe(
    throttleTime(1000),
    map(() => ({ source: 'user', timestamp: Date.now() }))
  ),
  autoRefresh$.pipe(
    map(() => ({ source: 'auto', timestamp: Date.now() }))
  )
).pipe(
  filter(event => event.timestamp > Date.now() - 60000)
).subscribe(event => {
  console.log(`${event.source}-Update: ${new Date(event.timestamp).toLocaleTimeString()}`);
});
```

**Wenn mehrere Datenquellen integriert werden**

```ts
import { fromEvent, timer } from 'rxjs';
import { mergeWith, map, startWith } from 'rxjs';

// Button erstellen
const saveButton = document.createElement('button');
saveButton.textContent = 'Speichern';
document.body.appendChild(saveButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hauptstream: Benutzer-Speicheraktion
const manualSave$ = fromEvent(saveButton, 'click').pipe(
  map(() => '💾 Manuelles Speichern')
);

// ✅ Pipeable Operator-Version - Automatisches Speichern zum Hauptstream hinzufügen
manualSave$
  .pipe(
    startWith('📝 Bearbeitung gestartet'),
    mergeWith(
      timer(10000, 10000).pipe(map(() => '⏰ Automatisches Speichern'))  // Alle 10 Sekunden automatisches Speichern
    )
  )
  .subscribe(message => {
    const div = document.createElement('div');
    div.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
    output.appendChild(div);
  });
```

### Zusammenfassung

- **`merge`**: Optimal, wenn mehrere Streams gleichwertig integriert werden sollen
- **`mergeWith`**: Optimal, wenn dem Hauptstream Transformationen oder Verarbeitungen hinzugefügt werden sollen, während andere Streams integriert werden


## ⚠️ Wichtige Hinweise

### Abschlusszeitpunkt

Der verbundene Stream wird erst abgeschlossen, wenn alle Observables abgeschlossen sind.

```ts
import { of, interval, NEVER } from 'rxjs';
import { mergeWith, take } from 'rxjs';

of(1, 2, 3).pipe(
  mergeWith(
    interval(1000).pipe(take(2)),
    // NEVER  // ← Wird nie abgeschlossen, wenn dies hinzugefügt wird
  )
).subscribe({
  next: console.log,
  complete: () => console.log('✅ Abgeschlossen')
});
// Ausgabe: 1 → 2 → 3 → 0 → 1 → ✅ Abgeschlossen
```

### Kontrolle der parallelen Ausführungsanzahl

Standardmäßig werden alle Streams gleichzeitig ausgeführt, aber in Kombination mit `mergeMap` kann dies kontrolliert werden.

```ts
import { from, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

from([1, 2, 3, 4, 5]).pipe(
  mergeMap(
    val => of(val).pipe(delay(1000)),
    2  // Maximal 2 parallel ausführen
  )
).subscribe(console.log);
```

### Fehlerbehandlung

Wenn in einem der Observables ein Fehler auftritt, endet der gesamte Stream mit einem Fehler.

```ts
import { throwError, interval } from 'rxjs';
import { mergeWith, take, catchError } from 'rxjs';
import { of } from 'rxjs';

interval(1000).pipe(
  take(2),
  mergeWith(
    throwError(() => new Error('Fehler aufgetreten')).pipe(
      catchError(err => of('Fehler behoben'))
    )
  )
).subscribe({
  next: console.log,
  error: err => console.error('Fehler:', err.message)
});
// Ausgabe: 0 → Fehler behoben → 1
```


## 📚 Verwandte Operatoren

- **[merge](/de/guide/creation-functions/combination/merge)** - Creation Function-Version
- **[concatWith](/de/guide/operators/combination/concatWith)** - Sequentielle Verbindung, Pipeable-Version
- **[mergeMap](/de/guide/operators/transformation/mergeMap)** - Paralleles Mapping jedes Werts
