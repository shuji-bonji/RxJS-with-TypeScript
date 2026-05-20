---
description: "raceWith ist ein RxJS-Join-Operator, der aus dem ursprünglichen Observable und anderen Observables nur den Stream übernimmt, der den Wert zuerst ausgegeben hat. Er ist ideal für Timeout-Implementierungen, Fallback-Prozesse, die schnellste Übernahme aus mehreren Datenquellen und andere Situationen, in denen eine Auswahl nach Race-Bedingungen erforderlich ist; die pipe-fähige Version des Operators ist für die Verwendung in Pipelines geeignet."
---

# raceWith - übernimmt den schnellsten Stream

Der Operator `raceWith` **übernimmt** nur das erste Observable, das einen Wert aus dem ursprünglichen Observable und den anderen angegebenen Observables ausgibt,
danach ignoriert er die anderen Observable.
Dies ist die Pipeable Operator-Version von Creation Function's `race`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langsam. (5Sekunden)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Sekunden)'));
const fast$ = timer(2000).pipe(map(() => 'Schnell (2Sekunden)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Ausgabe: Schnell (2Sekunden)
```

- Nur das Observable, das den Wert zuerst ausgegeben hat (in diesem Beispiel `fast$`), ist der Gewinner und fährt mit den nachfolgenden Streams fort.
- Andere Observable werden ignoriert.

[🌐 Offizielle RxJS Dokumentation - `raceWith`](https://rxjs.dev/api/operators/raceWith)

## 💡 Typisches Nutzungsmuster.

- **Timeout-Implementierung**: den Timeout-Timer gegen den Hauptprozess antreten lassen.
- **Fallback-Verarbeitung**: aus mehreren Datenquellen die schnellste auswählen
- **Benutzerinteraktion**: die schnellere von Klick und automatischem Fortschreiten übernehmen

## 🧠 Praktische Code-Beispiele (mit UI)

Dies ist ein Beispiel für einen Wettlauf zwischen manuellem Klick und automatischem Fortschritts-Timer und der Übernahme des schnelleren.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Erstellung des Ausgabebereichs
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Praktische Beispiele für:</h3>';
document.body.appendChild(output);

// Erstellung von Schaltflächen
const button = document.createElement('button');
button.textContent = 'Manuell vorgehen (5Klick innerhalb von Sekunden)';
document.body.appendChild(button);

// Wartende Meldung
const waiting = document.createElement('div');
waiting.textContent = '5Klicken Sie innerhalb von Sekunden auf die Schaltfläche oder warten Sie auf die automatische Weiterschaltung...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Manueller Klick-Stream
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Manueller Klick wurde ausgewählt！')
);

// Automatischer Fortschrittstimer (5(nach 2,5 Sekunden)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Automatischer Fortschritt wurde ausgewählt！')
);

// Ausführung des Rennens
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- Der manuelle Klick wird angenommen, wenn die Schaltfläche innerhalb von 5 Sekunden angeklickt wird.
- Nach 5 Sekunden wird die automatische Progression übernommen.
- **Der schnellere ist der Gewinner**, der langsamere wird ignoriert.

## 🔄 Unterschiede zur Creation Function `race`.

### Grundlegende Unterschiede.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langsam. (5Sekunden)'));
const medium$ = timer(3000).pipe(map(() => 'Normal (3Sekunden)'));
const fast$ = timer(2000).pipe(map(() => 'Schnell (2Sekunden)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Ausgabe: Schnell (2Sekunden)
```

### Spezifische Beispiele für die Verwendung

**Wenn Sie nur ein einfaches Rennen wollen, ist die Creation Function die richtige Wahl**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Server1Antwort von'));
const server2$ = timer(2000).pipe(map(() => 'Server2Antwort von'));
const server3$ = timer(4000).pipe(map(() => 'Server3Antwort von'));

// Einfach und leicht zu lesen
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Angenommene:', response);
});
// Ausgabe: Angenommene: Server2Antwort von (Schnellste2Sekunden)
```

**Wenn Sie dem Hauptstrom einen Konvertierungsprozess hinzufügen wollen, empfiehlt sich der Pipeable Operator**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Suche';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hauptströmung: Suchanfragen der Benutzer
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Durchsuchen...';

    // APISimulieren Sie einen Anruf (3(dauert Sekunden)
    return timer(3000).pipe(
      map(() => '🔍 Suchergebnisse: 100Treffer'),
      catchError((err: unknown) => of('❌ Fehler aufgetreten'))
    );
  })
);

// ✅ Pipeable OperatorAusgabe - Abgeschlossen in einer Pipeline
userSearch$
  .pipe(
    raceWith(
      // Zeitüberschreitung (in2Sekunden)
      timer(2000).pipe(
        map(() => '⏱️ Zeitüberschreitung (s): Die Suche dauert zu lange')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionAusgabe - Mainstream muss separat geschrieben werden
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Zeitüberschreitung (s): Die Suche dauert zu lange')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementierung der Fallback-Verarbeitung**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UIErstellung
const output = document.createElement('div');
output.innerHTML = '<h3>Datenerfassung (mit Fall-Back)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Beginn der Datenerfassung';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Während der Erfassung...';

  // HauptAPI(Vorrangig)：Zeit々Fehlschlag
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ HauptAPIErfolgreiche Erfassung von');
      } else {
        return throwError(() => new Error('HauptAPIScheitern'));
      }
    }),
    catchError((err: unknown) => {
      console.log('HauptAPIFehler, abgetreten an Fallback...');
      // Verzögerung bei Fehler, Abtretung an Fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorAusgabe - HauptAPIFallback hinzufügen zu
  mainApi$
    .pipe(
      raceWith(
        // SicherungAPI(Rückfall)：Geringfügig langsamer, aber zuverlässig
        timer(2000).pipe(
          map(() => '🔄 SicherungAPIAbgerufen von')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Haupt') ? 'green' : 'orange';
      }
    });
});
```

**Aufnahme der schnellsten Daten aus mehreren Quellen**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MehrereCDNSchnellstes Laden aus</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Bibliothek laden';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Laden von...';

    // CDN1Laden (simuliert) von
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorAusgabe - CDN1in der Haupt- und anderenCDNals Konkurrenten.
    return cdn1$.pipe(
      raceWith(
        // CDN2Laden (simuliert) von
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Laden (simuliert) von
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Laden abgeschlossen</strong><br>
    Erfasst von: ${response.source}<br>
    Datei: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Zusammenfassung.

- **`race`**: ideal, wenn Sie einfach den schnellsten von mehreren Streams übernehmen wollen
- raceWith**: ideal, wenn Sie Timeouts und Fallbacks implementieren wollen, während Sie den Hauptstrom transformieren und verarbeiten

## ⚠️ Anmerkungen.

### Beispiel einer Timeout-Implementierung

Implementierung der Timeout-Behandlung mit `raceWith`.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Zeitaufwändiger Prozess (3Sekunden)
const slowRequest$ = of('Erfolgreiche Datenerfassung').pipe(delay(3000));

// Zeitüberschreitung (in2Sekunden)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Zeitüberschreitung (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Ausgabe: Zeitüberschreitung (s)
```

### Alle Streams sind abonniert auf

raceWith" abonniert alle Observable, bis ein Gewinner feststeht.
Nachdem der Gewinner feststeht, wird das unterlegene Observable automatisch abgemeldet.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ Abschuss')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ Abschuss')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Ausgabe:
// fast$ Abschuss
// fast
// (slow$ist1Abbestellt bei Sekunde,3nicht am Ende der zweiten Periode abgefeuert.)
```

### Für synchrone Observables.

Wenn alles synchron ausgegeben wird, ist der erste registrierte der Gewinner.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Ausgabe: A (weil es zuerst abonniert wurde)
```

## 📚 Verwandte Operatoren.

- **[race](/de/guide/creation-functions/selection/race)** - Creation Function Version
- **[timeout](/de/guide/operators/utility/timeout)** - Nur Timeout-Operator.
- **[mergeWith](/de/guide/operators/combination/mergeWith)** - Alle Streams zusammenführen
