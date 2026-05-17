---
description: raceWith ist ein RxJS-Kombinationsoperator, der nur den Stream übernimmt, der als erster vom ursprünglichen Observable und anderen Observables einen Wert ausgibt. Ideal für Timeout-Implementierung, Fallback-Verarbeitung und Auswahl des schnellsten aus mehreren Datenquellen, wenn eine Auswahl basierend auf Konkurrenzbedingungen erforderlich ist. Als Pipeable Operator bequem für die Verwendung in Pipelines.
---

# raceWith - Den schnellsten Stream in der Pipeline übernehmen

Der `raceWith`-Operator übernimmt **nur das Observable, das zuerst einen Wert ausgibt** vom ursprünglichen Observable und anderen angegebenen Observables,
und ignoriert danach alle anderen Observables.
Dies ist die Pipeable Operator-Version der Creation Function `race`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langsam (5 Sekunden)'));
const medium$ = timer(3000).pipe(map(() => 'Mittel (3 Sekunden)'));
const fast$ = timer(2000).pipe(map(() => 'Schnell (2 Sekunden)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Ausgabe: Schnell (2 Sekunden)
```

- Nur das Observable, das zuerst einen Wert ausgibt (in diesem Beispiel `fast$`) wird Gewinner und setzt den Stream fort.
- Andere Observables werden ignoriert.

[🌐 RxJS Official Documentation - `raceWith`](https://rxjs.dev/api/operators/raceWith)


## 💡 Typische Anwendungsmuster

- **Timeout-Implementierung**: Hauptverarbeitung und Timeout-Timer konkurrieren lassen
- **Fallback-Verarbeitung**: Das schnellste aus mehreren Datenquellen übernehmen
- **Benutzerinteraktion**: Früheren von Klick und automatischem Fortschritt übernehmen


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, bei dem manueller Klick und automatischer Fortschritts-Timer konkurrieren und der frühere übernommen wird.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Manuell fortfahren (innerhalb von 5 Sekunden klicken)';
document.body.appendChild(button);

// Wartemeldung
const waiting = document.createElement('div');
waiting.textContent = 'Bitte klicken Sie innerhalb von 5 Sekunden auf den Button oder warten Sie auf den automatischen Fortschritt...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Manueller Klick-Stream
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Manueller Klick wurde ausgewählt!')
);

// Automatischer Fortschritts-Timer (nach 5 Sekunden)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ Automatischer Fortschritt wurde ausgewählt!')
);

// Rennen durchführen
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

- Wenn der Button innerhalb von 5 Sekunden geklickt wird, wird der manuelle Klick übernommen.
- Nach Ablauf von 5 Sekunden wird der automatische Fortschritt übernommen.
- **Der frühere wird Gewinner** und der spätere wird ignoriert.


## 🔄 Unterschied zur Creation Function `race`

### Grundlegender Unterschied

| | `race` (Creation Function) | `raceWith` (Pipeable Operator) |
|:---|:---|:---|
| **Verwendungsort** | Als unabhängige Funktion | Innerhalb einer `.pipe()`-Kette |
| **Schreibweise** | `race(obs1$, obs2$, obs3$)` | `obs1$.pipe(raceWith(obs2$, obs3$))` |
| **Erster Stream** | Alle gleichwertig behandelt | Als Hauptstream behandelt |
| **Vorteil** | Einfach und lesbar | Leicht mit anderen Operatoren kombinierbar |

### Konkrete Beispiele für die Auswahl

**Wenn nur einfaches Rennen → Creation Function empfohlen**

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Antwort von Server 1'));
const server2$ = timer(2000).pipe(map(() => 'Antwort von Server 2'));
const server3$ = timer(4000).pipe(map(() => 'Antwort von Server 3'));

// Einfach und lesbar
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Übernommen:', response);
});
// Ausgabe: Übernommen: Antwort von Server 2 (schnellste 2 Sekunden)
```

**Wenn Transformation zum Hauptstream hinzugefügt werden soll → Pipeable Operator empfohlen**

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Suchen';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Hauptstream: Benutzer-Suchanfrage
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Suche läuft...';

    // API-Aufruf simulieren (dauert 3 Sekunden)
    return timer(3000).pipe(
      map(() => '🔍 Suchergebnis: 100 Treffer'),
      catchError((err: unknown) => of('❌ Ein Fehler ist aufgetreten'))
    );
  })
);

// ✅ Pipeable Operator-Version - vollständig in einer Pipeline
userSearch$
  .pipe(
    raceWith(
      // Timeout (2 Sekunden)
      timer(2000).pipe(
        map(() => '⏱️ Timeout: Die Suche dauert zu lange')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation Function-Version - Hauptstream muss separat geschrieben werden
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Timeout: Die Suche dauert zu lange')
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

// UI erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Datenabruf (mit Fallback)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Datenabruf starten';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Abruf läuft...';

  // Haupt-API (bevorzugt): scheitert manchmal
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ Erfolgreicher Abruf von Haupt-API');
      } else {
        return throwError(() => new Error('Haupt-API fehlgeschlagen'));
      }
    }),
    catchError((err: unknown) => {
      console.log('Haupt-API fehlgeschlagen, zu Fallback wechseln...');
      // Bei Fehler verzögern, um Fallback den Vortritt zu lassen
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable Operator-Version - Fallback zu Haupt-API hinzufügen
  mainApi$
    .pipe(
      raceWith(
        // Backup-API (Fallback): etwas langsamer, aber zuverlässig
        timer(2000).pipe(
          map(() => '🔄 Abruf von Backup-API')
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

**Schnellsten aus mehreren Datenquellen übernehmen**

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>Schnellstes Laden von mehreren CDNs</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Bibliothek laden';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Laden läuft...';

    // Von CDN1 laden (simuliert)
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable Operator-Version - CDN1 als Hauptstream, andere CDNs als Konkurrenten hinzufügen
    return cdn1$.pipe(
      raceWith(
        // Von CDN2 laden (simuliert)
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // Von CDN3 laden (simuliert)
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asien)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Laden abgeschlossen</strong><br>
    Quelle: ${response.source}<br>
    Datei: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Zusammenfassung

- **`race`**: Optimal, wenn das schnellste aus mehreren Streams einfach übernommen werden soll
- **`raceWith`**: Optimal, wenn dem Hauptstream Transformationen oder Verarbeitungen hinzugefügt werden sollen, während Timeout oder Fallback implementiert wird


## ⚠️ Wichtige Hinweise

### Beispiel für Timeout-Implementierung

Implementierung der Timeout-Verarbeitung mit `raceWith`

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Zeitaufwändige Verarbeitung (3 Sekunden)
const slowRequest$ = of('Datenabruf erfolgreich').pipe(delay(3000));

// Timeout (2 Sekunden)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Timeout')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Ausgabe: Timeout
```

### Alle Streams werden abonniert

`raceWith` abonniert alle Observables, bis ein Gewinner feststeht.
Nachdem der Gewinner feststeht, werden die verlorenen Observables automatisch abbestellt.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ ausgelöst')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ ausgelöst')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Ausgabe:
// fast$ ausgelöst
// fast
// (slow$ wird nach 1 Sekunde abbestellt und löst nach 3 Sekunden nicht aus)
```

### Bei synchronen Observables

Wenn alle synchron ausgegeben werden, wird das zuerst registrierte zum Gewinner.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Ausgabe: A (da zuerst registriert)
```


## 📚 Verwandte Operatoren

- **[race](/de/guide/creation-functions/selection/race)** - Creation Function-Version
- **[timeout](/de/guide/operators/utility/timeout)** - Spezialisierter Timeout-Operator
- **[mergeWith](/de/guide/operators/combination/mergeWith)** - Alle Streams zusammenführen
