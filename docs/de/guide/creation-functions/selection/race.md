---
description: "race Creation Function wählt den ersten Stream aus. Nützlich für Timeouts, Server-Fallbacks und schnellste Antworten mit TypeScript."
---

# race - Wählt den Stream aus, der zuerst einen Wert emittiert

`race` ist eine spezielle Kombinations-Creation Function, die **nur das Observable am Leben hält, das zuerst einen Wert emittiert**, und alle anderen Observables ignoriert.

## Grundlegende Syntax und Verwendung

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Langsam (5 Sekunden)'));
const fast$ = timer(2000).pipe(map(() => 'Schnell (2 Sekunden)'));

race(slow$, fast$).subscribe(console.log);
// Ausgabe: Schnell (2 Sekunden)
```

- Nur das Observable, das zuerst einen Wert emittiert, wird zum Gewinner und setzt seinen Stream fort.
- Andere Observables werden ignoriert.

[🌐 RxJS Offizielle Dokumentation - `race`](https://rxjs.dev/api/index/function/race)

## Typische Anwendungsmuster

- **Verarbeitung der schnelleren von mehreren Benutzeraktionen (Klick, Tastatureingabe, Scrollen)**
- **Verwendung des schnelleren von mehreren Triggern wie manuellem Senden und automatischem Speichern**
- **Bevorzugte Anzeige der zuerst abgeschlossenen Daten von mehreren Datenabrufprozessen**

## Praktisches Codebeispiel (mit UI)

Simuliert ein Rennen, bei dem nur der erste emittierte Wert von drei Streams mit unterschiedlichem Timing ausgewählt wird.

```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisches Beispiel für race:</h3>';
document.body.appendChild(output);

// Observables mit unterschiedlichem Timing
const slow$ = timer(5000).pipe(map(() => 'Langsam (nach 5 Sekunden)'));
const medium$ = timer(3000).pipe(map(() => 'Mittel (nach 3 Sekunden)'));
const fast$ = timer(2000).pipe(map(() => 'Schnell (nach 2 Sekunden)'));

const startTime = Date.now();

// Rennstart-Nachricht
const waiting = document.createElement('div');
waiting.textContent = 'Rennen gestartet... Warten auf den ersten Stream, der emittiert.';
output.appendChild(waiting);

// race ausführen
race(slow$, medium$, fast$).subscribe(winner => {
  const endTime = Date.now();
  const elapsed = ((endTime - startTime) / 1000).toFixed(2);

  const result = document.createElement('div');
  result.innerHTML = `<strong>Gewinner:</strong> ${winner} (Verstrichene Zeit: ${elapsed} Sekunden)`;
  result.style.color = 'green';
  result.style.marginTop = '10px';
  output.appendChild(result);

  const explanation = document.createElement('div');
  explanation.textContent = '※ Nur das Observable, das zuerst einen Wert emittiert, wird ausgewählt.';
  explanation.style.marginTop = '5px';
  output.appendChild(explanation);
});
```

- Nach 2 Sekunden emittiert `fast$` zuerst und wird ab dann nur noch `fast$` ausgegeben.
- Die Emissionen von `medium$` und `slow$` werden ignoriert.

## Verwandte Operatoren

- **[raceWith](/de/guide/operators/combination/raceWith)** - Pipeable Operator-Version (zur Verwendung in Pipelines)
- **[timeout](/de/guide/operators/utility/timeout)** - Spezieller Timeout-Operator
- **[merge](/de/guide/creation-functions/combination/merge)** - Creation Function zum Zusammenführen aller Streams
