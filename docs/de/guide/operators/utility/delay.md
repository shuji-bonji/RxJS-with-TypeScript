---
description: "Der delay-Operator verzögert die Emission jedes Werts in einem Observable um eine angegebene Zeit. Er ist effektiv für UI-Effekte, Ratenbegrenzung, Steuerung asynchroner Prozesse und Verzögerungssimulation in Tests. Unterschiede zu delayWhen und typsichere Implementierung mit TypeScript werden anhand praktischer Codebeispiele erklärt."
---

# delay - Verzögerung von Werten

Der `delay`-Operator wird verwendet, um die Emission jedes Werts in einem Stream um eine bestimmte Zeit zu verzögern.
Er ist nützlich für Animationseffekte oder die Anpassung des Timings von Benutzer-Feedbacks.


## 🔰 Grundlegende Syntax und Funktionsweise

Dies ist die minimale Konfiguration, um einen Wert nach einer bestimmten Zeit zu emittieren.

```ts
import { of } from 'rxjs';
import { delay } from 'rxjs';

of('Hello')
  .pipe(
    delay(1000) // Wert wird nach 1 Sekunde emittiert
  )
  .subscribe(console.log);
// Ausgabe:
// Hello
```

In diesem Beispiel wird der durch `of('Hello')` erzeugte Wert 1 Sekunde verzögert von `subscribe()` empfangen.

[🌐 RxJS Offizielle Dokumentation - delay](https://rxjs.dev/api/index/function/delay)

## 💡 Typische Anwendungsfälle

Hier ist ein Beispiel, wie delay verwendet wird, um das Emissions-Timing anzupassen, wenn mehrere Werte emittiert werden.

```ts
import { of } from 'rxjs';
import { delay, concatMap } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    concatMap(
      (val, index) => of(val).pipe(delay(1000 * index)) // A sofort, B nach 1 Sek., C nach 2 Sek.
    )
  )
  .subscribe(console.log);
// Ausgabe:
// A
// B
```

Auf diese Weise ist es möglich, durch Kombination mit `concatMap` für jeden Wert eine individuelle Verzögerung festzulegen.


## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { of } from 'rxjs';
import { delay, tap } from 'rxjs';

// Ausgabebereich
const delayOutput = document.createElement('div');
delayOutput.innerHTML = '<h3>Beispiel für delay:</h3>';
document.body.appendChild(delayOutput);

// Funktion zur Anzeige der aktuellen Uhrzeit
function addTimeLog(message: string) {
  const now = new Date();
  const time =
    now.toLocaleTimeString('de-DE', { hour12: false }) +
    '.' +
    now.getMilliseconds().toString().padStart(3, '0');

  const logItem = document.createElement('div');
  logItem.textContent = `${time}: ${message}`;
  delayOutput.appendChild(logItem);
}

// Startzeit aufzeichnen
addTimeLog('Start');

// Wertesequenz
of('A', 'B', 'C')
  .pipe(
    tap((val) => addTimeLog(`Wert ${val} wird vor Emission ausgegeben`)),
    delay(1000), // 1 Sekunde Verzögerung
    tap((val) => addTimeLog(`Wert ${val} wurde nach 1 Sekunde emittiert`))
  )
  .subscribe();
```


## ✅ Zusammenfassung

- `delay` ist ein Operator zur **Steuerung des Output-Timings eines Observables**
- Es ist möglich, eine konstante Verzögerung anzuwenden oder durch Kombination mit `concatMap` die **Verzögerung für jeden Wert zu steuern**
- Praktisch für **asynchrone Anpassungen zur UX-Verbesserung** wie UI-Output oder Timer-Effekte
