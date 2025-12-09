---
description: Der repeat-Operator führt den gesamten Stream eine angegebene Anzahl von Malen erneut aus, nachdem das Quell-Observable normal abgeschlossen wurde. Er kann für regelmäßige Polling-Verarbeitung, wiederholende Animationen und Szenarien eingesetzt werden, die eine andere Steuerung als retry erfordern.
---

# repeat - Stream-Wiederholung

Der `repeat`-Operator führt **den gesamten Stream eine angegebene Anzahl von Malen erneut aus**, nachdem das Quell-Observable **normal abgeschlossen wurde**.
Er ist nützlich für Polling-Verarbeitung, wiederholende Animationen und Steuerung, die sich von Retry unterscheidet.

## 🔰 Grundlegende Syntax und Funktionsweise

Die einfachste Verwendung ist eine Konfiguration, die eine Wertereihe eine bestimmte Anzahl von Malen wiederholt.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Das Ganze 2 Mal wiederholen (insgesamt 2 Mal ausgegeben)
  )
  .subscribe(console.log);
// Ausgabe:
// A
// B
// A
// B
```

[🌐 RxJS Offizielle Dokumentation - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Typische Anwendungsfälle

Zum Beispiel wird es für einfache Polling-Verarbeitung oder wiederholende Display-Animationen verwendet.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Datenabruf erfolgreich')
  .pipe(
    tap(() => console.log('Request gestartet')),
    delay(1000),
    repeat(3) // 3 Mal wiederholen
  )
  .subscribe(console.log);
// Ausgabe:
// Request gestartet
// ✅ Datenabruf erfolgreich
// Request gestartet
// ✅ Datenabruf erfolgreich
// Request gestartet
// ✅ Datenabruf erfolgreich
```

In diesem Beispiel wird "Request → Datenabruf" alle 1 Sekunde 3 Mal wiederholt.

## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Ausgabebereich
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>Beispiel für repeat:</h3>';
document.body.appendChild(repeatOutput);

// Anzeige der Wiederholungsanzahl
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Wiederholungen: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Werte-Ausgabebereich
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Sequenz-Wiederholung
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Wiederholungen: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Wert: ${val} (Wiederholung ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Zusammenfassung

- `repeat` führt **das Ganze erneut aus, nachdem das Observable normal abgeschlossen wurde**
- Anders als `retry` wird es **bei Fehlern nicht erneut ausgeführt**
- Verwendbar für Polling-Verarbeitung oder **Blink-Animationen von Platzhaltern** und andere wiederholende Animationen
