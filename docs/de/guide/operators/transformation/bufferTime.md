---
description: "Der bufferTime-Operator gibt in festen Zeitabständen emittierte Werte als Array aus. Ideal für zeitbasierte Batch-Verarbeitung wie Batch-Übertragung von Echtzeit-Logs, UI-Ereignis-Aggregation oder Netzwerkeffizienzsteigerung. Erklärt den Unterschied zu buffer und typsichere Implementierung mit TypeScript."
---

# bufferTime - Zeitbasierter Puffer

Der `bufferTime`-Operator fasst Werte in angegebenen Zeitintervallen **zusammen und gibt sie als Array** aus.
Praktisch, wenn Sie den Stream in festen Zeitintervallen aufteilen und wie eine Batch-Verarbeitung behandeln möchten.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { bufferTime } from 'rxjs';

// Werte alle 100ms emittieren
const source$ = interval(100);

source$.pipe(
  bufferTime(1000)
).subscribe(buffer => {
  console.log('In 1 Sekunde gesammelte Werte:', buffer);
});

// Ausgabebeispiel:
// In 1 Sekunde gesammelte Werte: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
// In 1 Sekunde gesammelte Werte: [10, 11, 12, 13, 14, 15, 16, 17, 18, 19]
// ...
```

- In 1 Sekunde emittierte Werte werden als Array zusammengefasst und nacheinander ausgegeben.

[🌐 RxJS Offizielle Dokumentation - `bufferTime`](https://rxjs.dev/api/operators/bufferTime)

## 💡 Typische Anwendungsmuster

- Batch-Übertragung in festen Zeitintervallen
- Zusammengefasste Verarbeitung von Benutzeroperationen (z.B. Drag-Operationen)
- Datensammlung von Sensoren oder IoT-Geräten
- Ausdünnung und Kompression von Log- oder Trace-Informationen

## 🧠 Praktisches Codebeispiel (mit UI)

Puffert Klick-Ereignisse 1 Sekunde lang und gibt sie jede Sekunde zusammen aus.

```ts
import { fromEvent } from 'rxjs';
import { bufferTime } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Klick-Ereignisstrom
const clicks$ = fromEvent(document, 'click');

clicks$.pipe(
  bufferTime(1000)
).subscribe(clickArray => {
  const message = `Klicks in 1 Sekunde: ${clickArray.length}`;
  console.log(message);
  output.textContent = message;
});
```

- Zeigt zusammengefasst an, wie oft in 1 Sekunde geklickt wurde.
- Durch Pufferungsverarbeitung können aufeinanderfolgende Ereignissse zusammen verwaltet werden.
