---
description: "Der scan-Operator ist ein RxJS-Operator, der Werte sequenziell akkumuliert und Zwischenergebnisse ausgibt. Im Gegensatz zu reduce() gibt er bei jedem eingehenden Wert ein Ergebnis aus und wird daher für Echtzeit-Aggregation, Zustandsverwaltung, kumulative Zähler und Streaming-Berechnungen verwendet. Erklärung der TypeScript-typsicheren Implementierung."
---

# scan - Werte akkumulativ erzeugen

Der `scan`-Operator wendet eine Akkumulationsfunktion auf jeden Wert des Streams an und gibt **sequenzielle Zwischenergebnisse** aus.
Ähnlich wie `Array.prototype.reduce` für Arrays, unterscheidet es sich jedoch dadurch, dass Zwischenergebnisse ausgegeben werden, bevor alle Werte eintreffen.

## 🔰 Grundsyntax und Verwendung

```ts
import { of } from 'rxjs';
import { scan } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(scan((acc, curr) => acc + curr, 0))
  .subscribe(console.log);

// Ausgabe: 1, 3, 6, 10, 15

```

- `acc` ist der akkumulierte Wert, `curr` ist der aktuelle Wert.
- Beginnt mit dem Anfangswert (in diesem Fall `0`) und akkumuliert sequenziell.

[🌐 RxJS Offizielle Dokumentation - `scan`](https://rxjs.dev/api/operators/scan)

## 💡 Typische Anwendungsmuster

- Hochzählen oder Score-Aggregation
- Verwaltung des Echtzeit-Validierungsstatus von Formularen
- Akkumulierte Verarbeitung gepufferter Ereignisse
- Aufbau von Daten für Echtzeit-Aggregationsdiagramme

## 🧠 Praktisches Codebeispiel (mit UI)

Zeigt die kumulative Anzahl der Klicks jedes Mal an, wenn auf eine Schaltfläche geklickt wird.

```ts
import { fromEvent } from 'rxjs';
import { scan, tap } from 'rxjs';

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Klicken';
document.body.appendChild(button);

// Ausgabebereich erstellen
const counter = document.createElement('div');
counter.style.marginTop = '10px';
document.body.appendChild(counter);

// Klickereignisse akkumulieren
fromEvent(button, 'click')
  .pipe(
    tap((v) => console.log(v)),
    scan((count) => count + 1, 0)
  )
  .subscribe((count) => {
    counter.textContent = `Klickanzahl: ${count}`;
  });
```

- Bei jedem Klick auf den Button erhöht sich der Zähler um 1.
- Mit `scan` kann **einfache Zähllogik ohne Zustandsverwaltung** geschrieben werden.
