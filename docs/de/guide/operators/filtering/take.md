---
description: Der take-Operator holt die angegebene Anzahl von Werten vom Anfang eines Observable-Streams und ignoriert alle nachfolgenden Werte, wobei der Stream automatisch abgeschlossen wird. Nützlich, wenn Sie nur die ersten paar Datensätze extrahieren möchten.
---

# take - Nur die ersten N Werte abrufen

Der `take`-Operator holt **die angegebene Anzahl** von Werten vom Anfang eines Streams und ignoriert alle nachfolgenden Werte.
Nach dem Abschluss wird auch der Stream automatisch `complete`.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

const source$ = interval(1000);

source$.pipe(
  take(3)
).subscribe(console.log);
// Ausgabe: 0, 1, 2
```

- Holt nur die ersten 3 Werte und abonniert sie.
- Nach dem Abrufen von 3 Werten wird das Observable automatisch `complete`.

[🌐 Offizielle RxJS-Dokumentation - `take`](https://rxjs.dev/api/operators/take)

## 💡 Typische Anwendungsmuster

- Nur die ersten paar Einträge für UI-Anzeige oder Protokollierung aufzeichnen
- Temporäres Abonnement, um nur die erste Antwort zu extrahieren
- Begrenzter Abruf von Testdaten oder Demodaten

## 🧠 Praxisbeispiel (mit UI)

Holt und zeigt nur die ersten 5 Werte an, die jede Sekunde ausgegeben werden.

```ts
import { interval } from 'rxjs';
import { take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Praxisbeispiel für take:</h3>';
document.body.appendChild(output);

// Gibt jede Sekunde einen Wert aus
const source$ = interval(1000);

// Holt nur die ersten 5 Werte
source$.pipe(take(5)).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `Wert: ${value}`;
    output.appendChild(item);
  },
  complete: () => {
    const complete = document.createElement('div');
    complete.textContent = 'Abgeschlossen';
    complete.style.fontWeight = 'bold';
    output.appendChild(complete);
  },
});

```

- Die ersten 5 Werte (`0`, `1`, `2`, `3`, `4`) werden nacheinander angezeigt,
- Danach wird die Meldung "Abgeschlossen" angezeigt.
