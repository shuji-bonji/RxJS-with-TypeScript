---
description: Der first-Operator ruft aus einem Stream nur den ersten Wert oder den ersten Wert ab, der eine angegebene Bedingung erfüllt, und beendet dann den Stream. Praktisch, wenn Sie nur das erste Ereignis verarbeiten oder Initialdaten abrufen möchten.
---

# first - Ersten Wert oder ersten Wert abrufen, der Bedingung erfüllt

Der `first`-Operator ruft aus einem Stream den **ersten Wert** oder den **ersten Wert, der eine Bedingung erfüllt**, ab und beendet den Stream.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { first } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Nur ersten Wert abrufen
numbers$.pipe(
  first()
).subscribe(console.log);

// Nur ersten Wert abrufen, der Bedingung erfüllt
numbers$.pipe(
  first(n => n > 3)
).subscribe(console.log);

// Ausgabe:
// 1
// 4
```

- `first()` ruft den ersten Wert ab, der fließt, und beendet.
- Wenn eine Bedingung übergeben wird, wird **der erste Wert, der die Bedingung erfüllt**, abgerufen.
- Wenn kein Wert die Bedingung erfüllt, wird ein Fehler ausgel&oum

l;st.

[🌐 RxJS Offizielle Dokumentation - `first`](https://rxjs.dev/api/operators/first)


## 💡 Typische Anwendungsmuster

- Nur das erste eintreffende Ereignis verarbeiten
- Erste Daten erkennen, die eine Bedingung erfüllen (z.B. Score von 5 oder höher)
- Nur erste Daten vor Timeout oder Abbruch übernehmen

## 🧠 Praktisches Codebeispiel (mit UI)

Auch bei mehrfachem Klicken des Buttons wird **nur der erste Klick verarbeitet**.

```ts
import { fromEvent } from 'rxjs';
import { first } from 'rxjs';

const title = document.createElement('div');
title.innerHTML = '<h3>Praktisches Beispiel für first:</h3>';
document.body.appendChild(title);

// Button erstellen
const button = document.createElement('button');
button.textContent = 'Bitte klicken (reagiert nur beim ersten Mal)';
document.body.appendChild(button);

// Ausgabebereich erstellen
let count = 0;
const output = document.createElement('div');
document.body.appendChild(output);
// Button-Click-Stream
fromEvent(button, 'click')
  .pipe(first())
  .subscribe(() => {
    const message = document.createElement('div');
    count++;
    message.textContent = `Ersten Klick erkannt! ${count}`;
    output.appendChild(message);
  });
```

- Nur das erste Click-Ereignis wird empfangen, danach wird es ignoriert.
- Der Stream wird automatisch nach dem ersten Klick `complete`.
