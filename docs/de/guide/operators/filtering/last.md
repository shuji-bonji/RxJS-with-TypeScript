---
description: "Der last-Operator holt beim Abschluss eines Streams nur den letzten Wert oder den letzten Wert, der eine Bedingung erfüllt. Erklärt werden der Unterschied zu first(), Einstellung von Standardwerten, Behandlung von EmptyError und typsichere Implementierung in TypeScript mit praktischen Codebeispielen. Auch der Unterschied zu takeLast() wird vorgestellt."
---

# last - Letzten Wert oder letzten Wert abrufen, der Bedingung erfüllt

Der `last`-Operator ruft aus einem Stream den **letzten Wert** oder den **letzten Wert, der eine Bedingung erfüllt**, ab und beendet den Stream.


## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { last } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

// Nur letzten Wert abrufen
numbers$.pipe(
  last()
).subscribe(console.log);

// Nur letzten Wert abrufen, der Bedingung erfüllt
numbers$.pipe(
  last(n => n < 5)
).subscribe(console.log);

// Ausgabe:
// 5
// 4
```

- `last()` gibt beim Abschluss des Streams den **zuletzt ausgegebenen Wert** aus.
- Wenn eine Bedingung übergeben wird, kann nur **der letzte Wert, der die Bedingung erfüllt**, abgerufen werden.
- Wenn kein Wert die Bedingung erfüllt, tritt ein Fehler auf.

[🌐 RxJS Offizielle Dokumentation - `last`](https://rxjs.dev/api/operators/last)


## 💡 Typische Anwendungsmuster

- Letztes Element gefilterter Daten abrufen
- Neuesten Zustand beim Stream-Abschluss abrufen
- Letzte wichtige Operation aus Sitzungs- oder Operationsprotokoll extrahieren


## 🧠 Praktisches Codebeispiel (mit UI)

Von mehreren eingegebenen Zahlen wird der letzte Wert unter 5 abgerufen und angezeigt, nachdem 5 Eingaben erfolgt sind.

```ts
import { fromEvent } from 'rxjs';
import { map, filter, take, last } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>Praktisches Beispiel für last:</h3>';
document.body.appendChild(output);

// Eingabefeld erstellen
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Zahl eingeben und Enter drücken';
document.body.appendChild(input);

// Eingabeereignisstream
fromEvent<KeyboardEvent>(input, 'keydown')
  .pipe(
    filter((e) => e.key === 'Enter'),
    map(() => parseInt(input.value, 10)),
    take(5), // Nach 5 Eingaben complete
    filter((n) => !isNaN(n) && n < 5), // Nur Werte unter 5 durchlassen
    last() // Letzten Wert unter 5 abrufen
  )
  .subscribe({
    next: (value) => {
      const item = document.createElement('div');
      item.textContent = `Letzter Wert unter 5: ${value}`;
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
1. 5 Mal Zahlen eingeben und Enter drücken
2. Von den eingegebenen Zahlen nur die "unter 5" auswählen
3. Nur die zuletzt eingegebene Zahl unter 5 anzeigen
4. Stream schließt natürlich ab und endet
