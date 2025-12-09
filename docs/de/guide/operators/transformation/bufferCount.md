---
description: bufferCount ist ein Transformationsoperator von RxJS, der Werte in angegebener Anzahl zusammenfasst und als Array ausgibt. Ideal für anzahlbasierte Stream-Steuerung wie Batch-Verarbeitung, Datenaggregation nach fester Anzahl oder Paketaufteilung für Übertragung. Realisiert typsichere Array-Operationen durch TypeScript-Typinferenz.
---

# bufferCount - Werte in angegebener Anzahl zusammenfassen

Der `bufferCount`-Operator fasst emittierte Werte in angegebener Anzahl **zusammen und gibt sie als Array** aus.
Praktisch für Batch-Verarbeitung nach Anzahl der Werte.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { interval } from 'rxjs';
import { bufferCount } from 'rxjs';

// Werte alle 100ms emittieren
const source$ = interval(100);

source$.pipe(
  bufferCount(5)
).subscribe(buffer => {
  console.log('Werte alle 5 Stück:', buffer);
});

// Ausgabe:
// Werte alle 5 Stück: [0, 1, 2, 3, 4]
// Werte alle 5 Stück: [5, 6, 7, 8, 9]
// ...
```

- Fasst 5 Werte zusammen und gibt sie als Array aus.
- Das Merkmal ist die Zusammenfassung **anzahlbasiert**, nicht zeitbasiert.

[🌐 RxJS Offizielle Dokumentation - `bufferCount`](https://rxjs.dev/api/operators/bufferCount)

## 💡 Typische Anwendungsmuster

- Aufteilung und Übertragung von Datenpaketen
- Batch-Speicherung oder -Verarbeitung nach fester Anzahl
- Aggregation von Eingabeereignissen nach fester Anzahl

## 🧠 Praktisches Codebeispiel (mit UI)

Beispiel, das Tastatureingaben alle 5 Mal zusammenfasst und anzeigt.

```ts
import { fromEvent } from 'rxjs';
import { map, bufferCount } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Tastatureingabe-Ereignisstrom
fromEvent<KeyboardEvent>(document, 'keydown').pipe(
  map(event => event.key),
  bufferCount(5)
).subscribe(keys => {
  const message = `5 Eingaben: ${keys.join(', ')}`;
  console.log(message);
  output.textContent = message;
});
```

- Bei jedem 5. Tastendruck werden diese 5 Tasten zusammen angezeigt.
- Ermöglicht Erfahrung mit anzahlbasierter Aggregationsverarbeitung.
