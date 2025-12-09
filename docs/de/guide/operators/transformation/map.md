---
description: "Der map-Operator ist ein grundlegendes Transformationsmittel, das eine Funktion auf jeden Wert in einem Observable anwendet und neue Werte generiert. Wird häufig für Formformatierung, API-Antwortverarbeitung und Datentransformation verwendet. Erklärung der TypeScript-Typinferenz, Kombination mit anderen Operatoren und Performance-Optimierung."
---

# map - Transformationsfunktion auf jeden Wert anwenden

Der `map`-Operator wendet eine angegebene Funktion auf **jeden Wert** in einem Stream an und erzeugt einen neuen transformierten Wert.
Ähnlich wie die `Array.prototype.map`-Methode für Arrays, arbeitet dieser jedoch auf **asynchronen Streams**.


## 🔰 Grundsyntax und Verwendung

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3).pipe(
  map(value => value * 10)
).subscribe(console.log);
// Ausgabe: 10, 20, 30
```

Wendet die Funktion value => value * 10 auf jeden Wert an und erzeugt neue Werte.

[🌐 RxJS Offizielle Dokumentation - map](https://rxjs.dev/api/index/function/map)


## 💡 Typische Anwendungsmuster
- Transformation von API-Antworten (nur notwendige Eigenschaften extrahieren)
- Formatierung von Formulareingabedaten
- Verarbeitung von Zahlen oder Strings innerhalb eines Streams
- Extraktion nur notwendiger Daten aus UI-Ereignissen


## 🧠 Praktisches Codebeispiel (mit UI)

Ein Beispiel, das eingegebene Zahlen in Echtzeit verdoppelt und anzeigt.

```ts
import { fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Eingabefeld erstellen
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Zahl eingeben';
document.body.appendChild(input);

// Ausgabefeld erstellen
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Eingabeereignis-Stream
fromEvent(input, 'input').pipe(
  map(event => Number((event.target as HTMLInputElement).value)),
  map(value => value * 2)
).subscribe(result => {
  output.textContent = `Verdoppelter Wert: ${result}`;
});
```

- Eingabewerte werden in Echtzeit verdoppelt und ausgegeben.
- Durch aufeinanderfolgendes Anwenden von map wird eine einfache Datentransformationskette realisiert.
