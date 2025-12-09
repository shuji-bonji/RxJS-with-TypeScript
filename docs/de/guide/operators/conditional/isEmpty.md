---
description: "Der isEmpty-Operator bestimmt, ob ein Observable ohne Emission von Werten abgeschlossen wurde. Wird für Erkennung leerer Daten, bedingte Verzweigung und Datenexistenzprüfungen verwendet. Erläutert die Erkennung leerer Ergebnisse nach filter() und typsichere Implementierung mit TypeScript anhand praktischer Codebeispiele."
---

# isEmpty - Überprüfen, ob ein Stream leer ist

Der `isEmpty`-Operator **emittiert `true`, wenn ein Observable abgeschlossen wird, ohne einen einzigen Wert zu emittieren**.
Wenn auch nur ein Wert emittiert wird, emittiert er `false` und wird abgeschlossen.

## 🔰 Grundlegende Syntax und Verhalten

```ts
import { of, EMPTY } from 'rxjs';
import { isEmpty } from 'rxjs';

EMPTY.pipe(isEmpty()).subscribe(console.log); // Ausgabe: true
of(1).pipe(isEmpty()).subscribe(console.log); // Ausgabe: false
```

[🌐 RxJS Offizielle Dokumentation - isEmpty](https://rxjs.dev/api/index/function/isEmpty)

## 💡 Typische Anwendungsfälle

- Wenn Sie feststellen möchten, ob das Ergebnis einer Filterung oder Suche leer ist
- Wenn Sie im Falle einer leeren Menge einen Fehler ausgeben oder zu einer anderen Verarbeitung wechseln möchten

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

from([1, 3, 5])
  .pipe(
    filter((x) => x % 2 === 0),
    isEmpty()
  )
  .subscribe((result) => {
    console.log('Ist leer:', result);
  });

// Ausgabe:
// Ist leer: true
```

## 🧪 Praktische Codebeispiele (mit UI)

### ✅ 1. Überprüfen, ob das Ergebnis leer ist

```ts
import { from } from 'rxjs';
import { filter, isEmpty } from 'rxjs';

const container = document.createElement('div');
container.innerHTML = '<h3>Beispiel für isEmpty-Operator:</h3>';
document.body.appendChild(container);

const checkButton = document.createElement('button');
checkButton.textContent = 'Auf gerade Zahlen prüfen';
container.appendChild(checkButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
container.appendChild(output);

checkButton.addEventListener('click', () => {
  from([1, 3, 5])
    .pipe(
      filter((x) => x % 2 === 0),
      isEmpty()
    )
    .subscribe((isEmptyResult) => {
      output.textContent = isEmptyResult
        ? 'Keine geraden Zahlen enthalten.'
        : 'Gerade Zahlen enthalten.';
      output.style.color = isEmptyResult ? 'red' : 'green';
    });
});
```

### ✅ 2. Überprüfen, ob Benutzersuchergebnisse leer sind

```ts
import { fromEvent, of, from } from 'rxjs';
import { debounceTime, switchMap, map, filter, isEmpty, delay } from 'rxjs';

const searchContainer = document.createElement('div');
searchContainer.innerHTML = '<h3>Suchergebnisprüfung mit isEmpty:</h3>';
document.body.appendChild(searchContainer);

const input = document.createElement('input');
input.placeholder = 'Suchbegriff eingeben';
input.style.marginBottom = '10px';
searchContainer.appendChild(input);

const resultBox = document.createElement('div');
resultBox.style.padding = '10px';
resultBox.style.border = '1px solid #ccc';
searchContainer.appendChild(resultBox);

const mockData = ['apple', 'banana', 'orange', 'grape'];

fromEvent(input, 'input')
  .pipe(
    debounceTime(300),
    map((e) => (e.target as HTMLInputElement).value.trim().toLowerCase()),
    filter((text) => text.length > 0),
    switchMap((query) =>
      of(mockData).pipe(
        delay(300),
        map((list) => list.filter((item) => item.includes(query))),
        switchMap((filtered) => from(filtered).pipe(isEmpty()))
      )
    )
  )
  .subscribe((noResults) => {
    resultBox.textContent = noResults
      ? 'Keine übereinstimmenden Elemente gefunden'
      : 'Übereinstimmende Elemente gefunden';
    resultBox.style.color = noResults ? 'red' : 'green';
  });
```
