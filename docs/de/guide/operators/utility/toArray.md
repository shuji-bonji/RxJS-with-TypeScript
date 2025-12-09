---
description: toArray ist ein RxJS Utility-Operator, der alle bis zum Abschluss des Observables emittierten Werte in einem Array zusammenfasst und emittiert. Er eignet sich optimal für Batch-Verarbeitung, UI-Anzeige nach Massenabruf oder Aggregationsverarbeitung - Szenarien, in denen der gesamte Stream als Array behandelt werden soll. Da er Werte bis zum Abschluss ansammelt, kann er nicht mit unendlichen Streams verwendet werden.
---

# toArray - Umwandlung von Werten in ein Array

Der `toArray`-Operator ist ein Operator, der **alle bis zum Abschluss des Observables emittierten Werte in einem Array zusammenfasst und emittiert**.
Er ist nützlich für Batch-Verarbeitung, UI-Anzeige nach Massenabruf oder Aggregation.


## 🔰 Grundlegende Syntax und Funktionsweise

```ts
import { of } from 'rxjs';
import { toArray } from 'rxjs';

of(1, 2, 3).pipe(
  toArray()
).subscribe(console.log);

// Ausgabe:
// [1, 2, 3]
```

Alle Werte werden als ein Array zusammengefasst und beim Abschluss des Observables emittiert.

[🌐 RxJS Offizielle Dokumentation - toArray](https://rxjs.dev/api/index/function/toArray)

## 💡 Typische Anwendungsfälle

Kann verwendet werden, wenn Sie mehrere asynchrone Ergebnisse zusammen verarbeiten möchten oder wenn Sie Batch-Ausgabe in der UI durchführen möchten.

```ts
import { interval, of } from 'rxjs';
import { take, toArray, delayWhen, delay } from 'rxjs';

interval(500)
  .pipe(
    take(5),
    delayWhen((val) => of(val).pipe(delay(val * 200))),
    toArray()
  )
  .subscribe((result) => {
    console.log('Zusammen beim Abschluss empfangen:', result);
  });

// Ausgabe:
// Zusammen beim Abschluss empfangen: [0, 1, 2, 3, 4]
```


## 🧪 Praktisches Codebeispiel (mit UI)

```ts
import { interval } from 'rxjs';
import { take, toArray } from 'rxjs';

// Ausgabebereich
const toArrayOutput = document.createElement('div');
toArrayOutput.innerHTML = '<h3>Beispiel für toArray:</h3>';
document.body.appendChild(toArrayOutput);

// Anzeigebereich für einzelne Werte
const individualValues = document.createElement('div');
individualValues.innerHTML = '<h4>Einzelne Werte:</h4>';
toArrayOutput.appendChild(individualValues);

// Anzeigebereich für Array-Ergebnis
const arrayResult = document.createElement('div');
arrayResult.innerHTML = '<h4>Array-Ergebnis:</h4>';
arrayResult.style.marginTop = '20px';
toArrayOutput.appendChild(arrayResult);

// Einzelne Werte abonnieren
interval(500)
  .pipe(take(5))
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Wert: ${val}`;
    individualValues.appendChild(valueItem);
  });

// Denselben Stream als Array abonnieren
interval(500)
  .pipe(take(5), toArray())
  .subscribe((array) => {
    const resultItem = document.createElement('div');
    resultItem.textContent = `Ergebnis-Array: [${array.join(', ')}]`;
    resultItem.style.fontWeight = 'bold';
    resultItem.style.padding = '10px';
    resultItem.style.backgroundColor = '#f5f5f5';
    resultItem.style.borderRadius = '5px';
    arrayResult.appendChild(resultItem);

    // Array-Elemente einzeln anzeigen
    const arrayItems = document.createElement('div');
    arrayItems.style.marginTop = '10px';

    array.forEach((item, index) => {
      const arrayItem = document.createElement('div');
      arrayItem.textContent = `array[${index}] = ${item}`;
      arrayItems.appendChild(arrayItem);
    });

    arrayResult.appendChild(arrayItems);
  });
```


## ✅ Zusammenfassung

- `toArray` **emittiert alle Werte zusammen als Array beim Abschluss**
- Optimal für Szenarien, in denen der gesamte Stream aggregiert behandelt werden soll
- In Kombination mit `concatMap` oder `delay` auch für **Batch-Verarbeitung asynchroner Sequenzen** geeignet
