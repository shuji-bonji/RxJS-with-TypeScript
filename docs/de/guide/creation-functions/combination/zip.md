---
description: "Die zip-Erstellungsfunktion ordnet und paart Werte aus mehreren Observables in der entsprechenden Reihenfolge und gibt sie zu dem Zeitpunkt aus, zu dem alle Quellen ihre Werte nacheinander veröffentlicht haben. Sie kann verwendet werden, um Daten zu synchronisieren und die Ergebnisse der parallelen Verarbeitung zu kombinieren."
---

# zip - Entsprechende Werte paaren

`zip` ist eine Erstellungsfunktion, die **entsprechende geordnete Werte**, die von mehreren Observables ausgegeben werden, zusammenfasst und als Array oder Tupel ausgibt.
Sie wartet darauf, dass die Werte nacheinander von allen Quell-Observables eintreffen und erstellt Paare, wenn sie bereit sind.

## Grundlegende Syntax und Verwendung

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

const source1$ = of('A', 'B', 'C');
const source2$ = interval(1000).pipe(
  map((val) => val * 10),
  take(3)
);

zip(source1$, source2$).subscribe(([letter, number]) => {
  console.log(letter, number);
});

// Ausgabe:
// A 0
// B 10
// C 20
```

- Ein Paar wird zu dem Zeitpunkt erstellt und ausgegeben, zu dem jede Observable einen Wert ausgegeben hat.
- Wenn einer der Werte verzögert ist, wird gewartet, bis beide ausgerichtet sind.

[🌐 Offizielle RxJS-Dokumentation - zip](https://rxjs.dev/api/index/function/zip)

## Typische Nutzungsmuster

- **Anfrage und Antwort zuordnen**
- **IDs synchron mit entsprechenden Daten paaren**
- **Mehrere parallel verarbeitete Streams zu einem einzigen Satz zusammenfassen**

## Praktisches Code-Beispiel (mit UI)

Beispiel für die **Kombination und Anzeige** verschiedener Datenquellen (Obst und Preis).

```ts
import { zip, of, interval } from 'rxjs';
import { map, take } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>zip Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Obst-Stream
const fruits$ = of('🍎 Apple', '🍌 Banana', '🍇 Grape');

// Preis-Stream (alle 2 Sekunden ausgeben)
const prices$ = interval(2000).pipe(
  map((i) => [100, 200, 300][i]),
  take(3)
);

// Mit zip kombinieren und anzeigen
zip(fruits$, prices$).subscribe(([fruit, price]) => {
  const item = document.createElement('div');
  item.textContent = `${fruit} - €${price}`;
  output.appendChild(item);
});
```

- Obst- und Preislisten werden als Paar angezeigt, wenn sie in einer **Eins-zu-Eins-Entsprechung** ausgerichtet sind.
- Wenn eine der beiden Listen fehlt, wird zu diesem Zeitpunkt nichts ausgegeben.

## Verwandte Operatoren

- **[zipWith](/de/guide/operators/combination/zipWith)** - Pipeable Operator Version (wird in der Pipeline verwendet)
- **[combineLatest](/de/guide/creation-functions/combination/combineLatest)** - Kombiniert die neuesten Werte (Erstellungsfunktion)
- **[withLatestFrom](/de/guide/operators/combination/withLatestFrom)** - Wird nur durch den Hauptstrom ausgelöst
