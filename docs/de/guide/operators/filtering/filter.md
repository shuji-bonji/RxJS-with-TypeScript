---
description: Der filter-Operator ist ein Filteroperator, der Werte in einem Stream basierend auf einer angegebenen Bedingungsfunktion auswählt und nur Werte durchlässt, die die Bedingung erfüllen. Er rationalisiert Streams durch Ausschluss unnötiger Daten wie Formularvalidierung, Extraktion spezifischer Daten und Ausschluss von null oder undefined. Kann auch als TypeScript Type Guard verwendet werden.
---

# filter - Nur Werte durchlassen, die die Bedingung erfüllen

Der `filter`-Operator wählt Werte in einem Stream basierend auf einer angegebenen Bedingungsfunktion aus und lässt nur Werte durch, die die Bedingung erfüllen.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Ausgabe: 2, 4, 6, 8, 10
```

- Nur Werte, die die Bedingung erfüllen, werden durchgelassen.
- Funktioniert ähnlich wie `Array.prototype.filter()` bei Arrays, wird aber auf einem Observable sequenziell verarbeitet.

[🌐 Offizielle RxJS-Dokumentation - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Typische Anwendungsmuster

- Validierung von Formulareingabewerten
- Nur Daten mit einem bestimmten Typ oder einer bestimmten Struktur zulassen
- Filterung von Sensorereignissen oder Stream-Daten

## 🧠 Praxisbeispiel (mit UI)

Zeigt nur gerade Zahlen in Echtzeit an, wenn sie eingegeben werden.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'Praxisbeispiel für filter:';
document.body.appendChild(title);

// Eingabefeld erstellen
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Zahl eingeben';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Ausgabebereich erstellen
const output = document.createElement('div');
document.body.appendChild(output);

// Eingabeereignis-Stream
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Gerade Zahl erkannt: ${evenNumber}`;
    output.prepend(item);
  });

```

- Nur wenn die Zahl gerade ist, wird sie in der Ausgabe angezeigt.
- Ungerade Zahlen oder ungültige Eingaben werden ignoriert.
