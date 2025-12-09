---
description: mergeAll ist ein Operator, der ein Higher-order Observable (Observable of Observables) empfängt und alle inneren Observables parallel abonniert, um Werte zu vereinfachen.
---

# mergeAll - Alle inneren Observables parallel vereinfachen

Der `mergeAll`-Operator empfängt ein **Higher-order Observable** (Observable of Observables),
**abonniert alle inneren Observables parallel** und vereinfacht die Werte.

## 🔰 Grundlegende Syntax und Verwendung

```ts
import { fromEvent, interval } from 'rxjs';
import { map, mergeAll, take } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

// Bei jedem Klick einen neuen Zähler starten (Higher-order Observable)
const higherOrder$ = clicks$.pipe(
  map(() => interval(1000).pipe(take(3)))
);

// Alle Zähler parallel abonnieren
higherOrder$
  .pipe(mergeAll())
  .subscribe(x => console.log(x));

// Ausgabe (bei 3 Klicks):
// 0 (1. Zähler)
// 1 (1. Zähler)
// 0 (2. Zähler) ← Parallele Ausführung
// 2 (1. Zähler)
// 1 (2. Zähler)
// 0 (3. Zähler) ← Parallele Ausführung
// ...
```

- Jedes innere Observable vom Higher-order Observable wird **parallel abonniert**
- Werte von allen inneren Observables werden **zu einem einzigen Stream verbunden**
- Anzahl paralleler Abonnements kann begrenzt werden (`mergeAll(2)` = maximal 2 parallel)

[🌐 RxJS Official Documentation - `mergeAll`](https://rxjs.dev/api/index/function/mergeAll)

## 💡 Typische Anwendungsmuster

- **Mehrere API-Aufrufe parallel ausführen**
- **Unabhängige Streams bei jeder Benutzeraktion starten**
- **Mehrere Echtzeit-Verbindungen wie WebSocket oder EventSource integrieren**

## 🧠 Praktisches Codebeispiel

Ein Beispiel, das bei jeder Eingabeänderung API-Aufrufe (simuliert) parallel ausführt

```ts
import { fromEvent, of } from 'rxjs';
import { map, mergeAll, delay, debounceTime } from 'rxjs';

const input = document.createElement('input');
input.placeholder = 'Suchbegriff eingeben';
document.body.appendChild(input);

const output = document.createElement('div');
document.body.appendChild(output);

// Eingabeereignis mit Debounce
const search$ = fromEvent(input, 'input').pipe(
  debounceTime(300),
  map((e) => (e.target as HTMLInputElement).value)
);

// Higher-order Observable: Simulierter API-Aufruf für jeden Eingabewert
const results$ = search$.pipe(
  map(query =>
    // Simulierter API-Aufruf (500ms Verzögerung)
    of(`Ergebnis: "${query}"`).pipe(delay(500))
  ),
  mergeAll() // Alle API-Aufrufe parallel ausführen
);

results$.subscribe(result => {
  const item = document.createElement('div');
  item.textContent = result;
  output.prepend(item);
});
```

- Auch wenn der Benutzer die Eingabe schnell ändert, werden **alle API-Aufrufe parallel ausgeführt**
- Alte Suchergebnisse können nach neuen Ergebnissen angezeigt werden (keine Reihenfolgegarantie)

## 🔄 Verwandte Operatoren

| Operator | Beschreibung |
|---|---|
| `mergeMap` | Kurzform von `map` + `mergeAll` (häufig verwendet) |
| [concatAll](./concatAll) | Innere Observables nacheinander abonnieren (auf Abschluss des vorherigen warten) |
| [switchAll](./switchAll) | Zu neuem inneren Observable wechseln (altes abbrechen) |
| [exhaustAll](./exhaustAll) | Neue innere Observables ignorieren, wenn bereits ausgeführt wird |

## ⚠️ Wichtige Hinweise

### Begrenzung der Anzahl paralleler Abonnements

Ohne Begrenzung der Anzahl paralleler Abonnements können Performance-Probleme auftreten.

```ts
// Anzahl paralleler Abonnements auf 2 begrenzen
higherOrder$.pipe(
  mergeAll(2) // Maximal 2 parallel ausführen
).subscribe();
```

### Keine Reihenfolgegarantie

`mergeAll` führt parallel aus, daher **ist die Reihenfolge der Werte nicht garantiert**.
Wenn Reihenfolge wichtig ist, verwenden Sie [concatAll](./concatAll).
